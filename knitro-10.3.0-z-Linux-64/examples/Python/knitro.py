#*******************************************************
#* Copyright (c) 2017 by Artelys                       *
#* All Rights Reserved                                 *
#*******************************************************

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  Artelys Knitro 10.3.0 Python API
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


'''Python interface for Artelys Knitro.

This module provides a Python interface for the Artelys Knitro solver.
Multiple instances of the Knitro context can be constructed and run
simultaneously in separate threads.  Each instance of Knitro context
allocates a distinct Knitro instance in C.

Applications should create a Knitro context and call its
methods, similar to Python code in examples/Python.

This source is provided for informational purposes.
'''

import sys
import itertools
import ctypes

#------------------------------------------------------------------
#     Python 2 / Python 3 compatibility handling
#------------------------------------------------------------------

try:
    xrange
except NameError:
    xrange = range

#------------------------------------------------------------------
#     VERSION NUMBER
#------------------------------------------------------------------
__version__ = '10.3.0'


#------------------------------------------------------------------
#     STATIC LOAD METHOD FOR THE NATIVE LIBRARY
#------------------------------------------------------------------
try:
    if sys.platform == 'win32': # Windows
        _knitroLibraryFile = "knitro.dll"
        _knitro = ctypes.windll.LoadLibrary (_knitroLibraryFile)
    elif sys.platform == 'darwin': # Mac OS X
        _knitroLibraryFile = "libknitro.dylib"
        _knitro = ctypes.cdll.LoadLibrary (_knitroLibraryFile)
    else: # Linux, Solaris
        _knitroLibraryFile = "libknitro.so"
        _knitro = ctypes.cdll.LoadLibrary (_knitroLibraryFile)
except OSError as e:
    print ("Failed to load the Artelys Knitro native library '" + _knitroLibraryFile + "'.")
    print ("Make sure that your environment variables are set properly.")
    print
    raise


#------------------------------------------------------------------
#     METHODS AND CLASSES TO WORK WITH C ARRAYS AND POINTERS
#------------------------------------------------------------------

class KTR_array_handler:
    '''Handler class for user-provided and C arrays.
    '''

    @staticmethod
    def _cIntArray (pyArray):
        '''Construct a 'ctypes' array of ints from a Python list.
        '''
        if pyArray is not None:
            n = len (pyArray)
            if n > 0:
                return (ctypes.c_int * n) (*pyArray)
        return None

    @staticmethod
    def _cDoubleArray (pyArray):
        '''Construct a 'ctypes' array of doubles from a Python list.
        '''
        if pyArray is not None:
            n = len (pyArray)
            if n > 0:
                return (ctypes.c_double * n) (*pyArray)
        return None

    @staticmethod
    def _cStringArray (pyArray):
        '''Construct a 'ctypes' array of strings (char*) from a Python list.
        '''
        if pyArray is not None:
            n = len (pyArray)
            if n > 0:
                pyArrayCopy = map (lambda x: x.encode('UTF-8'),  pyArray)
                return (ctypes.c_char_p * n) (*pyArrayCopy)
        return None

    @staticmethod
    def _userArray (size, cArray):
        '''Construct a Python list from a 'ctypes' array.
        '''
        if cArray:
            return [cArray[i] for i in xrange (size)]
        else:
            return None

    @staticmethod
    def _userToCArray (pyArray, cArray):
        '''Copy the content of a Python list to a 'ctypes' array.
        '''
        if pyArray is not None:
            for i in xrange (len (pyArray)):
                cArray[i] = pyArray[i]

    @staticmethod
    def _cToUserArray (size, cArray, pyArray):
        '''Copy the content a 'ctypes' array to a Python list.
        '''
        if cArray:
            pyArray[:] = cArray

def _cIntArray (userArray):
    return KTR_array_handler._cIntArray (userArray)

def _cDoubleArray (userArray):
    return KTR_array_handler._cDoubleArray (userArray)

def _cStringArray (userArray):
    return KTR_array_handler._cStringArray (userArray)

def _userArray (size, cArray):
    return KTR_array_handler._userArray (size, cArray)

def _userToCArray (userArray, cArray):
    KTR_array_handler._userToCArray (userArray, cArray)

def _cToUserArray (size, cArray, userArray):
    KTR_array_handler._cToUserArray (size, cArray, userArray)

#------------------------------------------------------------------
#     DEFINITIONS OF KNITRO INTERNAL ctypes.StructureS
#------------------------------------------------------------------

class KTR_context (ctypes.Structure):
    '''Wrapper for KTR_context ctypes.Structure
    '''
    _fields_ = []
    def __str__(self):
      return "KTR_context"
KTR_context_ptr = ctypes.POINTER(KTR_context)
KTR_context_ptr.__str__ = lambda self: "KTR_context_ptr (" + repr (self.contents) + ")"
KTR_context_ptr.__hash__ = lambda self: str (self.contents).__hash__()

class ZLM_context (ctypes.Structure):
    '''Wrapper for ZLM_context ctypes.Structure
    '''
    _fields_ = []
    def __str__(self):
      return "ZLM_context"
ZLM_context_ptr = ctypes.POINTER(ZLM_context)
ZLM_context_ptr.__str__ = lambda self: "ZLM_context_ptr (" + repr (self.contents) + ")"


#------------------------------------------------------------------
#     PRIVATE METHODS FOR HANDLING USER CALLBACKS AND PARAMETERS
#------------------------------------------------------------------

#---- Stores a list for each KTR_context object, each list storing user parameters (first position) and user callbacks (from second position)
_callbacks = dict ()

def _registerCallback (kc, fnPtr):
    '''Register callback fnPtr into KTR_context kc
    
    This method makes sure that fnPtr is kept alive during the whole
    optimization process and only destroyed upon calling KTR_free.
    '''
    if kc not in _callbacks:
        _callbacks[kc] = [None]
    _callbacks[kc].append (fnPtr)

def _registerUserParams (kc, userParams):
    '''Register user parameters userParams into KTR_context kc
    
    This method makes sure that userParams is kept alive during the whole
    optimization process and only destroyed upon calling KTR_free.
    '''
    if userParams is not None:
        if kc not in _callbacks:
            _callbacks[kc] = [userParams]
        else:
            _callbacks[kc][0] = userParams

def _unregisterAll (kc):
    '''Unregister all user callbacks and parameters from KTR_context kc
    '''
    if kc in _callbacks:
        del _callbacks[kc]

def _getUserParams (kc):
    '''Retrieve the user parameters stored in KTR_context kc
    '''
    if kc in _callbacks:
        return _callbacks[kc][0]
    return None


#------------------------------------------------------------------
#     PRIVATE DEFINITIONS AND WRAPPERS FOR USER CALLBACKS
#------------------------------------------------------------------

#---- SIGINT handler
class _signal_handler (object):
    """ This class handles SIGINT to return proper
        KTR_RC_USER_TERMINATION to Knitro on interrupt.
    """
    def __init__ (self, fnPtr):
        self._fnPtr = fnPtr
        
    def call(self, *args, **kwargs):
        try:
            return self._fnPtr(*args, **kwargs)
        except KeyboardInterrupt:
            return KTR_RC_USER_TERMINATION

#---- KTR_puts
_KTR_puts = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.c_char_p, ctypes.c_void_p
)
class _KTR_puts_wrapper (object):
    def __init__ (self, kc, fnPtr):
        self._kc = kc
        self._fnPtr = fnPtr
        sig_handler = _signal_handler (self.call)
        self.c_fnPtr = _KTR_puts (sig_handler.call)
    def set_KTR_context_ptr (self, kc):
        self._kc = kc
    def call (self, str, userParams):
        return self._fnPtr (str.decode('UTF-8'), _getUserParams (self._kc))

#---- KTR_callback
_KTR_callback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_void_p
)
class _KTR_callback_wrapper (object):
    def __init__ (self, kc, fnPtr):
        self._kc = kc
        self._fnPtr = fnPtr
        sig_handler = _signal_handler (self.call)
        self.c_fnPtr = _KTR_callback (sig_handler.call)
    def set_KTR_context_ptr (self, kc):
        self._kc = kc
    def call (
        self,
        evalRequestCode, n, m, nnzJ, nnzH,
        c_x, c_lambda,
        c_obj, c_c,
        c_objGrad, c_jac,
        c_hessian, c_hessVector,
        c_userParams
    ):
        x = _userArray (n, c_x)
        lambda_ = _userArray (n+m, c_lambda)
        obj = _userArray(1, c_obj)
        c = _userArray (m, c_c)
        objGrad = _userArray (n, c_objGrad)
        jac = _userArray (nnzJ, c_jac)
        hessian = _userArray (nnzH, c_hessian)
        hessVector = _userArray (n, c_hessVector)
        ret = self._fnPtr (
            evalRequestCode,
            n, m, nnzJ, nnzH,
            x, lambda_,
            obj, c,
            objGrad, jac,
            hessian, hessVector,
            _getUserParams (self._kc)
        )
        _userToCArray (obj, c_obj)
        _userToCArray (c, c_c)
        _userToCArray (objGrad, c_objGrad)
        _userToCArray (jac, c_jac)
        _userToCArray (hessian, c_hessian)
        _userToCArray (hessVector, c_hessVector)
        return ret

#---- KTR_lsq_callback
_KTR_lsq_callback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_void_p
)
class _KTR_lsq_callback_wrapper (object):
    def __init__ (self, kc, fnPtr):
        self._kc = kc
        self._fnPtr = fnPtr
        sig_handler = _signal_handler (self.call)
        self.c_fnPtr = _KTR_lsq_callback (sig_handler.call)
    def set_KTR_context_ptr (self, kc):
        self._kc = kc
    def call (
        self,
        n, m, nnzJ,
        c_x,
        c_res,
        c_jac,
        c_userParams
    ):
        x = _userArray (n, c_x)
        res = _userArray (m, c_res)
        jac = _userArray (nnzJ, c_jac)
        ret = self._fnPtr (
            n, m, nnzJ,
            x,
            res,
            jac,
            _getUserParams (self._kc)
        )
        _userToCArray (res, c_res)
        _userToCArray (jac, c_jac)
        return ret

#---- KTR_newpt_callback
_KTR_newpt_callback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    KTR_context_ptr,
    ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_double, ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_void_p
)
class _KTR_newpt_callback_wrapper (object):
    def __init__ (self, kc, fnPtr):
        self._kc = kc
        self._fnPtr = fnPtr
        sig_handler = _signal_handler (self.call)
        self.c_fnPtr = _KTR_newpt_callback (sig_handler.call)
    def set_KTR_context_ptr (self, kc):
        self._kc = kc
    def call (
        self,
        kc,
        n, m, nnzJ,
        c_x, c_lambda,
        obj, c_c,
        c_objGrad, c_jac,
        c_userParams
    ):
        x = _userArray (n, c_x)
        lambda_ = _userArray (n+m, c_lambda)
        c = _userArray (m, c_c)
        objGrad = _userArray (n, c_objGrad)
        jac = _userArray (nnzJ, c_jac)
        ret = self._fnPtr (
            kc,
            n, m, nnzJ,
            x, lambda_,
            obj, c,
            objGrad, jac,
            _getUserParams (self._kc)
        )
        return ret

#---- KTR_ms_initpt_callback
_KTR_ms_initpt_callback = ctypes.CFUNCTYPE(
    ctypes.c_int,
    ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_void_p
)
class _KTR_ms_initpt_callback_wrapper (object):
    def __init__ (self, kc, fnPtr):
        self._kc = kc
        self._fnPtr = fnPtr
        sig_handler = _signal_handler (self.call)
        self.c_fnPtr = _KTR_ms_initpt_callback (sig_handler.call)
    def set_KTR_context_ptr (self, kc):
        self._kc = kc
    def call (
        self,
        nSolveNumber, n, m,
        c_xLoBnds, c_xUpBnds,
        c_x, c_lambda,
        userParams
    ):
        xLoBnds = _userArray (n, c_xLoBnds)
        xUpBnds = _userArray (n, c_xUpBnds)
        x = _userArray (n, c_x)
        lambda_ = _userArray (n+m, c_lambda)
        ret = self._fnPtr (
            nSolveNumber,
            n, m,
            xLoBnds, xUpBnds,
            x, lambda_,
            _getUserParams (self._kc)
        )
        _userToCArray (x, c_x)
        _userToCArray (lambda_, c_lambda)
        return ret


#------------------------------------------------------------------
#     METHODS FOR CREATING AND DESTROYING KNITRO SOLVER OBJECTS
#------------------------------------------------------------------

def KTR_new_init (kc):
    if KTR_set_int_param(kc, KTR_PARAM_PAR_CONCURRENT_EVALS, KTR_PAR_CONCURRENT_EVALS_NO):
        raise RuntimeError ("Error while initializing parameter 'par_concurrent_evals' to 'no'")

#---- KTR_new
_knitro.KTR_new.argtypes = []
_knitro.KTR_new.restype = KTR_context_ptr
def KTR_new ():
    kc = _knitro.KTR_new ()
    if kc:
        KTR_new_init (kc)
    return kc

#---- KTR_new_puts
_knitro.KTR_new_puts.argtypes = [_KTR_puts, ctypes.c_void_p]
_knitro.KTR_new_puts.restype = KTR_context_ptr
def KTR_new_puts (fnPtr, userParams):
    if fnPtr is not None:
        fnPtrWrapper = _KTR_puts_wrapper (None, fnPtr)
        kc = _knitro.KTR_new_puts (fnPtrWrapper.c_fnPtr, None)
        if kc:
            KTR_new_init (kc)
            fnPtrWrapper.set_KTR_context_ptr (kc)
            _registerCallback (kc, fnPtrWrapper)
            _registerUserParams (kc, userParams)
    else:
        kc = _knitro.KTR_new_puts (ctypes.cast(None, _KTR_puts), None)
        if kc:
            KTR_new_init (kc)
            _registerUserParams (kc, userParams)
    return kc

#---- KTR_free
_knitro.KTR_free.argtypes = [ctypes.POINTER(KTR_context_ptr)]
_knitro.KTR_free.restype = ctypes.c_int
def KTR_free (kc_handle):
    _unregisterAll (kc_handle)
    return _knitro.KTR_free (ctypes.byref (kc_handle))


#------------------------------------------------------------------
#     METHODS FOR CREATING AND DESTROYING KNITRO SOLVER OBJECTS IN HIGH VOLUME
#------------------------------------------------------------------

#---- ZLM_checkout_license
_knitro.ZLM_checkout_license.argtypes = []
_knitro.ZLM_checkout_license.restype = ZLM_context_ptr
def ZLM_checkout_license ():
    return _knitro.ZLM_checkout_license ()

#---- KTR_new_zlm
_knitro.KTR_new_zlm.argtypes = [_KTR_puts, ctypes.c_void_p, ZLM_context_ptr]
_knitro.KTR_new_zlm.restype = KTR_context_ptr
def KTR_new_zlm (fnPtr, userParams, pZLMcontext):
    if fnPtr is not None:
        fnPtrWrapper = _KTR_puts_wrapper (None, fnPtr)
        kc = _knitro.KTR_new_zlm (fnPtrWrapper.c_fnPtr, userParams, pZLMcontext)
        if kc:
            KTR_new_init (kc)
            fnPtrWrapper.set_KTR_context_ptr (kc)
            _registerCallback (kc, fnPtrWrapper)
            _registerUserParams (kc, userParams)
    else:
        kc = _knitro.KTR_new_zlm (ctypes.cast(None, _KTR_puts), userParams, pZLMcontext)
        if kc:
            KTR_new_init (kc)
            _registerUserParams (kc, userParams)
    return kc

#---- ZLM_release_license
_knitro.ZLM_release_license.argtypes = [ZLM_context_ptr]
_knitro.ZLM_release_license.restype = ctypes.c_int
def ZLM_release_license (pZLMcontext):
    return _knitro.ZLM_release_license (pZLMcontext)


#------------------------------------------------------------------
#     METHODS FOR CHANGING AND READING SOLVER PARAMETERS
#------------------------------------------------------------------

#---- KTR_reset_params_to_defaults
_knitro.KTR_reset_params_to_defaults.argtypes = [KTR_context_ptr]
_knitro.KTR_reset_params_to_defaults.restype = ctypes.c_int
def KTR_reset_params_to_defaults (kc):
    return _knitro.KTR_reset_params_to_defaults (kc)

#---- KTR_load_param_file
_knitro.KTR_load_param_file.argtypes = [KTR_context_ptr, ctypes.c_char_p]
_knitro.KTR_load_param_file.restype = ctypes.c_int
def KTR_load_param_file (kc, filename):
    return _knitro.KTR_load_param_file (kc, filename.encode('UTF-8'))

#---- KTR_save_param_file
_knitro.KTR_save_param_file.argtypes = [KTR_context_ptr, ctypes.c_char_p]
_knitro.KTR_save_param_file.restype = ctypes.c_int
def KTR_save_param_file (kc, filename):
    return _knitro.KTR_save_param_file (kc, filename.encode('UTF-8'))

#---- KTR_set_int_param_by_name
_knitro.KTR_set_int_param_by_name.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.c_int]
_knitro.KTR_set_int_param_by_name.restype = ctypes.c_int
def KTR_set_int_param_by_name (kc, name, value):
    return _knitro.KTR_set_int_param_by_name (kc, name.encode('UTF-8'), value)

#---- KTR_set_char_param_by_name
_knitro.KTR_set_char_param_by_name.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.c_char_p]
_knitro.KTR_set_char_param_by_name.restype = ctypes.c_int
def KTR_set_char_param_by_name (kc, name, value):
    return _knitro.KTR_set_char_param_by_name (kc, name.encode('UTF-8'), value.encode('UTF-8'))

#---- KTR_set_double_param_by_name
_knitro.KTR_set_double_param_by_name.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.c_double]
_knitro.KTR_set_double_param_by_name.restype = ctypes.c_int
def KTR_set_double_param_by_name (kc, name, value):
    return _knitro.KTR_set_double_param_by_name (kc, name.encode('UTF-8'), ctypes.c_double (value))

#---- KTR_set_param_by_name
_knitro.KTR_set_param_by_name.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.c_double]
_knitro.KTR_set_param_by_name.restype = ctypes.c_int
def KTR_set_param_by_name (kc, name, value):
    return _knitro.KTR_set_param_by_name (kc, name.encode('UTF-8'), ctypes.c_double (value))

#---- KTR_set_int_param
_knitro.KTR_set_int_param.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_int]
_knitro.KTR_set_int_param.restype = ctypes.c_int
def KTR_set_int_param (kc, param_id, value):
    return _knitro.KTR_set_int_param (kc, param_id, value)

#---- KTR_set_char_param
_knitro.KTR_set_char_param.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_char_p]
_knitro.KTR_set_char_param.restype = ctypes.c_int
def KTR_set_char_param (kc, param_id, value):
    return _knitro.KTR_set_char_param (kc, param_id, value.encode('UTF-8'))

#---- KTR_set_double_param
_knitro.KTR_set_double_param.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_double]
_knitro.KTR_set_double_param.restype = ctypes.c_int
def KTR_set_double_param (kc, param_id, value):
    return _knitro.KTR_set_double_param (kc, param_id, ctypes.c_double (value))

#---- KTR_get_int_param_by_name
_knitro.KTR_get_int_param_by_name.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_get_int_param_by_name.restype = ctypes.c_int
def KTR_get_int_param_by_name (kc, name, value):
    if len (value) == 0:
        value.append (0)
    c_value = ctypes.c_int (int (value[0]))
    ret = _knitro.KTR_get_int_param_by_name (kc, name.encode('UTF-8'), ctypes.byref (c_value))
    value[0] = c_value.value
    return ret

#---- KTR_get_double_param_by_name
_knitro.KTR_get_double_param_by_name.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_double_param_by_name.restype = ctypes.c_int
def KTR_get_double_param_by_name (kc, name, value):
    if len (value) == 0:
        value.append (0)
    c_value = ctypes.c_double (value[0])
    ret = _knitro.KTR_get_double_param_by_name (kc, name.encode('UTF-8'), ctypes.byref (c_value))
    value[0] = c_value.value
    return ret

#---- KTR_get_int_param
_knitro.KTR_get_int_param.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_get_int_param.restype = ctypes.c_int
def KTR_get_int_param (kc, param_id, value):
    if len (value) == 0:
        value.append (0)
    c_value = ctypes.c_int (int (value[0]))
    ret = _knitro.KTR_get_int_param (kc, param_id, ctypes.byref (c_value))
    value[0] = c_value.value
    return ret

#---- KTR_get_double_param
_knitro.KTR_get_double_param.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_double_param.restype = ctypes.c_int
def KTR_get_double_param (kc, param_id, value):
    if len (value) == 0:
        value.append (0)
    c_value = ctypes.c_double (value[0])
    ret = _knitro.KTR_get_double_param (kc, param_id, ctypes.byref (c_value))
    value[0] = c_value.value
    return ret

#---- KTR_get_param_name
_knitro.KTR_get_param_name.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t]
_knitro.KTR_get_param_name.restype = ctypes.c_int
def KTR_get_param_name (kc, param_id, param_name, output_size):
    if len (param_name) == 0:
        param_name.append (" "*output_size)
    c_param_name = ctypes.c_char_p (param_name[0].encode('UTF-8'))
    ret = _knitro.KTR_get_param_name (kc, param_id, c_param_name, output_size)
    param_name[0] = c_param_name.value.decode('UTF-8')
    return ret

#---- KTR_get_param_doc
_knitro.KTR_get_param_doc.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t]
_knitro.KTR_get_param_doc.restype = ctypes.c_int
def KTR_get_param_doc (kc, param_id, description, output_size):
    if len (description) == 0:
        description.append (" "*output_size)
    c_description = ctypes.c_char_p (description[0].encode('UTF-8'))
    ret = _knitro.KTR_get_param_doc (kc, param_id, c_description, output_size)
    description[0] = c_description.value.decode('UTF-8')
    return ret

#---- KTR_get_param_type
_knitro.KTR_get_param_type.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_get_param_type.restype = ctypes.c_int
def KTR_get_param_type (kc, param_id, param_type):
    if len (param_type) == 0:
        param_type.append (0)
    c_param_type = ctypes.c_int (param_type[0])
    ret = _knitro.KTR_get_param_type (kc, param_id, ctypes.byref (c_param_type))
    param_type[0] = c_param_type.value
    return ret

#---- KTR_get_num_param_values
_knitro.KTR_get_num_param_values.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_get_num_param_values.restype = ctypes.c_int
def KTR_get_num_param_values (kc, param_id, num_param_values):
    if len (num_param_values) == 0:
        num_param_values.append (0)
    c_num_param_values = ctypes.c_int (num_param_values[0])
    ret = _knitro.KTR_get_num_param_values (kc, param_id, ctypes.byref (c_num_param_values))
    num_param_values[0] = c_num_param_values.value
    return ret

#---- KTR_get_param_value_doc
_knitro.KTR_get_param_value_doc.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t]
_knitro.KTR_get_param_value_doc.restype = ctypes.c_int
def KTR_get_param_value_doc (kc, param_id, value_id, param_value_string, output_size):
    if len (param_value_string) == 0:
        param_value_string.append (" "*output_size)
    c_param_value_string = ctypes.c_char_p (param_value_string[0].encode('UTF-8'))
    ret = _knitro.KTR_get_param_value_doc (kc, param_id, value_id, c_param_value_string, output_size)
    param_value_string[0] = c_param_value_string.value.decode('UTF-8')
    return ret

#---- KTR_get_param_id
_knitro.KTR_get_param_id.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_get_param_id.restype = ctypes.c_int
def KTR_get_param_id (kc, name, param_id):
    if len (param_id) == 0:
        param_id.append (0)
    c_param_id = ctypes.c_int (param_id[0])
    ret = _knitro.KTR_get_param_id (kc, name.encode('UTF-8'), ctypes.byref (c_param_id))
    param_id[0] = c_param_id.value
    return ret

#---- KTR_load_tuner_file
_knitro.KTR_load_tuner_file.argtypes = [KTR_context_ptr, ctypes.c_char_p]
_knitro.KTR_load_tuner_file.restype = ctypes.c_int
def KTR_load_tuner_file (kc, filename):
    return _knitro.KTR_load_tuner_file (kc, filename.encode('UTF-8'))

#---- KTR_get_release
_knitro.KTR_get_release.argtypes = [ctypes.c_int, ctypes.c_char_p]
_knitro.KTR_get_release.restype = None
def KTR_get_release (size, release):
    if len (release) == 0:
        release.append (" "*size)
    c_release = ctypes.c_char_p (release[0].encode('UTF-8')) # TODO .encode('UTF-8')
    _knitro.KTR_get_release (size, c_release)
    release[0] = c_release.value.decode('UTF-8')

#---- KTR_set_feastols
_knitro.KTR_set_feastols.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_set_feastols.restype = ctypes.c_int
def KTR_set_feastols (kc, cFeasTols, xFeasTols, ccFeasTols):
    return _knitro.KTR_set_feastols (kc, _cDoubleArray (cFeasTols), _cDoubleArray (xFeasTols), _cDoubleArray (ccFeasTols))

#---- KTR_set_var_scalings
_knitro.KTR_set_var_scalings.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_set_var_scalings.restype = ctypes.c_int
def KTR_set_var_scalings (kc, xScaleFactors, xScaleCenters):
    return _knitro.KTR_set_var_scalings (kc, _cDoubleArray (xScaleFactors), _cDoubleArray (xScaleCenters))

#---- KTR_set_con_scalings
_knitro.KTR_set_con_scalings.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_set_con_scalings.restype = ctypes.c_int
def KTR_set_con_scalings (kc, cScaleFactors, cScaleCenters):
    return _knitro.KTR_set_con_scalings (kc, _cDoubleArray (cScaleFactors), _cDoubleArray (cScaleCenters))

#---- KTR_set_obj_scaling
_knitro.KTR_set_obj_scaling.argtypes = [KTR_context_ptr, ctypes.c_double]
_knitro.KTR_set_obj_scaling.restype = ctypes.c_int
def KTR_set_obj_scaling (kc, objScaleFactor):
    return _knitro.KTR_set_obj_scaling (kc, ctypes.c_double (objScaleFactor))

#---- KTR_set_names
_knitro.KTR_set_names.argtypes = [KTR_context_ptr, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.c_char_p)]
_knitro.KTR_set_names.restype = ctypes.c_int
def KTR_set_names (kc, objName, varNames, conNames):
    _knitro.KTR_set_names (kc, objName.encode('UTF-8'), _cStringArray(varNames), _cStringArray(conNames)) # TODO .encode('UTF-8')

#---- KTR_set_linearvars
_knitro.KTR_set_linearvars.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_set_linearvars.restype = ctypes.c_int
def KTR_set_linearvars (kc, linearVars):
    _knitro.KTR_set_linearvars (kc, _cIntArray(linearVars))

#---- KTR_set_honorbnds
_knitro.KTR_set_honorbnds.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_set_honorbnds.restype = ctypes.c_int
def KTR_set_honorbnds (kc, honorbnds):
    _knitro.KTR_set_honorbnds (kc, _cIntArray(honorbnds))


#------------------------------------------------------------------
#     METHODS FOR PROBLEM INITIALIZATION
#------------------------------------------------------------------

#---- KTR_init_problem
_knitro.KTR_init_problem.argtypes = [
    KTR_context_ptr,
    ctypes.c_int,
    ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)
]
_knitro.KTR_init_problem.restype = ctypes.c_int
def KTR_init_problem (
    kc,
    n,
    objGoal, objType,
    xLoBnds, xUpBnds,
    cType, cLoBnds, cUpBnds,
    jacIndexVars, jacIndexCons,
    hessIndexRows, hessIndexCols,
    xInitial, lambdaInitial
):
    '''Wrapper for the KTR_init_problem method
    
    The arguments of KTR_init_problem are similar to the arguments
    of the C version. However, C pointers and arrays must be provided
    as basic Python lists (of length 1 for C pointers).
    
    Additionally, the size parameters m, nnzJ, and nnzH are not requested
    as they are automatically obtained from the size of the corresponding
    list arguments:
    - cType, cLoBnds, cUpBnds for argument m,
    - jacIndexVars, jacIndexCons for argument nnzJ,
    - hessIndexRows, hessIndexCols for argument nnzH.
    
    On the other hand, the size parameter n is required as list arguments
    xLoBnds and xUpBnds may be provided with value None.
    '''
    if n == 0:
        raise ValueError ("Knitro-Python error: Problem cannot be initialized with no variables (i.e. n = 0)!") 
    if (xLoBnds is not None and n != len (xLoBnds)) or (xUpBnds is not None and n != len (xUpBnds)):
        raise ValueError ("Arrays xLoBnds and xUpBnds have invalid non-zero sizes (must be exactly n)!")
    if xInitial is not None and n != len (xInitial):
        raise ValueError ("Array xInitial has an invalid non-zero size (must be exactly n)!")
    if (cType is not None):
        m = len (cType)
    else:
        m = 0
    if ( (cLoBnds is not None and m != len (cLoBnds)) or (cLoBnds is None and m != 0) or
         (cUpBnds is not None and m != len (cUpBnds)) or (cUpBnds is None and m != 0) ):
        raise ValueError ("Knitro-Python error: Arrays cType, cLoBnds and cUpBnds have different sizes!")
    if jacIndexVars is not None:
        nnzJ = len (jacIndexVars)
    else:
        nnzJ = 0
    if jacIndexCons is not None and nnzJ != len (jacIndexCons):
        raise ValueError ("Knitro-Python error: Arrays jacIndexVars and jacIndexCons have different sizes!")
    if hessIndexRows is not None:
        nnzH = len (hessIndexRows)
    else:
        nnzH = 0
    if hessIndexCols is not None and nnzH != len (hessIndexCols):
        raise ValueError ("Knitro-Python error: Arrays hessIndexRows and hessIndexCols have different sizes!")
    if lambdaInitial is not None and n+m != len (lambdaInitial):
        raise ValueError ("Knitro-Python error: Array lambdaInitial has invalid non-zero size (must be exactly n+m)!")
    return _knitro.KTR_init_problem (
        kc,
        n,
        objGoal,
        objType,
        _cDoubleArray (xLoBnds), _cDoubleArray (xUpBnds),
        m, _cIntArray (cType), _cDoubleArray (cLoBnds), _cDoubleArray (cUpBnds),
        nnzJ, _cIntArray (jacIndexVars), _cIntArray (jacIndexCons),
        nnzH, _cIntArray (hessIndexRows), _cIntArray (hessIndexCols),
        _cDoubleArray (xInitial), _cDoubleArray (lambdaInitial)
    )

#---- KTR_lsq_init_problem
_knitro.KTR_lsq_init_problem.argtypes = [
    KTR_context_ptr,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)
]
_knitro.KTR_lsq_init_problem.restype = ctypes.c_int
def KTR_lsq_init_problem (
    kc,
    n,
    xLoBnds, xUpBnds,
    rType,
    jacIndexVars, jacIndexCons,
    xInitial, lambdaInitial
):
    '''Wrapper for the KTR_lsq_init_problem method
    
    The arguments of KTR_lsq_init_problem are similar to the arguments
    of the C version. However, C pointers and arrays must be provided
    as basic Python lists (of length 1 for C pointers).
    
    Additionally, the size parameters m and nnzJ are not requested
    as they are automatically obtained from the size of the corresponding
    list arguments:
    - rType for argument m,
    - jacIndexVars, jacIndexCons for argument nnzJ.
    
    On the other hand, the size parameter n is required as list arguments
    xLoBnds and xUpBnds may be provided with value None.
    '''
    if n == 0:
        raise ValueError ("Knitro-Python error: Problem cannot be initialized with no variables (i.e. n = 0)!") 
    if (xLoBnds is not None and n != len (xLoBnds)) or (xUpBnds is not None and n != len (xUpBnds)):
        raise ValueError ("Arrays xLoBnds and xUpBnds have invalid non-zero sizes (must be exactly n)!")
    if xInitial is not None and n != len (xInitial):
        raise ValueError ("Array xInitial has an invalid non-zero size (must be exactly n)!")
    if (rType is not None):
        m = len (rType)
    else:
        m = 0
    if jacIndexVars is not None:
        nnzJ = len (jacIndexVars)
    else:
        nnzJ = 0
    if jacIndexCons is not None and nnzJ != len (jacIndexCons):
        raise ValueError ("Knitro-Python error: Arrays jacIndexVars and jacIndexCons have different sizes!")
    if lambdaInitial is not None and n+m != len (lambdaInitial):
        raise ValueError ("Knitro-Python error: Array lambdaInitial has invalid non-zero size (must be exactly n+m)!")
    return _knitro.KTR_lsq_init_problem (
        kc,
        n,
        _cDoubleArray (xLoBnds), _cDoubleArray (xUpBnds),
        m, _cIntArray (rType),
        nnzJ, _cIntArray (jacIndexVars), _cIntArray (jacIndexCons),
        _cDoubleArray (xInitial), _cDoubleArray (lambdaInitial)
    )

#---- KTR_mip_init_problem
_knitro.KTR_mip_init_problem.argtypes = [
    KTR_context_ptr,
    ctypes.c_int,
    ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
    ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)
]
_knitro.KTR_mip_init_problem.restype = ctypes.c_int
def KTR_mip_init_problem (
    kc,
    n,
    objGoal, objType, objFnType,
    xType, xLoBnds, xUpBnds,
    cType, cFnType, cLoBnds, cUpBnds,
    jacIndexVars, jacIndexCons,
    hessIndexRows, hessIndexCols,
    xInitial, lambdaInitial
):
    '''Wrapper for the KTR_mip_init_problem method
    
    The arguments of KTR_mip_init_problem are similar to the arguments
    of the C version. However, C pointers and arrays must be provided
    as basic Python lists (of length 1 for C pointers).
    
    Additionally, the size parameters m, nnzJ, and nnzH are not requested
    as they are automatically obtained from the size of the corresponding
    list arguments:
    - cType, cFnType, cLoBnds, cUpBnds for argument m,
    - jacIndexVars, jacIndexCons for argument nnzJ,
    - hessIndexRows, hessIndexCols for argument nnzH.

    The size parameter n is not required either as list argument xType is
    must not be None and must have length n. However, for consistency with
    the KTR_init_problem method, it is requested anyway.
    '''
    if n == 0:
        raise ValueError ("Knitro-Python error: Problem cannot be initialized with no variables (i.e. n = 0)!") 
    if n != len (xType):
        raise ValueError ("Array xType has an invalid size (must be exactly n)!")
    if (xLoBnds is not None and n != len (xLoBnds)) or (xUpBnds is not None and n != len (xUpBnds)):
        raise ValueError ("Arrays xLoBnds and xUpBnds have invalid non-zero sizes (must be exactly n)!")
    if xInitial is not None and (len (xInitial) > 0 and n != len (xInitial)):
        raise ValueError ("Array xInitial has an invalid non-zero size (must be exactly n)!")
    if (cType is not None):
        m = len (cType)
    else:
        m = 0
    if ( (cFnType is not None and m != len (cFnType)) or (cFnType is None and m != 0) or
         (cLoBnds is not None and m != len (cLoBnds)) or (cLoBnds is None and m != 0) or
         (cUpBnds is not None and m != len (cUpBnds)) or (cUpBnds is None and m != 0) ):
        raise ValueError ("Knitro-Python error: Arrays cType, cLoBnds and cUpBnds have different sizes!")
    if jacIndexVars is not None:
        nnzJ = len (jacIndexVars)
    else:
        nnzJ = 0
    if jacIndexCons is not None and nnzJ != len (jacIndexCons):
        raise ValueError ("Knitro-Python error: Arrays jacIndexVars and jacIndexCons have different sizes!")
    if hessIndexRows is not None:
        nnzH = len (hessIndexRows)
    else:
        nnzH = 0
    if hessIndexCols is not None and nnzH != len (hessIndexCols):
        raise ValueError ("Knitro-Python error: Arrays hessIndexRows and hessIndexCols have different sizes!")
    if lambdaInitial is not None and len (lambdaInitial) > 0 and  n+m != len (lambdaInitial):
        raise ValueError ("Knitro-Python error: Array lambdaInitial has invalid non-zero size (must be exactly n+m)!")
    return _knitro.KTR_mip_init_problem (
        kc,
        n,
        objGoal, objType, objFnType,
        _cIntArray (xType), _cDoubleArray (xLoBnds), _cDoubleArray (xUpBnds),
        m, _cIntArray (cType), _cIntArray (cFnType), _cDoubleArray (cLoBnds), _cDoubleArray (cUpBnds),
        nnzJ, _cIntArray (jacIndexVars), _cIntArray (jacIndexCons),
        nnzH, _cIntArray (hessIndexRows), _cIntArray (hessIndexCols),
        _cDoubleArray (xInitial), _cDoubleArray (lambdaInitial)
    )


#------------------------------------------------------------------
#     METHODS FOR PROBLEM MODIFICATION
#------------------------------------------------------------------

#---- KTR_set_compcons
_knitro.KTR_set_compcons.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_set_compcons.restype = ctypes.c_int
def KTR_set_compcons (kc, numCompConstraints, indexList1, indexList2):
    return _knitro.KTR_set_compcons (kc, numCompConstraints, _cIntArray (indexList1), _cIntArray (indexList2))

#---- KTR_addcompcons # DEPRECATED - Use KTR_set_compcons
_knitro.KTR_addcompcons.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_addcompcons.restype = ctypes.c_int
def KTR_addcompcons (kc, numCompConstraints, indexList1, indexList2):
    return _knitro.KTR_addcompcons (kc, numCompConstraints, _cIntArray (indexList1), _cIntArray (indexList2))

#---- KTR_chgvarbnds
_knitro.KTR_chgvarbnds.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_chgvarbnds.restype = ctypes.c_int
def KTR_chgvarbnds (kc, xLoBnds, xUpBnds):
    return _knitro.KTR_chgvarbnds (kc, _cDoubleArray (xLoBnds), _cDoubleArray (xUpBnds))

#------------------------------------------------------------------
#     METHODS FOR PROBLEM SOLVING
#------------------------------------------------------------------

#---- KTR_solve
_knitro.KTR_solve.argtypes = [
    KTR_context_ptr,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_void_p
]
_knitro.KTR_solve.restype = ctypes.c_int
def KTR_solve (
    kc,
    x, lambda_,
    evalStatus,
    obj, c,
    objGrad, jac,
    hess, hessVector,
    userParams
):
    '''Wrapper for the KTR_solve method

    NOTE: argument obj may be provided as an empty list []. Knitro will
    automatically append the solution objective value to it.
    '''
    if c is not None and len (c) > 0 and len (x) + len (c) != len (lambda_):
        raise ValueError ("Knitro-Python error: Arrays x, c and lambda_ have incomptatible sizes!")
    if objGrad is not None and len (objGrad) > 0 and len (x) != len (objGrad):
        raise ValueError ("Knitro-Python error: Arrays x and objGrad have incomptatible sizes!")
    if hessVector is not None and len (hessVector) > 0 and len (x) != len (hessVector):
        raise ValueError ("Knitro-Python error: Arrays x and hessVector have incomptatible sizes!")
    _registerUserParams (kc, userParams)
    c_x = _cDoubleArray (x)
    c_lambda = _cDoubleArray (lambda_)
    if obj is None:
        c_obj = ctypes.c_double (0)
    else:
        if len (obj) == 0:
            obj.append (0)
        c_obj = ctypes.c_double (obj[0])
    ret = _knitro.KTR_solve (
        kc,
        c_x, c_lambda,
        evalStatus,
        ctypes.byref (c_obj), _cDoubleArray (c),
        _cDoubleArray (objGrad), _cDoubleArray (jac),
        _cDoubleArray (hess), _cDoubleArray (hessVector),
        None
    )
    # Copy solution independently of return status
    _cToUserArray (len (x), c_x, x)
    _cToUserArray (len (lambda_), c_lambda, lambda_)
    if obj is not None:
        obj[0] = c_obj.value
    return ret

#---- KTR_restart
_knitro.KTR_restart.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_restart.restype = ctypes.c_int
def KTR_restart (kc, xInitial, lambdaInitial):
    return _knitro.KTR_restart (kc, _cDoubleArray (xInitial), _cDoubleArray (lambdaInitial))

#---- KTR_mip_set_branching_priorities
_knitro.KTR_mip_set_branching_priorities.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_int)]
_knitro.KTR_mip_set_branching_priorities.restype = ctypes.c_int
def KTR_mip_set_branching_priorities (kc, xPriorities):
    return _knitro.KTR_mip_set_branching_priorities (kc, _cIntArray (xPriorities))

#---- KTR_mip_set_intvar_strategy
_knitro.KTR_mip_set_intvar_strategy.argtypes = [KTR_context_ptr, ctypes.c_int, ctypes.c_int]
_knitro.KTR_mip_set_intvar_strategy.restype = ctypes.c_int
def KTR_mip_set_intvar_strategy (kc, xIndex, xStrategy):
    return _knitro.KTR_mip_set_intvar_strategy (kc, ctypes.c_int (xIndex), ctypes.c_int (xStrategy))

#---- KTR_mip_solve
_knitro.KTR_mip_solve.argtypes = [
    KTR_context_ptr,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
    ctypes.c_void_p
]
_knitro.KTR_mip_solve.restype = ctypes.c_int
def KTR_mip_solve (
    kc,
    x, lambda_,
    evalStatus,
    obj, c,
    objGrad, jac,
    hess, hessVector,
    userParams
):
    '''Wrapper for the KTR_mip_solve method
    
    NOTE: argument obj may be provided as an empty list []. Knitro will
    automatically append the solution objective value to it.
    '''
    if c is not None and len (c) > 0 and len (x) + len (c) != len (lambda_):
        raise ValueError ("Knitro-Python error: Arrays x, c and lambda_ have incomptatible sizes!")
    if objGrad is not None and len (objGrad) > 0 and len (x) != len (objGrad):
        raise ValueError ("Knitro-Python error: Arrays x and objGrad have incomptatible sizes!")
    if hessVector is not None and len (hessVector) > 0 and len (x) != len (hessVector):
        raise ValueError ("Knitro-Python error: Arrays x and hessVector have incomptatible sizes!")
    _registerUserParams (kc, userParams)
    c_x = _cDoubleArray (x)
    c_lambda = _cDoubleArray (lambda_)
    if obj is None:
        c_obj = ctypes.c_double (0)
    else:
        if len (obj) == 0:
            obj.append (0)
        c_obj = ctypes.c_double (obj[0])
    ret = _knitro.KTR_mip_solve (
        kc,
        c_x, c_lambda,
        evalStatus,
        ctypes.byref (c_obj), _cDoubleArray (c),
        _cDoubleArray (objGrad), _cDoubleArray (jac),
        _cDoubleArray (hess), _cDoubleArray (hessVector),
        None
    )
    if ret >= 0:
        _cToUserArray (len (x), c_x, x)
        _cToUserArray (len (lambda_), c_lambda, lambda_)
        if obj is not None:
            obj[0] = c_obj.value
    return ret

#---- KTR_set_findiff_relstepsizes
_knitro.KTR_set_findiff_relstepsizes.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_set_findiff_relstepsizes.restype = ctypes.c_int
def KTR_set_findiff_relstepsizes (kc, relStepSizes):
    return _knitro.KTR_set_findiff_relstepsizes (kc, _cDoubleArray (relStepSizes))


#------------------------------------------------------------------
#     METHODS FOR REGISTERING USER CALLBACKS
#------------------------------------------------------------------

#---- KTR_set_func_callback
_knitro.KTR_set_func_callback.argtypes = [KTR_context_ptr, _KTR_callback]
_knitro.KTR_set_func_callback.restype = ctypes.c_int
def KTR_set_func_callback (kc, fnPtr):
    '''Register function evaluation callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, \
              objGrad, jac, hessian, hessVector, userParams)
    '''
    fnPtrWrapper = _KTR_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_func_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_grad_callback
_knitro.KTR_set_grad_callback.argtypes = [KTR_context_ptr, _KTR_callback]
_knitro.KTR_set_grad_callback.restype = ctypes.c_int
def KTR_set_grad_callback (kc, fnPtr):
    '''Register gradient evaluation callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, \
              objGrad, jac, hessian, hessVector, userParams)
    '''
    fnPtrWrapper = _KTR_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_grad_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_hess_callback
_knitro.KTR_set_hess_callback.argtypes = [KTR_context_ptr, _KTR_callback]
_knitro.KTR_set_hess_callback.restype = ctypes.c_int
def KTR_set_hess_callback (kc, fnPtr):
    '''Register hessian evaluation callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, \
              objGrad, jac, hessian, hessVector, userParams)
    '''
    fnPtrWrapper = _KTR_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_hess_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_lsq_set_res_callback
_knitro.KTR_lsq_set_res_callback.argtypes = [KTR_context_ptr, _KTR_lsq_callback]
_knitro.KTR_lsq_set_res_callback.restype = ctypes.c_int
def KTR_lsq_set_res_callback (kc, fnPtr):
    '''Register residual evaluation callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, x, res, jac, userParams)
    '''
    fnPtrWrapper = _KTR_lsq_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_lsq_set_res_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_lsq_set_jac_callback
_knitro.KTR_lsq_set_jac_callback.argtypes = [KTR_context_ptr, _KTR_lsq_callback]
_knitro.KTR_lsq_set_jac_callback.restype = ctypes.c_int
def KTR_lsq_set_jac_callback (kc, fnPtr):
    '''Register residual gradient evaluation callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, x, res, jac, userParams)
    '''
    fnPtrWrapper = _KTR_lsq_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_lsq_set_jac_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_newpoint_callback
_knitro.KTR_set_newpt_callback.argtypes = [KTR_context_ptr, _KTR_newpt_callback]
_knitro.KTR_set_newpt_callback.restype = ctypes.c_int
def KTR_set_newpt_callback (kc, fnPtr):
    '''Register new estimate callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (kc, n, m, nnzJ, x, lambda_, obj, c, \
              objGrad, jac, userParams)
    '''
    fnPtrWrapper = _KTR_newpt_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_newpt_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_newpoint_callback
_knitro.KTR_set_newpoint_callback.argtypes = [KTR_context_ptr, _KTR_callback]
_knitro.KTR_set_newpoint_callback.restype = ctypes.c_int
def KTR_set_newpoint_callback (kc, fnPtr):
    '''Register new estimate callback (deprecated)
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, \
              objGrad, jac, hessian, hessVector, userParams)
    '''
    fnPtrWrapper = _KTR_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_newpoint_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_ms_process_callback
_knitro.KTR_set_ms_process_callback.argtypes = [KTR_context_ptr, _KTR_callback]
_knitro.KTR_set_ms_process_callback.restype = ctypes.c_int
def KTR_set_ms_process_callback (kc, fnPtr):
    '''Register multistart process callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, \
              objGrad, jac, hessian, hessVector, userParams)
    '''
    fnPtrWrapper = _KTR_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_ms_process_callback (kc, fnPtrWrapper.c_fnPtr)
    
#---- KTR_set_mip_node_callback
_knitro.KTR_set_mip_node_callback.argtypes = [KTR_context_ptr, _KTR_callback]
_knitro.KTR_set_mip_node_callback.restype = ctypes.c_int
def KTR_set_mip_node_callback (kc, fnPtr):
    '''Register mip node callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, \
              objGrad, jac, hessian, hessVector, userParams)
    '''
    fnPtrWrapper = _KTR_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_mip_node_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_ms_initpt_callback
_knitro.KTR_set_ms_initpt_callback.argtypes = [KTR_context_ptr, _KTR_ms_initpt_callback]
_knitro.KTR_set_ms_initpt_callback.restype = ctypes.c_int
def KTR_set_ms_initpt_callback (kc, fnPtr):
    '''Register multistart initial point callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (nSolveNumber, n, m, xLoBnds, xUpBnds, x, lambda_, userParams)
    '''
    fnPtrWrapper = _KTR_ms_initpt_callback_wrapper (kc, fnPtr)
    _registerCallback (kc, fnPtrWrapper)
    return _knitro.KTR_set_ms_initpt_callback (kc, fnPtrWrapper.c_fnPtr)

#---- KTR_set_puts_callback
_knitro.KTR_set_puts_callback.argtypes = [KTR_context_ptr, _KTR_puts]
_knitro.KTR_set_puts_callback.restype = ctypes.c_int
def KTR_set_puts_callback (kc, fnPtr):
    '''Register "put string" callback
    
    The argument fnPtr is expected to be a Python method with the following
    prototype:
    def fnPtr (str, userParams)
    '''
    if fnPtr is not None:
        fnPtrWrapper = _KTR_puts_wrapper (kc, fnPtr)
        _registerCallback (kc, fnPtrWrapper)
        return _knitro.KTR_set_puts_callback (kc, fnPtrWrapper.c_fnPtr)
    else:
        return _knitro.KTR_set_puts_callback (kc, ctypes.cast(None, _KTR_puts))


#------------------------------------------------------------------
#     METHODS TO GET SOLUTION PROPERTIES
#------------------------------------------------------------------

#---- KTR_get_number_FC_evals
_knitro.KTR_get_number_FC_evals.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_FC_evals.restype = ctypes.c_int
def KTR_get_number_FC_evals (kc):
    return _knitro.KTR_get_number_FC_evals (kc)

#---- KTR_get_number_GA_evals
_knitro.KTR_get_number_GA_evals.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_GA_evals.restype = ctypes.c_int
def KTR_get_number_GA_evals (kc):
    return _knitro.KTR_get_number_GA_evals (kc)

#---- KTR_get_number_H_evals
_knitro.KTR_get_number_H_evals.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_H_evals.restype = ctypes.c_int
def KTR_get_number_H_evals (kc):
    return _knitro.KTR_get_number_H_evals (kc)

#---- KTR_get_number_HV_evals
_knitro.KTR_get_number_HV_evals.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_HV_evals.restype = ctypes.c_int
def KTR_get_number_HV_evals (kc):
    return _knitro.KTR_get_number_HV_evals (kc)

#---- KTR_get_solution
_knitro.KTR_get_solution.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_solution.restype = ctypes.c_int
def KTR_get_solution (kc, status, obj, x, lambda_):
    c_status = ctypes.c_int (status[0])
    c_obj = ctypes.c_double (obj[0])
    c_x = _cDoubleArray (x)
    c_lambda = _cDoubleArray (lambda_)
    ret = _knitro.KTR_get_solution (kc, ctypes.byref (c_status), ctypes.byref (c_obj), c_x, c_lambda)
    if ret == 0:
        status[0] = c_status.value
        obj[0] = c_obj.value
        _cToUserArray (len (x), c_x, x)
        _cToUserArray (len (lambda_), c_lambda, lambda_)
    return ret

#---- KTR_get_constraint_values
_knitro.KTR_get_constraint_values.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_constraint_values.restype = ctypes.c_int
def KTR_get_constraint_values (kc, c):
    c_c = _cDoubleArray (c)
    ret = _knitro.KTR_get_constraint_values (kc, c_c)
    if ret == 0:
        _cToUserArray (len (c), c_c, c)
    return ret

#---- KTR_get_number_iters
_knitro.KTR_get_number_iters.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_iters.restype = ctypes.c_int
def KTR_get_number_iters (kc):
    return _knitro.KTR_get_number_iters (kc)

#---- KTR_get_number_cg_iters
_knitro.KTR_get_number_cg_iters.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_cg_iters.restype = ctypes.c_int
def KTR_get_number_cg_iters (kc):
    return _knitro.KTR_get_number_cg_iters (kc)

#---- KTR_get_number_major_iters
_knitro.KTR_get_number_major_iters.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_major_iters.restype = ctypes.c_int
def KTR_get_number_major_iters (kc):
    return _knitro.KTR_get_number_major_iters (kc)

#---- KTR_get_number_minor_iters
_knitro.KTR_get_number_minor_iters.argtypes = [KTR_context_ptr]
_knitro.KTR_get_number_minor_iters.restype = ctypes.c_int
def KTR_get_number_minor_iters (kc):
    return _knitro.KTR_get_number_minor_iters (kc)

#---- KTR_get_abs_feas_error
_knitro.KTR_get_abs_feas_error.argtypes = [KTR_context_ptr]
_knitro.KTR_get_abs_feas_error.restype = ctypes.c_double
def KTR_get_abs_feas_error (kc):
    return _knitro.KTR_get_abs_feas_error (kc)

#---- KTR_get_rel_feas_error
_knitro.KTR_get_rel_feas_error.argtypes = [KTR_context_ptr]
_knitro.KTR_get_rel_feas_error.restype = ctypes.c_double
def KTR_get_rel_feas_error (kc):
    return _knitro.KTR_get_rel_feas_error (kc)

#---- KTR_get_abs_opt_error
_knitro.KTR_get_abs_opt_error.argtypes = [KTR_context_ptr]
_knitro.KTR_get_abs_opt_error.restype = ctypes.c_double
def KTR_get_abs_opt_error (kc):
    return _knitro.KTR_get_abs_opt_error (kc)

#---- KTR_get_rel_opt_error
_knitro.KTR_get_rel_opt_error.argtypes = [KTR_context_ptr]
_knitro.KTR_get_rel_opt_error.restype = ctypes.c_double
def KTR_get_rel_opt_error (kc):
    return _knitro.KTR_get_rel_opt_error (kc)

#---- KTR_get_objgrad_values
_knitro.KTR_get_objgrad_values.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_objgrad_values.restype = ctypes.c_int
def KTR_get_objgrad_values (kc, objGrad):
    c_objGrad = _cDoubleArray (objGrad)
    ret = _knitro.KTR_get_objgrad_values (kc, c_objGrad)
    if ret == 0:
        _cToUserArray (len (objGrad), c_objGrad, objGrad)
    return ret

#---- KTR_get_jacobian_values
_knitro.KTR_get_jacobian_values.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_jacobian_values.restype = ctypes.c_int
def KTR_get_jacobian_values (kc, jac):
    c_jac = _cDoubleArray (jac)
    ret = _knitro.KTR_get_jacobian_values (kc, c_jac)
    if ret == 0:
        _cToUserArray (len (jac), c_jac, jac)
    return ret

#---- KTR_lsq_get_jacobian_values
_knitro.KTR_lsq_get_jacobian_values.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_lsq_get_jacobian_values.restype = ctypes.c_int
def KTR_lsq_get_jacobian_values (kc, jac):
    c_jac = _cDoubleArray (jac)
    ret = _knitro.KTR_lsq_get_jacobian_values (kc, c_jac)
    if ret == 0:
        _cToUserArray (len (jac), c_jac, jac)
    return ret

#---- KTR_get_hessian_values
_knitro.KTR_get_hessian_values.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_hessian_values.restype = ctypes.c_int
def KTR_get_hessian_values (kc, hess):
    c_hess = _cDoubleArray (hess)
    ret = _knitro.KTR_get_hessian_values (kc, c_hess)
    if ret == 0:
        _cToUserArray (len (hess), c_hess, hess)
    return ret

#---- KTR_get_mip_num_nodes
_knitro.KTR_get_mip_num_nodes.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_num_nodes.restype = ctypes.c_int
def KTR_get_mip_num_nodes (kc):
    return _knitro.KTR_get_mip_num_nodes (kc)

#---- KTR_get_mip_num_solves
_knitro.KTR_get_mip_num_solves.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_num_solves.restype = ctypes.c_int
def KTR_get_mip_num_solves (kc):
    return _knitro.KTR_get_mip_num_solves (kc)

#---- KTR_get_mip_abs_gap
_knitro.KTR_get_mip_abs_gap.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_abs_gap.restype = ctypes.c_double
def KTR_get_mip_abs_gap (kc):
    return _knitro.KTR_get_mip_abs_gap (kc)

#---- KTR_get_mip_rel_gap
_knitro.KTR_get_mip_rel_gap.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_rel_gap.restype = ctypes.c_double
def KTR_get_mip_rel_gap (kc):
    return _knitro.KTR_get_mip_rel_gap (kc)

#---- KTR_get_mip_incumbent_obj
_knitro.KTR_get_mip_incumbent_obj.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_incumbent_obj.restype = ctypes.c_double
def KTR_get_mip_incumbent_obj (kc):
    return _knitro.KTR_get_mip_incumbent_obj (kc)

#---- KTR_get_mip_relaxation_bnd
_knitro.KTR_get_mip_relaxation_bnd.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_relaxation_bnd.restype = ctypes.c_double
def KTR_get_mip_relaxation_bnd (kc):
    return _knitro.KTR_get_mip_relaxation_bnd (kc)

#---- KTR_get_mip_lastnode_obj
_knitro.KTR_get_mip_lastnode_obj.argtypes = [KTR_context_ptr]
_knitro.KTR_get_mip_lastnode_obj.restype = ctypes.c_double
def KTR_get_mip_lastnode_obj (kc):
    return _knitro.KTR_get_mip_lastnode_obj (kc)

#---- KTR_get_mip_incumbent_x
_knitro.KTR_get_mip_incumbent_x.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double)]
_knitro.KTR_get_mip_incumbent_x.restype = ctypes.c_int
def KTR_get_mip_incumbent_x (kc, x):
    c_x = _cDoubleArray (x)
    ret = _knitro.KTR_get_mip_incumbent_x (kc, c_x)
    if ret == 1:
        _cToUserArray (len (x), c_x, x)
    return ret


#------------------------------------------------------------------
#     METHODS FOR CHECKING DERIVATIVES
#------------------------------------------------------------------

#---- KTR_check_first_ders
_knitro.KTR_check_first_ders.argtypes = [KTR_context_ptr, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_double, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_void_p]
_knitro.KTR_check_first_ders.restype = ctypes.c_int
def KTR_check_first_ders (kc, x, finiteDiffMethod, absThreshold, relThreshold, evalStatus, obj, c, objGrad, jac, userParams):
    _registerUserParams (kc, userParams)
    c_x = _cDoubleArray (x)
    ret = _knitro.KTR_check_first_ders (kc, c_x, finiteDiffMethod, ctypes.c_double (absThreshold), ctypes.c_double (relThreshold), evalStatus, ctypes.c_double (obj), _cDoubleArray (c), _cDoubleArray (objGrad), _cDoubleArray (jac), None)
    if ret == 0:
        _cToUserArray (len (x), c_x, x)
    return ret


#------------------------------------------------------------------
#     FIELDS
#------------------------------------------------------------------

#---- DEFINE CONSTANTS TO MATCH include/_knitro.h.

#---- PROBLEM DEFINITION CONSTANTS.
KTR_INFBOUND                    = 1.0e20

KTR_PARAMTYPE_INTEGER           = 0
KTR_PARAMTYPE_FLOAT             = 1
KTR_PARAMTYPE_STRING            = 2

KTR_OBJGOAL_MINIMIZE            = 0
KTR_OBJGOAL_MAXIMIZE            = 1

KTR_OBJTYPE_CONSTANT            = -1
KTR_OBJTYPE_GENERAL             = 0
KTR_OBJTYPE_LINEAR              = 1
KTR_OBJTYPE_QUADRATIC           = 2

KTR_CONTYPE_GENERAL             = 0
KTR_CONTYPE_LINEAR              = 1
KTR_CONTYPE_QUADRATIC           = 2

KTR_RESTYPE_GENERAL             = 0
KTR_RESTYPE_LINEAR              = 1

KTR_VARTYPE_CONTINUOUS          = 0
KTR_VARTYPE_INTEGER             = 1
KTR_VARTYPE_BINARY              = 2

KTR_FNTYPE_UNCERTAIN            = 0
KTR_FNTYPE_CONVEX               = 1
KTR_FNTYPE_NONCONVEX            = 2

KTR_LINEARVAR_NO                = 0
KTR_LINEARVAR_YES               = 1

#---- KNITRO RETURN CODES.
KTR_RC_BEGINEND                 = 0
KTR_RC_EVALFC                   = 1
KTR_RC_EVALGA                   = 2
KTR_RC_EVALH                    = 3
KTR_RC_EVALX0                   = 4
KTR_RC_FINISHED                 = 5
KTR_RC_NEWPOINT                 = 6
KTR_RC_EVALHV                   = 7
KTR_RC_EVALH_NO_F               = 8
KTR_RC_EVALHV_NO_F              = 9
KTR_RC_NODE                     = 10
KTR_RC_MSPROCESS                = 11

KTR_RC_OPTIMAL_OR_SATISFACTORY  = 0
KTR_RC_OPTIMAL                  = 0
KTR_RC_NEAR_OPT                 = -100
KTR_RC_FEAS_XTOL                = -101
KTR_RC_FEAS_NO_IMPROVE          = -102    
KTR_RC_FEAS_FTOL                = -103
KTR_RC_INFEASIBLE               = -200
KTR_RC_INFEAS_XTOL              = -201    
KTR_RC_INFEAS_NO_IMPROVE        = -202
KTR_RC_INFEAS_MULTISTART        = -203
KTR_RC_INFEAS_CON_BOUNDS        = -204 
KTR_RC_INFEAS_VAR_BOUNDS        = -205 
KTR_RC_UNBOUNDED                = -300
KTR_RC_ITER_LIMIT_FEAS          = -400
KTR_RC_ITER_LIMIT               = -400
KTR_RC_TIME_LIMIT_FEAS          = -401
KTR_RC_TIME_LIMIT               = -401
KTR_RC_FEVAL_LIMIT_FEAS         = -402
KTR_RC_MIP_EXH_FEAS             = -403
KTR_RC_MIP_EXH                  = -403
KTR_RC_MIP_TERM_FEAS            = -404
KTR_RC_MIP_FEAS_TERM            = -404
KTR_RC_MIP_SOLVE_LIMIT_FEAS     = -405
KTR_RC_MIP_SOLVE_LIMIT          = -405
KTR_RC_MIP_NODE_LIMIT_FEAS      = -406
KTR_RC_MIP_NODE_LIMIT           = -406
KTR_RC_ITER_LIMIT_INFEAS        = -410
KTR_RC_TIME_LIMIT_INFEAS        = -411
KTR_RC_FEVAL_LIMIT_INFEAS       = -412
KTR_RC_MIP_EXH_INFEAS           = -413
KTR_RC_MIP_SOLVE_LIMIT_INFEAS   = -415
KTR_RC_MIP_NODE_LIMIT_INFEAS    = -416
KTR_RC_CALLBACK_ERR             = -500
KTR_RC_LP_SOLVER_ERR            = -501
KTR_RC_EVAL_ERR                 = -502
KTR_RC_OUT_OF_MEMORY            = -503
KTR_RC_USER_TERMINATION         = -504
KTR_RC_OPEN_FILE_ERR            = -505
KTR_RC_BAD_N_OR_F               = -506
KTR_RC_BAD_CONSTRAINT           = -507
KTR_RC_BAD_JACOBIAN             = -508
KTR_RC_BAD_HESSIAN              = -509
KTR_RC_BAD_CON_INDEX            = -510
KTR_RC_BAD_JAC_INDEX            = -511
KTR_RC_BAD_HESS_INDEX           = -512
KTR_RC_BAD_CON_BOUNDS           = -513
KTR_RC_BAD_VAR_BOUNDS           = -514
KTR_RC_ILLEGAL_CALL             = -515
KTR_RC_BAD_KCPTR                = -516
KTR_RC_NULL_POINTER             = -517
KTR_RC_BAD_INIT_VALUE           = -518
KTR_RC_NEWPOINT_HALT            = -519
KTR_RC_BAD_LICENSE              = -520
KTR_RC_BAD_PARAMINPUT           = -521
KTR_RC_LINEAR_SOLVER_ERR        = -522
KTR_RC_DERIV_CHECK_FAILED       = -523
KTR_RC_DERIV_CHECK_TERMINATE    = -524
KTR_RC_OVERFLOW_ERR             = -525
KTR_RC_INTERNAL_ERROR           = -600

#---- KNITRO PARAMETERS.
KTR_PARAM_NEWPOINT              = 1001
KTR_NEWPOINT_NONE                   = 0
KTR_NEWPOINT_SAVEONE                = 1
KTR_NEWPOINT_SAVEALL                = 2
KTR_NEWPOINT_USER                   = 3
KTR_PARAM_HONORBNDS             = 1002
KTR_HONORBNDS_NO                    = 0
KTR_HONORBNDS_ALWAYS                = 1
KTR_HONORBNDS_INITPT                = 2
KTR_PARAM_ALGORITHM             = 1003
KTR_PARAM_ALG                   = 1003
KTR_ALG_AUTOMATIC                   = 0
KTR_ALG_AUTO                        = 0
KTR_ALG_BAR_DIRECT                  = 1
KTR_ALG_BAR_CG                      = 2
KTR_ALG_ACT_CG                      = 3
KTR_ALG_IPDIRECT                    = 1
KTR_ALG_IPCG                        = 2
KTR_ALG_ACTIVE                      = 3
KTR_ALG_ACT_SQP                     = 4
KTR_ALG_MULTI                       = 5    
KTR_PARAM_BAR_MURULE            = 1004
KTR_PARAM_BARRULE               = 1004
KTR_BAR_MURULE_AUTOMATIC            = 0
KTR_BAR_MURULE_AUTO                 = 0
KTR_BAR_MURULE_MONOTONE             = 1
KTR_BAR_MURULE_ADAPTIVE             = 2
KTR_BAR_MURULE_PROBING              = 3
KTR_BAR_MURULE_DAMPMPC              = 4
KTR_BAR_MURULE_FULLMPC              = 5
KTR_BAR_MURULE_QUALITY              = 6
KTR_PARAM_BAR_FEASIBLE          = 1006     
KTR_PARAM_FEASIBLE              = 1006
KTR_BAR_FEASIBLE_NO                 = 0    
KTR_BAR_FEASIBLE_STAY               = 1
KTR_BAR_FEASIBLE_GET                = 2
KTR_BAR_FEASIBLE_GET_STAY           = 3
KTR_FEASIBLE_NO                     = 0
KTR_FEASIBLE_ALWAYS                 = 1
KTR_PARAM_GRADOPT               = 1007
KTR_GRADOPT_EXACT                   = 1
KTR_GRADOPT_FORWARD                 = 2
KTR_GRADOPT_CENTRAL                 = 3
KTR_GRADOPT_USER_FORWARD            = 4
KTR_GRADOPT_USER_CENTRAL            = 5
KTR_PARAM_HESSOPT               = 1008
KTR_HESSOPT_EXACT                   = 1
KTR_HESSOPT_BFGS                    = 2
KTR_HESSOPT_SR1                     = 3
KTR_HESSOPT_FINITE_DIFF             = 4
KTR_HESSOPT_PRODUCT_FINDIFF         = 4
KTR_HESSOPT_PRODUCT                 = 5
KTR_HESSOPT_LBFGS                   = 6
KTR_HESSOPT_GAUSS_NEWTON            = 7 
KTR_PARAM_BAR_INITPT            = 1009
KTR_BAR_INITPT_AUTO                 = 0
KTR_BAR_INITPT_YES                  = 1
KTR_BAR_INITPT_STRAT1               = 1
KTR_BAR_INITPT_NO                   = 2
KTR_BAR_INITPT_STRAT2               = 2
KTR_BAR_INITPT_STRAT3               = 3
KTR_PARAM_ACT_LPSOLVER          = 1012
KTR_ACT_LPSOLVER_INTERNAL           = 1
KTR_ACT_LPSOLVER_CPLEX              = 2
KTR_ACT_LPSOLVER_XPRESS             = 3
KTR_PARAM_LPSOLVER              = 1012
KTR_LP_INTERNAL                     = 1
KTR_LP_CPLEX                        = 2
KTR_LP_XPRESS                       = 3
KTR_PARAM_CG_MAXIT              = 1013
KTR_PARAM_MAXCGIT               = 1013 # DEPRECATED - Use KTR_PARAM_CG_MAXIT
KTR_PARAM_MAXIT                 = 1014
KTR_PARAM_OUTLEV                = 1015
KTR_OUTLEV_NONE                     = 0
KTR_OUTLEV_SUMMARY                  = 1
KTR_OUTLEV_MAJORIT10                = 2
KTR_OUTLEV_ITER_10                  = 2
KTR_OUTLEV_MAJORIT                  = 3
KTR_OUTLEV_ITER                     = 3
KTR_OUTLEV_ALLIT                    = 4
KTR_OUTLEV_ITER_VERBOSE             = 4
KTR_OUTLEV_ALLIT_X                  = 5
KTR_OUTLEV_ITER_X                   = 5
KTR_OUTLEV_ALL                      = 6
KTR_PARAM_OUTMODE               = 1016
KTR_OUTMODE_SCREEN                  = 0
KTR_OUTMODE_FILE                    = 1
KTR_OUTMODE_BOTH                    = 2
KTR_PARAM_SCALE                 = 1017
KTR_SCALE_NEVER                     = 0
KTR_SCALE_NO                        = 0
KTR_SCALE_ALLOW                     = 1
KTR_SCALE_USER_INTERNAL             = 1
KTR_SCALE_USER_NONE                 = 2    
KTR_SCALE_INTERNAL                  = 3    
KTR_PARAM_SHIFTINIT             = 1018
KTR_PARAM_SOC                   = 1019
KTR_SOC_NO                          = 0
KTR_SOC_MAYBE                       = 1    
KTR_SOC_YES                         = 2
KTR_PARAM_DELTA                 = 1020
KTR_PARAM_BAR_FEASMODETOL       = 1021
KTR_PARAM_FEASMODETOL           = 1021
KTR_PARAM_FEASTOL               = 1022
KTR_PARAM_FEASTOLABS            = 1023
KTR_PARAM_MAXTIMECPU            = 1024
KTR_PARAM_BAR_INITMU            = 1025
KTR_PARAM_MU                    = 1025
KTR_PARAM_OBJRANGE              = 1026
KTR_PARAM_OPTTOL                = 1027
KTR_PARAM_OPTTOLABS             = 1028
KTR_PARAM_LINSOLVER_PIVOTTOL    = 1029
KTR_PARAM_PIVOT                 = 1029 # DEPRECATED -- Use LINSOLVER_PIVOTTOL
KTR_PARAM_XTOL                  = 1030
KTR_PARAM_DEBUG                 = 1031
KTR_DEBUG_NONE                      = 0
KTR_DEBUG_PROBLEM                   = 1
KTR_DEBUG_EXECUTION                 = 2
KTR_PARAM_MULTISTART            = 1033
KTR_PARAM_MSENABLE              = 1033
KTR_MULTISTART_NO                   = 0
KTR_MULTISTART_YES                  = 1
KTR_PARAM_MSMAXSOLVES           = 1034
KTR_PARAM_MSMAXBNDRANGE         = 1035
KTR_PARAM_MSMAXTIMECPU          = 1036
KTR_PARAM_MSMAXTIMEREAL         = 1037
KTR_PARAM_LMSIZE                = 1038
KTR_PARAM_BAR_MAXCROSSIT        = 1039
KTR_PARAM_MAXCROSSIT            = 1039
KTR_PARAM_MAXTIMEREAL           = 1040
KTR_PARAM_CG_PRECOND            = 1041
KTR_CG_PRECOND_NONE                 = 0
KTR_CG_PRECOND_CHOL                 = 1
KTR_PARAM_BLASOPTION            = 1042
KTR_BLASOPTION_KNITRO               = 0
KTR_BLASOPTION_INTEL                = 1
KTR_BLASOPTION_DYNAMIC              = 2
KTR_PARAM_BAR_MAXREFACTOR       = 1043
KTR_PARAM_BAR_MAXBACKTRACK      = 1044 # DEPRECATED - Use LINESEARCH_MAXTRIALS
KTR_PARAM_LINESEARCH_MAXTRIALS  = 1044 
KTR_PARAM_BLASOPTIONLIB         = 1045
KTR_PARAM_OUTAPPEND             = 1046
KTR_OUTAPPEND_NO                    = 0
KTR_OUTAPPEND_YES                   = 1
KTR_PARAM_OUTDIR                = 1047    
KTR_PARAM_CPLEXLIB              = 1048
KTR_PARAM_BAR_PENRULE           = 1049
KTR_BAR_PENRULE_AUTO                = 0
KTR_BAR_PENRULE_SINGLE              = 1
KTR_BAR_PENRULE_FLEX                = 2
KTR_PARAM_BAR_PENCONS           = 1050
KTR_BAR_PENCONS_AUTO                = 0
KTR_BAR_PENCONS_NONE                = 1
KTR_BAR_PENCONS_ALL                 = 2
KTR_BAR_PENCONS_EQUALITIES          = 3
KTR_BAR_PENCONS_INFEAS              = 4
KTR_PARAM_MSNUMTOSAVE           = 1051
KTR_PARAM_MSSAVETOL             = 1052
KTR_PARAM_PRESOLVEDEBUG         = 1053
KTR_PRESOLVEDBG_NONE                = 0
KTR_PRESOLVEDBG_BASIC               = 1
KTR_PRESOLVEDBG_VERBOSE             = 2
KTR_PARAM_MSTERMINATE           = 1054
KTR_MSTERMINATE_MAXSOLVES           = 0
KTR_MSTERMINATE_OPTIMAL             = 1
KTR_MSTERMINATE_FEASIBLE            = 2
KTR_MSTERMINATE_ANY                 = 3
KTR_PARAM_MSSTARTPTRANGE        = 1055
KTR_PARAM_INFEASTOL             = 1056
KTR_PARAM_LINSOLVER             = 1057
KTR_LINSOLVER_AUTO                  = 0
KTR_LINSOLVER_INTERNAL              = 1
KTR_LINSOLVER_HYBRID                = 2        
KTR_LINSOLVER_DENSEQR               = 3
KTR_LINSOLVER_MA27                  = 4
KTR_LINSOLVER_MA57                  = 5
KTR_LINSOLVER_MKLPARDISO            = 6
KTR_PARAM_BAR_DIRECTINTERVAL    = 1058
KTR_PARAM_PRESOLVE              = 1059
KTR_PRESOLVE_NONE                   = 0
KTR_PRESOLVE_BASIC                  = 1
KTR_PRESOLVE_ADVANCED               = 2    
KTR_PARAM_PRESOLVE_TOL          = 1060
KTR_PARAM_BAR_SWITCHRULE        = 1061
KTR_BAR_SWITCHRULE_AUTO             = 0
KTR_BAR_SWITCHRULE_NEVER            = 1
KTR_BAR_SWITCHRULE_LEVEL1           = 2
KTR_BAR_SWITCHRULE_LEVEL2           = 3
KTR_PARAM_HESSIAN_NO_F          = 1062
KTR_HESSIAN_NO_F_FORBID             = 0
KTR_HESSIAN_NO_F_ALLOW              = 1
KTR_PARAM_MA_TERMINATE          = 1063
KTR_MA_TERMINATE_ALL                = 0
KTR_MA_TERMINATE_OPTIMAL            = 1
KTR_MA_TERMINATE_FEASIBLE           = 2
KTR_MA_TERMINATE_ANY                = 3
KTR_PARAM_MA_MAXTIMECPU         = 1064
KTR_PARAM_MA_MAXTIMEREAL        = 1065    
KTR_PARAM_MSSEED                = 1066
KTR_PARAM_MA_OUTSUB             = 1067
KTR_MA_OUTSUB_NONE                  = 0
KTR_MA_OUTSUB_YES                   = 1    
KTR_PARAM_MS_OUTSUB             = 1068
KTR_MS_OUTSUB_NONE                  = 0
KTR_MS_OUTSUB_YES                   = 1    
KTR_PARAM_XPRESSLIB             = 1069
KTR_PARAM_TUNER                 = 1070
KTR_TUNER_OFF                       = 0
KTR_TUNER_ON                        = 1
KTR_PARAM_TUNER_OPTIONSFILE     = 1071
KTR_PARAM_TUNER_MAXTIMECPU      = 1072
KTR_PARAM_TUNER_MAXTIMEREAL     = 1073    
KTR_PARAM_TUNER_OUTSUB          = 1074
KTR_TUNER_OUTSUB_NONE               = 0
KTR_TUNER_OUTSUB_YES                = 1
KTR_TUNER_OUTSUB_SUMMARY            = 1
KTR_TUNER_OUTSUB_ALL                = 2
KTR_PARAM_TUNER_TERMINATE       = 1075
KTR_TUNER_TERMINATE_ALL             = 0
KTR_TUNER_TERMINATE_OPTIMAL         = 1
KTR_TUNER_TERMINATE_FEASIBLE        = 2
KTR_TUNER_TERMINATE_ANY             = 3
KTR_PARAM_LINSOLVER_OOC         = 1076
KTR_LINSOLVER_OOC_NO                = 0
KTR_LINSOLVER_OOC_MAYBE             = 1    
KTR_LINSOLVER_OOC_YES               = 2    
KTR_PARAM_BAR_RELAXCONS         = 1077    
KTR_BAR_RELAXCONS_NONE              = 0
KTR_BAR_RELAXCONS_EQS               = 1
KTR_BAR_RELAXCONS_INEQS             = 2
KTR_BAR_RELAXCONS_ALL               = 3    
KTR_PARAM_MSDETERMINISTIC       = 1078
KTR_MSDETERMINISTIC_NO              = 0
KTR_MSDETERMINISTIC_YES             = 1
KTR_PARAM_BAR_REFINEMENT        = 1079
KTR_BAR_REFINEMENT_NO               = 0
KTR_BAR_REFINEMENT_YES              = 1
KTR_PARAM_DERIVCHECK            = 1080
KTR_DERIVCHECK_NONE                 = 0
KTR_DERIVCHECK_FIRST                = 1
KTR_DERIVCHECK_SECOND               = 2
KTR_DERIVCHECK_ALL                  = 3
KTR_PARAM_DERIVCHECK_TYPE       = 1081
KTR_DERIVCHECK_FORWARD              = 1
KTR_DERIVCHECK_CENTRAL              = 2
KTR_PARAM_DERIVCHECK_TOL        = 1082
KTR_PARAM_LINSOLVER_INEXACT     = 1083
KTR_LINSOLVER_INEXACT_NO            = 0
KTR_LINSOLVER_INEXACT_YES           = 1
KTR_PARAM_LINSOLVER_INEXACTTOL  = 1084
KTR_PARAM_MAXFEVALS             = 1085
KTR_PARAM_FSTOPVAL              = 1086
KTR_PARAM_DATACHECK             = 1087
KTR_DATACHECK_NO                    = 0
KTR_DATACHECK_YES                   = 1
KTR_PARAM_DERIVCHECK_TERMINATE  = 1088
KTR_DERIVCHECK_STOPERROR            = 1
KTR_DERIVCHECK_STOPALWAYS           = 2
KTR_PARAM_BAR_WATCHDOG          = 1089
KTR_BAR_WATCHDOG_NO                 = 0
KTR_BAR_WATCHDOG_YES                = 1
KTR_PARAM_FTOL                  = 1090
KTR_PARAM_FTOL_ITERS            = 1091
KTR_PARAM_ACT_QPALG             = 1092
KTR_ACT_QPALG_AUTO                  = 0
KTR_ACT_QPALG_BAR_DIRECT            = 1
KTR_ACT_QPALG_BAR_CG                = 2
KTR_ACT_QPALG_ACT_CG                = 3
KTR_PARAM_BAR_INITPI_MPEC       = 1093
KTR_PARAM_XTOL_ITERS            = 1094
KTR_PARAM_LINESEARCH            = 1095
KTR_LINESEARCH_AUTO                 = 0
KTR_LINESEARCH_BACKTRACK            = 1
KTR_LINESEARCH_INTERPOLATE          = 2
KTR_PARAM_OUT_CSVINFO           = 1096
KTR_OUT_CSVINFO_NO                  = 0
KTR_OUT_CSVINFO_YES                 = 1
KTR_PARAM_INITPENALTY           = 1097
KTR_PARAM_ACT_LPFEASTOL         = 1098
KTR_PARAM_CG_STOPTOL            = 1099    
KTR_PARAM_RESTARTS              = 1100
KTR_PARAM_RESTARTS_MAXIT        = 1101
KTR_PARAM_BAR_SLACKBOUNDPUSH    = 1102    
KTR_PARAM_CG_PMEM               = 1103
KTR_PARAM_BAR_SWITCHOBJ         = 1104
KTR_BAR_SWITCHOBJ_NONE              = 0
KTR_BAR_SWITCHOBJ_SCALARPROX        = 1
KTR_BAR_SWITCHOBJ_DIAGPROX          = 2

KTR_PARAM_MIP_METHOD            = 2001
KTR_MIP_METHOD_AUTO                 = 0
KTR_MIP_METHOD_BB                   = 1
KTR_MIP_METHOD_HQG                  = 2
KTR_MIP_METHOD_MISQP                = 3
KTR_PARAM_MIP_BRANCHRULE        = 2002
KTR_MIP_BRANCH_AUTO                 = 0
KTR_MIP_BRANCH_MOSTFRAC             = 1
KTR_MIP_BRANCH_PSEUDOCOST           = 2
KTR_MIP_BRANCH_STRONG               = 3    
KTR_PARAM_MIP_SELECTRULE        = 2003
KTR_MIP_SEL_AUTO                    = 0
KTR_MIP_SEL_DEPTHFIRST              = 1
KTR_MIP_SEL_BESTBOUND               = 2
KTR_MIP_SEL_COMBO_1                 = 3
KTR_PARAM_MIP_INTGAPABS         = 2004
KTR_PARAM_MIP_INTGAPREL         = 2005
KTR_PARAM_MIP_MAXTIMECPU        = 2006
KTR_PARAM_MIP_MAXTIMEREAL       = 2007
KTR_PARAM_MIP_MAXSOLVES         = 2008
KTR_PARAM_MIP_INTEGERTOL        = 2009
KTR_PARAM_MIP_OUTLEVEL          = 2010
KTR_MIP_OUTLEVEL_NONE               = 0
KTR_MIP_OUTLEVEL_ITERS              = 1
KTR_MIP_OUTLEVEL_ITERSTIME          = 2
KTR_MIP_OUTLEVEL_ROOT               = 3   
KTR_PARAM_MIP_OUTINTERVAL       = 2011
KTR_PARAM_MIP_OUTSUB            = 2012
KTR_MIP_OUTSUB_NONE                 = 0
KTR_MIP_OUTSUB_YES                  = 1
KTR_MIP_OUTSUB_YESPROB              = 2
KTR_PARAM_MIP_DEBUG             = 2013
KTR_MIP_DEBUG_NONE                  = 0
KTR_MIP_DEBUG_ALL                   = 1
KTR_PARAM_MIP_IMPLICATNS        = 2014
KTR_MIP_IMPLICATNS_NO               = 0
KTR_MIP_IMPLICATNS_YES              = 1
KTR_PARAM_MIP_GUB_BRANCH        = 2015
KTR_MIP_GUB_BRANCH_NO               = 0
KTR_MIP_GUB_BRANCH_YES              = 1
KTR_PARAM_MIP_KNAPSACK          = 2016
KTR_MIP_KNAPSACK_NO                 = 0
KTR_MIP_KNAPSACK_INEQ               = 1
KTR_MIP_KNAPSACK_INEQ_EQ            = 2
KTR_PARAM_MIP_ROUNDING          = 2017
KTR_MIP_ROUND_AUTO                  = 0
KTR_MIP_ROUND_NONE                  = 1
KTR_MIP_ROUND_HEURISTIC             = 2
KTR_MIP_ROUND_NLP_SOME              = 3
KTR_MIP_ROUND_NLP_ALWAYS            = 4
KTR_PARAM_MIP_ROOTALG           = 2018
KTR_MIP_ROOTALG_AUTO                = 0
KTR_MIP_ROOTALG_BAR_DIRECT          = 1
KTR_MIP_ROOTALG_BAR_CG              = 2
KTR_MIP_ROOTALG_ACT_CG              = 3
KTR_MIP_ROOTALG_ACT_SQP             = 4
KTR_MIP_ROOTALG_MULTI               = 5
KTR_PARAM_MIP_LPALG             = 2019
KTR_MIP_LPALG_AUTO                  = 0
KTR_MIP_LPALG_BAR_DIRECT            = 1
KTR_MIP_LPALG_BAR_CG                = 2
KTR_MIP_LPALG_ACT_CG                = 3
KTR_PARAM_MIP_TERMINATE         = 2020
KTR_MIP_TERMINATE_OPTIMAL           = 0
KTR_MIP_TERMINATE_FEASIBLE          = 1
KTR_PARAM_MIP_MAXNODES          = 2021
KTR_PARAM_MIP_HEURISTIC         = 2022
KTR_MIP_HEURISTIC_AUTO              = 0
KTR_MIP_HEURISTIC_NONE              = 1
KTR_MIP_HEURISTIC_FEASPUMP          = 2
KTR_MIP_HEURISTIC_MPEC              = 3    
KTR_PARAM_MIP_HEUR_MAXIT        = 2023
KTR_PARAM_MIP_HEUR_MAXTIMECPU   = 2024
KTR_PARAM_MIP_HEUR_MAXTIMEREAL  = 2025
KTR_PARAM_MIP_PSEUDOINIT        = 2026
KTR_MIP_PSEUDOINIT_AUTO             = 0
KTR_MIP_PSEUDOINIT_AVE              = 1
KTR_MIP_PSEUDOINIT_STRONG           = 2
KTR_PARAM_MIP_STRONG_MAXIT      = 2027
KTR_PARAM_MIP_STRONG_CANDLIM    = 2028    
KTR_PARAM_MIP_STRONG_LEVEL      = 2029
KTR_PARAM_MIP_INTVAR_STRATEGY   = 2030
KTR_MIP_INTVAR_STRATEGY_NONE        = 0
KTR_MIP_INTVAR_STRATEGY_RELAX       = 1
KTR_MIP_INTVAR_STRATEGY_MPEC        = 2    
KTR_PARAM_MIP_RELAXABLE         = 2031
KTR_MIP_RELAXABLE_NONE              = 0
KTR_MIP_RELAXABLE_ALL               = 1
KTR_PARAM_MIP_NODEALG           = 2032
KTR_MIP_NODEALG_AUTO                = 0
KTR_MIP_NODEALG_BAR_DIRECT          = 1
KTR_MIP_NODEALG_BAR_CG              = 2
KTR_MIP_NODEALG_ACT_CG              = 3
KTR_MIP_NODEALG_ACT_SQP             = 4
KTR_MIP_NODEALG_MULTI               = 5
KTR_PARAM_MIP_HEUR_TERMINATE    = 2033
KTR_MIP_HEUR_TERMINATE_FEASIBLE     = 1
KTR_MIP_HEUR_TERMINATE_LIMIT        = 2

KTR_PARAM_PAR_NUMTHREADS        = 3001
KTR_PARAM_PAR_CONCURRENT_EVALS  = 3002
KTR_PAR_CONCURRENT_EVALS_NO         = 0
KTR_PAR_CONCURRENT_EVALS_YES        = 1
KTR_PARAM_PAR_BLASNUMTHREADS    = 3003
KTR_PARAM_PAR_LSNUMTHREADS      = 3004
KTR_PARAM_PAR_MSNUMTHREADS      = 3005
KTR_PAR_MSNUMTHREADS_AUTO           = 0