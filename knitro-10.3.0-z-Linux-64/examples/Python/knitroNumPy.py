#*******************************************************
#* Copyright (c) 2017 by Artelys                       *
#* All Rights Reserved                                 *
#*******************************************************

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++  Artelys Knitro 10.3.0 Python API
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


'''NumPy support in Python interface for Artelys Knitro.

This module activates the support of NumPy arrays in the
Python interface for Artelys Knitro.

NumPy are supported and returned instead of Python list.

This source is provided for informational purposes.
'''

import knitro
import ctypes
import numpy as np


#------------------------------------------------------------------
#     VERSION NUMBER
#------------------------------------------------------------------
__version__ = knitro.__version__


#------------------------------------------------------------------
#     METHODS AND CLASSES TO WORK WITH C ARRAYS AND ctypes.POINTERS
#     SPECIALIZATION FOR NUMPY ARRAYS
#------------------------------------------------------------------

@staticmethod
def _cIntArray (npArray):
    '''Construct a 'ctypes' array of ints from a NumPy array.
    '''
    if npArray is not None:
        n = len (npArray)
        if n > 0:
            if type(npArray) == np.ndarray and npArray.dtype == np.int32:
                return npArray.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
            else: # in case of a non NumPy array or non 32-bit integer element type, simply create a new array
                return (ctypes.c_int * n) (*npArray)
    return None

@staticmethod
def _cDoubleArray (npArray):
    '''Construct a 'ctypes' array of doubles from a NumPy array.
    '''
    if npArray is not None:
        n = len (npArray)
        if n > 0:
            if type(npArray) == np.ndarray and npArray.dtype == np.float:
                return npArray.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
            else:  # in case of a non NumPy array or non double element type, simply create a new array
                return (ctypes.c_double * n) (*npArray)
    return None

@staticmethod
def _userArray (size, cArray):
    '''Construct a NumPy array from a 'ctypes' array.
    '''
    if cArray:
        return np.ctypeslib.as_array (cArray, tuple ([size]))
    else:
        return None

@staticmethod
def _userToCArray (npArray, cArray):
    '''Copy the content of a NumPy array to a 'ctypes' array.
    '''
    if npArray is not None:
        if npArray.ctypes.data != ctypes.addressof(cArray.contents):
            for i in xrange (len (npArray)):
                cArray[i] = npArray[i]

@staticmethod
def _cToUserArray (size, cArray, npArray):
    '''Copy the content a 'ctypes' array to a NumPy array.
    '''
    if cArray:
        if type(npArray) == np.ndarray:
            npArray = np.ctypeslib.as_array (cArray, tuple ([size]))
        else:
            npArray[:] = cArray

knitro.KTR_array_handler._cIntArray = _cIntArray
knitro.KTR_array_handler._cDoubleArray = _cDoubleArray
knitro.KTR_array_handler._userArray = _userArray
knitro.KTR_array_handler._userToCArray = _userToCArray
knitro.KTR_array_handler._cToUserArray = _cToUserArray
