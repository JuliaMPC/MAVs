#*******************************************************
#* Copyright (c) 2016 by Artelys                       *
#* All Rights Reserved                                 *
#*******************************************************

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#  Knitro example driver using callback mode, defining separate
#  callback functions for each evaluation request.
#
#  This executable invokes Knitro to solve a simple nonlinear
#  optimization test problem.  The purpose is to illustrate how to
#  invoke Knitro using the Python language API.
#
#  Before running, make sure ../../lib is in the load path.
#  To run:
#    python exampleMINLP
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


from knitro import *
import math

 ## Solve test problem 1 (Synthesis of processing system) in
 #  M. Duran & I.E. Grossmann, "An outer approximation algorithm for
 #  a class of mixed integer nonlinear programs", Mathematical
 #  Programming 36, pp. 307-339, 1986.  The problem also appears as
 #  problem synthes1 in the MacMINLP test set.
 #
 #  min   5 x4 + 6 x5 + 8 x6 + 10 x1 - 7 x3 -18 math.log(x2 + 1)
 #       - 19.2 math.log(x1 - x2 + 1) + 10
 #  s.t.  0.8 math.log(x2 + 1) + 0.96 math.log(x1 - x2 + 1) - 0.8 x3 >= 0
 #        math.log(x2 + 1) + 1.2 math.log(x1 - x2 + 1) - x3 - 2 x6 >= -2
 #        x2 - x1 <= 0
 #        x2 - 2 x4 <= 0
 #        x1 - x2 - 2 x5 <= 0
 #        x4 + x5 <= 1
 #        0 <= x1 <= 2 
 #        0 <= x2 <= 2
 #        0 <= x3 <= 1
 #        x1, x2, x3 continuous
 #        x4, x5, x6 binary
 #        
 #
 #  The solution is (1.30098, 0, 1, 0, 1, 0).
 ##

#----------------------------------------------------------------
#   METHOD evaluateFC
#----------------------------------------------------------------
 ## Compute the function and constraint values at x.
 #
 #  For more information about the arguments, refer to the Knitro
 #  manual, especially the section on the Callable Library.
 ##
def evaluateFC (x, c):
    tmp1 = x[0] - x[1] + 1.0
    tmp2 = x[1] + 1.0
    obj  = 5.0*x[3] + 6.0*x[4] + 8.0*x[5] + 10.0*x[0] - 7.0*x[2] \
        - 18.0*math.log(tmp2) - 19.2*math.log(tmp1) + 10.0;
    c[0] = 0.8*math.log(tmp2) + 0.96*math.log(tmp1) - 0.8*x[2]
    c[1] = math.log(tmp2) + 1.2*math.log(tmp1) - x[2] - 2*x[5]
    c[2] = x[1] - x[0]
    c[3] = x[1] - 2*x[3]
    c[4] = x[0] - x[1] - 2*x[4]
    c[5] = x[3] + x[4]
    return obj


#----------------------------------------------------------------
#   METHOD evaluateGA
#----------------------------------------------------------------
 ## Compute the function and constraint first derivatives at x.
 #
 #  For more information about the arguments, refer to the Knitro
 #  manual, especially the section on the Callable Library.
 ##
def evaluateGA (x, objGrad, jac):
    tmp1 = x[0] - x[1] + 1.0
    tmp2 = x[1] + 1.0
    objGrad[0] = 10.0 - (19.2 / tmp1)
    objGrad[1] = (-18.0 / tmp2) + (19.2 / tmp1)
    objGrad[2] = -7.0
    objGrad[3] = 5.0
    objGrad[4] = 6.0
    objGrad[5] = 8.0

    #---- GRADIENT OF CONSTRAINT 0.
    jac[0] = 0.96 / tmp1
    jac[1] = (-0.96 / tmp1) + (0.8 / tmp2) 
    jac[2] = -0.8
    #---- GRADIENT OF CONSTRAINT 1.
    jac[3] = 1.2 / tmp1
    jac[4] = (-1.2 / tmp1) + (1.0 / tmp2) 
    jac[5] = -1.0
    jac[6] = -2.0
    #---- GRADIENT OF CONSTRAINT 2.
    jac[7] = -1.0
    jac[8] = 1.0
    #---- GRADIENT OF CONSTRAINT 3.
    jac[9] = 1.0
    jac[10] = -2.0    
    #---- GRADIENT OF CONSTRAINT 4.
    jac[11] = 1.0
    jac[12] = -1.0    
    jac[13] = -2.0
    #---- GRADIENT OF CONSTRAINT 5.
    jac[14] = 1.0
    jac[15] = 1.0


#----------------------------------------------------------------
#   METHOD evaluateH
#----------------------------------------------------------------
 ## Compute the Hessian of the Lagrangian at x and lambda.
 #
 #  For more information about the arguments, refer to the Knitro
 #  manual, especially the section on the Callable Library.
 ##
def evaluateH (x, lambda_, sigma, hess):
    tmp1 = x[0] - x[1] + 1.0
    tmp2 = x[1] + 1.0
    hess[0] = sigma*(19.2 / (tmp1*tmp1))   \
        + lambda_[0]*(-0.96 / (tmp1*tmp1)) \
        + lambda_[1]*(-1.2 / (tmp1*tmp1))
    hess[1] = sigma*(-19.2 / (tmp1*tmp1))  \
        + lambda_[0]*(0.96 / (tmp1*tmp1))  \
        + lambda_[1]*(1.2 / (tmp1*tmp1))
    hess[2] = sigma*((19.2 / (tmp1*tmp1)) + (18.0 / (tmp2*tmp2)))  \
        + lambda_[0]*((-0.96 / (tmp1*tmp1)) - (0.8 / (tmp2*tmp2))) \
        + lambda_[1]*((-1.2 / (tmp1*tmp1)) - (1.0 / (tmp2*tmp2)))


#----------------------------------------------------------------
#   MAIN METHOD FOR TESTING
#----------------------------------------------------------------

#---- DEFINE THE OPTIMIZATION TEST PROBLEM.
#---- FOR MORE INFORMATION ABOUT THE PROBLEM DEFINITION, REFER
#---- TO THE KNITRO MANUAL, ESPECIALLY THE SECTION ON THE
#---- CALLABLE LIBRARY.
n = 6
objGoal   = KTR_OBJGOAL_MINIMIZE
objType   = KTR_OBJTYPE_GENERAL
objFnType = KTR_FNTYPE_CONVEX
xLoBnds = [ 0.0 ] * n
xUpBnds = [ 2.0 ] * 2 + [ 1.0 ] * 4
xType  = [ KTR_VARTYPE_CONTINUOUS ] * 3 + [ KTR_VARTYPE_BINARY ] * 3
m = 6
cType = [ KTR_CONTYPE_GENERAL ] * 2 + [ KTR_CONTYPE_LINEAR ] * 4
cLoBnds = [ 0.0, -2.0 ] + [ -KTR_INFBOUND ] * 4
cUpBnds = [ KTR_INFBOUND ] * 2 + [ 0.0 ] * 3 + [ 1.0 ]
cFnType = [ KTR_FNTYPE_CONVEX ] * 6
jacIndexCons = [ 0 ] * 3 + [ 1 ] * 4 + [ 2 ] * 2 + [ 3 ] * 2 +[ 4 ] * 3 + [ 5 ] * 2
jacIndexVars    = [ 0, 1, 2, 0, 1, 2, 5, 0, 1, 1, 3, 0, 1, 4, 3, 4 ]
hessRows = [ 0, 0, 1 ]
hessCols = [ 0, 1, 1 ]


#---- SETUP AND RUN KNITRO TO SOLVE THE PROBLEM.

#---- CREATE A NEW KNITRO SOLVER INSTANCE.
kc = KTR_new()
if kc == None:
    raise RuntimeError ("Failed to find a Knitro license.")

#---- DEMONSTRATE HOW TO SET KNITRO PARAMETERS.
if KTR_set_int_param_by_name(kc, "mip_method", KTR_MIP_METHOD_BB):
    raise RuntimeError ("Error setting parameter 'mip_method'")
if KTR_set_int_param_by_name(kc, "algorithm", KTR_ALG_ACT_CG):
    raise RuntimeError ("Error setting parameter 'algorithm'")
if KTR_set_int_param_by_name(kc, "outmode", KTR_OUTMODE_SCREEN):
    raise RuntimeError ("Error setting parameter 'outmode'")
if KTR_set_int_param(kc, KTR_PARAM_OUTLEV, KTR_OUTLEV_ALL):
    raise RuntimeError ("Error setting parameter KTR_PARAM_OUTLEV")
if KTR_set_int_param(kc, KTR_PARAM_MIP_OUTINTERVAL, 1):
    raise RuntimeError ("Error setting parameter KTR_PARAM_MIP_OUTINTERVAL")
if KTR_set_int_param(kc, KTR_PARAM_MIP_MAXNODES, 10000):
    raise RuntimeError ("Error setting parameter KTR_PARAM_MIP_MAXNODES")

#---- SPECIFY THAT THE USER IS ABLE TO PROVIDE EVALUATIONS
#---- OF THE HESSIAN MATRIX WITHOUT THE OBJECTIVE COMPONENT.
#---- TURNED OFF BY DEFAULT BUT SHOULD BE ENABLED IF POSSIBLE.
if KTR_set_int_param (kc, KTR_PARAM_HESSIAN_NO_F, KTR_HESSIAN_NO_F_ALLOW):
    raise RuntimeError ("Error setting parameter KTR_PARAM_HESSIAN_NO_F")

#------------------------------------------------------------------ 
#     FUNCTION callbackEvalFC
#------------------------------------------------------------------
 ## The signature of this function matches KTR_callback in knitro.h.
 #  Only "obj" and "c" are modified.
 ##
def callbackEvalFC (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, objGrad, jac, hessian, hessVector, userParams):
    if evalRequestCode == KTR_RC_EVALFC:
        obj[0] = evaluateFC(x, c)
        return 0
    else:
        return KTR_RC_CALLBACK_ERR

#------------------------------------------------------------------
#     FUNCTION callbackEvalGA
#------------------------------------------------------------------
 ## The signature of this function matches KTR_callback in knitro.h.
 #  Only "objGrad" and "jac" are modified.
 ##
def callbackEvalGA (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, objGrad, jac, hessian, hessVector, userParams):
    if evalRequestCode == KTR_RC_EVALGA:
        evaluateGA(x, objGrad, jac)
        return 0
    else:
        return KTR_RC_CALLBACK_ERR

#------------------------------------------------------------------
#     FUNCTION callbackEvalH
#------------------------------------------------------------------
 ## The signature of this function matches KTR_callback in knitro.h.
 #  Only "hessian" or "hessVector" is modified.
 ##
def callbackEvalH (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, objGrad, jac, hessian, hessVector, userParams):
    if evalRequestCode == KTR_RC_EVALH:
        evaluateH(x, lambda_, 1.0, hessian)
        return 0
    elif evalRequestCode == KTR_RC_EVALH_NO_F:
        evaluateH(x, lambda_, 0.0, hessian)
        return 0
    else:
        return KTR_RC_CALLBACK_ERR

#---- REGISTER THE CALLBACK FUNCTIONS THAT PERFORM PROBLEM EVALUATION.
#---- THE HESSIAN CALLBACK ONLY NEEDS TO BE REGISTERED FOR SPECIFIC
#---- HESSIAN OPTIONS (E.G., IT IS NOT REGISTERED IF THE OPTION FOR
#---- BFGS HESSIAN APPROXIMATIONS IS SELECTED).
if KTR_set_func_callback(kc, callbackEvalFC):
    raise RuntimeError ("Error registering function callback.")
if KTR_set_grad_callback(kc, callbackEvalGA):
    raise RuntimeError ("Error registering gradient callback.")
if KTR_set_hess_callback(kc, callbackEvalH):
    raise RuntimeError ("Error registering hessian callback.")

#------------------------------------------------------------------ 
#     FUNCTION callbackProcessNode
#------------------------------------------------------------------
 ## The signature of this function matches KTR_callback in knitro.h.
 ##
def callbackProcessNode (evalRequestCode, n, m, nnzJ, nnzH, x, lambda_, obj, c, objGrad, jac, hessian, hessVector, userParams):
    #---- THE KNITRO CONTEXT POINTER WAS PASSED IN THROUGH "userParams".
    kc = userParams
    
    #---- PRINT INFO ABOUT THE STATUS OF THE MIP SOLUTION.
    print ("callbackProcessNode: ")
    print ("    Node number    = %d" % KTR_get_mip_num_nodes (kc))
    print ("    Node objective = %e" % obj[0])
    print ("    Current relaxation bound = %e" % KTR_get_mip_relaxation_bnd (kc))
    incumbentBound = KTR_get_mip_incumbent_obj (kc)
    if abs(incumbentBound) >= KTR_INFBOUND:
        print ("    No integer feasible point found yet.")
    else:
        print ("    Current incumbent bound  = %e" % incumbentBound)
        print ("    Absolute integrality gap = %e" % KTR_get_mip_abs_gap (kc))
        print ("    Relative integrality gap = %e" % KTR_get_mip_rel_gap (kc))

    #---- USER DEFINED TERMINATION EXAMPLE.
    #---- UNCOMMENT BELOW TO FORCE TERMINATION AFTER 3 NODES.
    #if KTR_get_mip_num_nodes (kc) == 3:
    #    return KTR_RC_USER_TERMINATION
    
    return 0

#---- REGISTER THE CALLBACK FUNCTION THAT PERFORMS SOME TASK AFTER
#---- EACH COMPLETION OF EACH NODE IN BRANCH-AND-BOUND TREE.
if KTR_set_mip_node_callback (kc, callbackProcessNode):
    raise RuntimeError ("Error registering node process callback.")

#---- INITIALIZE KNITRO WITH THE PROBLEM DEFINITION.
ret = KTR_mip_init_problem (kc, n, objGoal, objType, objFnType,
                                    xType, xLoBnds, xUpBnds,
                                    cType, cFnType, cLoBnds, cUpBnds,
                                    jacIndexVars, jacIndexCons,
                                    hessRows, hessCols, None, None)
if ret:
    raise RuntimeError ("Error initializing the problem, Knitro status = %d" % ret)

#---- SOLVE THE PROBLEM.
#----
#---- RETURN STATUS CODES ARE DEFINED IN "knitro.h" AND DESCRIBED
#---- IN THE KNITRO MANUAL.
x       = [0] * n
lambda_ = [0] * (m + n)
obj     = [0]
nStatus = KTR_mip_solve (kc, x, lambda_, 0, obj,
                         None, None, None, None, None, kc)

print
print
if nStatus != 0:
    raise RuntimeError ("Knitro failed to solve the problem, final status = %d" % nStatus)
else:
    #---- AN EXAMPLE OF OBTAINING SOLUTION INFORMATION.
    print ("Knitro successful, integrality gap   = %e" % KTR_get_mip_abs_gap (kc))

#---- BE CERTAIN THE NATIVE OBJECT INSTANCE IS DESTROYED.
KTR_free (kc)

#+++++++++++++++++++ End of source file +++++++++++++++++++++++++++++
