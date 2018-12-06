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
#    python exampleLSQ
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


from knitro import *
from math import *


 ## Solve simple nonlinear least-squares problem.
 #
 #  min   ( x1*1.309^x2 - 2.138 )^2 + ( x1*1.471^x2 - 3.421 )^2 + ( x1*1.49^x2 - 3.597 )^2
 #         + ( x1*1.565^x2 - 4.34 )^2 + ( x1*1.611^x2 - 4.882 )^2 + ( x1*1.68^x2-5.66 )^2
 #
 #  The standard start point (1.0, 5.0) usually converges to the standard
 #  minimum at (0.76886, 3.86041), with final objective = 0.00216.
 ##

#----------------------------------------------------------------
#   METHOD evaluateR
#----------------------------------------------------------------
 ## Compute the function and constraint values at x.
 #
 #  For more information about the arguments, refer to the Knitro
 #  manual, especially the section on the Callable Library.
 ##
def evaluateR (x, r):
	r[0] = x[0] * pow(1.309, x[1]) - 2.138
	r[1] = x[0] * pow(1.471, x[1]) - 3.421
	r[2] = x[0] * pow(1.49, x[1]) - 3.597
	r[3] = x[0] * pow(1.565, x[1]) - 4.34
	r[4] = x[0] * pow(1.611, x[1]) - 4.882
	r[5] = x[0] * pow(1.68, x[1]) - 5.66

#----------------------------------------------------------------
#   METHOD evaluateJ
#----------------------------------------------------------------
 ## Compute the residual first derivatives at x.
 #
 #  For more information about the arguments, refer to the Knitro
 #  manual, especially the section on the Callable Library.
 ##
def evaluateJ (x, jac):
    jac[0] = pow(1.309, x[1])
    jac[1] = x[0] * log(1.309) * pow(1.309, x[1])
    jac[2] = pow(1.471, x[1])
    jac[3] = x[0] * log(1.471) * pow(1.471, x[1])
    jac[4] = pow(1.49, x[1])
    jac[5] = x[0] * log(1.49) * pow(1.49, x[1])
    jac[6] = pow(1.565, x[1])
    jac[7] = x[0] * log(1.565) * pow(1.565, x[1])
    jac[8] = pow(1.611, x[1])
    jac[9] = x[0] * log(1.611) * pow(1.611, x[1])
    jac[10] = pow(1.68, x[1])
    jac[11] = x[0] * log(1.68) * pow(1.68, x[1])


#----------------------------------------------------------------
#   MAIN METHOD FOR TESTING
#----------------------------------------------------------------

#---- DEFINE THE OPTIMIZATION TEST PROBLEM.
#---- FOR MORE INFORMATION ABOUT THE PROBLEM DEFINITION, REFER
#---- TO THE KNITRO MANUAL, ESPECIALLY THE SECTION ON THE
#---- CALLABLE LIBRARY.
n = 2
xLoBnds = [ -KTR_INFBOUND, -KTR_INFBOUND ]
xUpBnds = [ KTR_INFBOUND, KTR_INFBOUND ]
m = 6
rType = [ KTR_RESTYPE_GENERAL ] * 6
jacIxRes = [ 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5 ]
jacIxVar = [ 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 ]

xInit = [ 1.0, 5.0 ]


#---- SETUP AND RUN KNITRO TO SOLVE THE PROBLEM.

#---- CREATE A NEW KNITRO SOLVER INSTANCE.
kc = KTR_new()
if kc == None:
    raise RuntimeError ("Failed to find a Knitro license.")

#---- DEMONSTRATE HOW TO SET KNITRO PARAMETERS.
if KTR_set_int_param(kc, KTR_PARAM_DERIVCHECK, KTR_DERIVCHECK_ALL):
    raise RuntimeError ("Error setting parameter 'derivcheck'")

#------------------------------------------------------------------ 
#     FUNCTION callbackEvalR
#------------------------------------------------------------------
 ## The signature of this function matches KTR_lsq_callback in knitro.h.
 #  Only "res" is modified.
 ##
def callbackEvalR (n, m, nnzJ, x, res, jac, userParams):
    evaluateR(x, res)
    return 0

#------------------------------------------------------------------
#     FUNCTION callbackEvalJ
#------------------------------------------------------------------
 ## The signature of this function matches KTR_lsq_callback in knitro.h.
 #  Only "jac" is modified.
 ##
def callbackEvalJ (n, m, nnzJ, x, res, jac, userParams):
    evaluateJ(x, jac)
    return 0

#---- REGISTER THE CALLBACK FUNCTIONS THAT PERFORM PROBLEM EVALUATION.
if KTR_lsq_set_res_callback(kc, callbackEvalR):
    raise RuntimeError ("Error registering residual callback.")
if KTR_lsq_set_jac_callback(kc, callbackEvalJ):
    raise RuntimeError ("Error registering residual jacobian callback.")

#---- INITIALIZE KNITRO WITH THE PROBLEM DEFINITION.
ret = KTR_lsq_init_problem (kc, n, xLoBnds, xUpBnds,
                                rType,
                                jacIxVar, jacIxRes,
                                xInit, None)
if ret:
	raise RuntimeError ("Error initializing the problem, "
                                + "Knitro status = "
                                + str(ret))

#---- SOLVE THE PROBLEM.
#----
#---- RETURN STATUS CODES ARE DEFINED IN "knitro.h" AND DESCRIBED
#---- IN THE KNITRO MANUAL.
x       = [0] * n
lambda_ = [0] * (m + n)
obj     = [0]
nStatus = KTR_solve (kc, x, lambda_, 0, obj,
                         None, None, None, None, None, None)

print
print
if nStatus != 0:
    raise RuntimeError ("Knitro failed to solve the problem, final status = %d" % nStatus)
else:
    #---- AN EXAMPLE OF OBTAINING SOLUTION INFORMATION.
    print ("Knitro successful, feasibility violation    = %e" % KTR_get_abs_feas_error (kc))
    print ("                   KKT optimality violation = %e" % KTR_get_abs_opt_error (kc))

#---- BE CERTAIN THE NATIVE OBJECT INSTANCE IS DESTROYED.
KTR_free (kc)

#+++++++++++++++++++ End of source file +++++++++++++++++++++++++++++
