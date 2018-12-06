/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This file contains routines to implement problemDef.h for
 *  a simple nonlinear least-squares problem.
 *
 *  min   ( x1*1.309^x2 - 2.138 )^2 + ( x1*1.471^x2 - 3.421 )^2 + ( x1*1.49^x2 - 3.597 )^2
 *         + ( x1*1.565^x2 - 4.34 )^2 + ( x1*1.611^x2 - 4.882 )^2 + ( x1*1.68^x2-5.66 )^2  
 *
 *  The standard start point (1.0, 5.0) usually converges to the standard
 *  minimum at (0.76886, 3.86041), with final objective = 0.00216.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <math.h>
#include "problemDef.h"
#include "knitro.h"

void getLsqProblemSizes (int * const n,
                         int * const m, 
                         int * const nnzJ) {
    *n = 2;
    *m = 6;
    *nnzJ = 12;
    return;
}

void getLsqProblemData ( double * const xLoBnds,
                         double * const xUpBnds,
                         double * const xInitial,
                         int    * const jacIndexVars,
                         int    * const jacIndexCons) {

    xLoBnds[0] = -1.0 * KTR_INFBOUND;
    xUpBnds[0] = KTR_INFBOUND;
    xLoBnds[1] = -1.0* KTR_INFBOUND;
    xUpBnds[1] = KTR_INFBOUND;
    xInitial[0] = 1.0;
    xInitial[1] = 5.0;

    /*---- PROVIDE FIRST DERIVATIVE STRUCTURAL INFORMATION. */
    jacIndexCons[0] = 0; jacIndexCons[1] = 0;
    jacIndexCons[2] = 1; jacIndexCons[3] = 1;
    jacIndexCons[4] = 2; jacIndexCons[5] = 2;
    jacIndexCons[6] = 3; jacIndexCons[7] = 3;
    jacIndexCons[8] = 4; jacIndexCons[9] = 4;
    jacIndexCons[10] = 5; jacIndexCons[11] = 5;

    jacIndexVars[0] = 0; jacIndexVars[1] = 1;
    jacIndexVars[2] = 0; jacIndexVars[3] = 1;
    jacIndexVars[4] = 0; jacIndexVars[5] = 1;
    jacIndexVars[6] = 0; jacIndexVars[7] = 1;
    jacIndexVars[8] = 0; jacIndexVars[9] = 1;
    jacIndexVars[10] = 0; jacIndexVars[11] = 1;
    return;
}


void computeR (const double * const x,
                     double * const r) {
    r[0] = x[0] * pow(1.309, x[1]) - 2.138;
    r[1] = x[0] * pow(1.471, x[1]) - 3.421; 
    r[2] = x[0] * pow(1.49, x[1]) - 3.597;
    r[3] = x[0] * pow(1.565, x[1]) - 4.34;
    r[4] = x[0] * pow(1.611, x[1]) - 4.882;
    r[5] = x[0] * pow(1.68, x[1]) - 5.66;
    return;
}


void computeJ (const double * const x, 
                      double * const jac) {
    jac[0] = pow(1.309, x[1]);
    jac[1] = x[0] * log(1.309) * pow(1.309, x[1]);
    jac[2] = pow(1.471, x[1]);
    jac[3] = x[0] * log(1.471) * pow(1.471, x[1]);
    jac[4] = pow(1.49, x[1]);
    jac[5] = x[0] * log(1.49) * pow(1.49, x[1]);
    jac[6] = pow(1.565, x[1]);
    jac[7] = x[0] * log(1.565) * pow(1.565, x[1]);
    jac[8] = pow(1.611, x[1]);
    jac[9] = x[0] * log(1.611) * pow(1.611, x[1]);
    jac[10] = pow(1.68, x[1]);
    jac[11] = x[0] * log(1.68) * pow(1.68, x[1]);
    return;
}

