/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This file contains routines to implement problemDef.h for
 *  test problem 1 (Synthesis of processing system) in
 *  M. Duran & I.E. Grossmann,  "An outer approximation algorithm for
 *  a class of mixed integer nonlinear programs", Mathematical
 *  Programming 36, pp. 307-339, 1986.  The problem also appears as
 *  problem synthes1 in the MacMINLP test set.
 *  
 *
 *  min   5 x4 + 6 x5 + 8 x6 + 10 x1 - 7 x3 -18 log(x2 + 1)
 *       - 19.2 log(x1 - x2 + 1) + 10
 *  s.t.  0.8 log(x2 + 1) + 0.96 log(x1 - x2 + 1) - 0.8 x3 >= 0
 *        log(x2 + 1) + 1.2 log(x1 - x2 + 1) - x3 - 2 x6 >= -2
 *        x2 - x1 <= 0
 *        x2 - 2 x4 <= 0
 *        x1 - x2 - 2 x5 <= 0
 *        x4 + x5 <= 1
 *        0 <= x1 <= 2 
 *        0 <= x2 <= 2
 *        0 <= x3 <= 1
 *        x1, x2, x3 continuous
 *        x4, x5, x6 binary
 *        
 *
 *  The solution is (1.30098, 0, 1, 0, 1, 0).
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <math.h>
#include "problemDef.h"
#include "knitro.h"         /*-- NEED THIS FOR KTR_ SYMBOLS */


/*------------------------------------------------------------------*/
/*     FUNCTION getProblemSizes                                     */
/*------------------------------------------------------------------*/
/** Define sizes for problem HS15.
 */
void  getProblemSizes (int *  const  n,
                       int *  const  m,
                       int *  const  nnzJ,
                       int *  const  nnzH)
{
    *n = 6;
    *m = 6;
    *nnzJ = 16;
    *nnzH = 3;

    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION loadProblemArrays                                   */
/*------------------------------------------------------------------*/
/** Load Knitro arrays for problem HS15.
 *  The function assumes array memory has been allocated to match
 *  the problem sizes.
 *
 *  For more information about the arguments, refer to the Knitro
 *  manual, especially the section on the Callable Library.
 */
void  getProblemData (int    * const  objType,
                      int    * const  objGoal,
                      double * const  xLoBnds,
                      double * const  xUpBnds,
                      double * const  xInitial,
                      int    * const  cType,
                      double * const  cLoBnds,
                      double * const  cUpBnds,
                      int    * const  jacIndexVars,
                      int    * const  jacIndexCons,
                      int    * const  hessRows,
                      int    * const  hessCols,
                      int    * const  objFnType,
                      int    * const  xType,    
                      int    * const  cFnType)
{
    /*---- DEFINE THE OBJECTIVE FUNCTION AND VARIABLE BOUNDS. */
    *objType = KTR_OBJTYPE_GENERAL;
    *objGoal = KTR_OBJGOAL_MINIMIZE;
    /*---- CONTINUOUS VARIABLES. */
    xLoBnds[0] = 0.0;
    xLoBnds[1] = 0.0;
    xLoBnds[2] = 0.0;
    xUpBnds[0] = 2.0;
    xUpBnds[1] = 2.0;
    xUpBnds[2] = 1.0;
    /*---- BINARY VARIABLES: SPECIFY 0 FOR LoBnd AND 1 FOR UpBnd. */
    xLoBnds[3] = 0.0;
    xLoBnds[4] = 0.0;
    xLoBnds[5] = 0.0;
    xUpBnds[3] = 1.0;
    xUpBnds[4] = 1.0;
    xUpBnds[5] = 1.0;
    /*---- SPECIFY INITIAL POINT HERE IF DESIRED.  FOR THIS EXAMPLE,
     *---- KNITRO WILL COMPUTE THE INITIAL POINT. */

    
    /*---- DEFINE THE CONSTRAINT FUNCTIONS. */
    cType[0] = KTR_CONTYPE_GENERAL;
    cType[1] = KTR_CONTYPE_GENERAL;
    cType[2] = KTR_CONTYPE_LINEAR;
    cType[3] = KTR_CONTYPE_LINEAR;
    cType[4] = KTR_CONTYPE_LINEAR;
    cType[5] = KTR_CONTYPE_LINEAR;    
    cLoBnds[0] = 0.0;
    cLoBnds[1] = -2.0;
    cLoBnds[2] = -KTR_INFBOUND;
    cLoBnds[3] = -KTR_INFBOUND;
    cLoBnds[4] = -KTR_INFBOUND;
    cLoBnds[5] = -KTR_INFBOUND;
    cUpBnds[0] = KTR_INFBOUND;
    cUpBnds[1] = KTR_INFBOUND;
    cUpBnds[2] = 0.0;
    cUpBnds[3] = 0.0;
    cUpBnds[4] = 0.0;
    cUpBnds[5] = 1.0;
    
    /*---- PROVIDE FIRST DERIVATIVE STRUCTURAL INFORMATION. */
    jacIndexCons[0]  = 0;
    jacIndexCons[1]  = 0;    
    jacIndexCons[2]  = 0;
    jacIndexCons[3]  = 1;
    jacIndexCons[4]  = 1;
    jacIndexCons[5]  = 1;
    jacIndexCons[6]  = 1;
    jacIndexCons[7]  = 2;
    jacIndexCons[8]  = 2;
    jacIndexCons[9]  = 3;
    jacIndexCons[10] = 3;
    jacIndexCons[11] = 4;
    jacIndexCons[12] = 4;
    jacIndexCons[13] = 4;
    jacIndexCons[14] = 5;    
    jacIndexCons[15] = 5;
    
    jacIndexVars[0]  = 0;
    jacIndexVars[1]  = 1;
    jacIndexVars[2]  = 2;
    jacIndexVars[3]  = 0;
    jacIndexVars[4]  = 1;
    jacIndexVars[5]  = 2;
    jacIndexVars[6]  = 5;
    jacIndexVars[7]  = 0;
    jacIndexVars[8]  = 1;
    jacIndexVars[9]  = 1;
    jacIndexVars[10] = 3;
    jacIndexVars[11] = 0;    
    jacIndexVars[12] = 1;
    jacIndexVars[13] = 4;
    jacIndexVars[14] = 3;
    jacIndexVars[15] = 4;    
    
    /*---- PROVIDE SECOND DERIVATIVE STRUCTURAL INFORMATION. */
    hessRows[0] = 0;
    hessRows[1] = 0;
    hessRows[2] = 1;
    hessCols[0] = 0;
    hessCols[1] = 1;
    hessCols[2] = 1;

    /*---- SPECIFY VARIABLE TYPE (e.g. CONTINUOUS, BINARY, INTEGER). */
    xType[0] = KTR_VARTYPE_CONTINUOUS;
    xType[1] = KTR_VARTYPE_CONTINUOUS;
    xType[2] = KTR_VARTYPE_CONTINUOUS;
    xType[3] = KTR_VARTYPE_BINARY;
    xType[4] = KTR_VARTYPE_BINARY;
    xType[5] = KTR_VARTYPE_BINARY;
        
    /*---- SPECIFY FUNCTIONS AS CONVEX (IF NOT KNOWN, THESE CAN BE
     *---- SET AS KTR_FNTYPE_UNCERTAIN. */
    *objFnType = KTR_FNTYPE_CONVEX;
    cFnType[0] = KTR_FNTYPE_CONVEX;
    cFnType[1] = KTR_FNTYPE_CONVEX;
    cFnType[2] = KTR_FNTYPE_CONVEX;
    cFnType[3] = KTR_FNTYPE_CONVEX;
    cFnType[4] = KTR_FNTYPE_CONVEX;
    cFnType[5] = KTR_FNTYPE_CONVEX;
    
    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION computeFC                                           */
/*------------------------------------------------------------------*/
/** Compute the function and constraint values at x.
 *
 *  For more information about the arguments, refer to the Knitro
 *  manual, especially the section on the Callable Library.
 */
double  computeFC (const double * const  x,
                   double * const  c)
{
    double  dObj;
    double  dTmp1;
    double  dTmp2;

    dTmp1 = x[0] - x[1] + 1.0;
    dTmp2 = x[1] + 1.0;
    dObj = 5.0*x[3] + 6.0*x[4] + 8.0*x[5] + 10.0*x[0] - 7.0*x[2]
        - 18.0*log(dTmp2) - 19.2*log(dTmp1) + 10.0;
    c[0] = 0.8*log(dTmp2) + 0.96*log(dTmp1) - 0.8*x[2];
    c[1] = log(dTmp2) + 1.2*log(dTmp1) - x[2] - 2*x[5];
    c[2] = x[1] - x[0];
    c[3] = x[1] - 2*x[3];
    c[4] = x[0] - x[1] - 2*x[4];
    c[5] = x[3] + x[4];

    return( dObj );
}


/*------------------------------------------------------------------*/
/*     FUNCTION computeGA                                           */
/*------------------------------------------------------------------*/
/** Compute the function and constraint first derivatives at x.
 *
 *  For more information about the arguments, refer to the Knitro
 *  manual, especially the section on the Callable Library.
 */
void  computeGA (const double * const  x,
                       double * const  objGrad,
                       double * const  jac)
{
    double  dTmp1;
    double  dTmp2;
    
    dTmp1 = x[0] - x[1] + 1.0;
    dTmp2 = x[1] + 1.0;
    objGrad[0] = 10.0 - (19.2 / dTmp1);
    objGrad[1] = (-18.0 / dTmp2) + (19.2 / dTmp1);
    objGrad[2] = -7.0;
    objGrad[3] = 5.0;
    objGrad[4] = 6.0;
    objGrad[5] = 8.0;

    /*---- GRADIENT OF CONSTRAINT 0. */
    jac[0] = 0.96 / dTmp1;
    jac[1] = (-0.96 / dTmp1) + (0.8 / dTmp2) ;
    jac[2] = -0.8;
    /*---- GRADIENT OF CONSTRAINT 1. */
    jac[3] = 1.2 / dTmp1;
    jac[4] = (-1.2 / dTmp1) + (1.0 / dTmp2) ;
    jac[5] = -1.0;
    jac[6] = -2.0;
    /*---- GRADIENT OF CONSTRAINT 2. */
    jac[7] = -1.0;
    jac[8] = 1.0;
    /*---- GRADIENT OF CONSTRAINT 3. */
    jac[9] = 1.0;
    jac[10] = -2.0;    
    /*---- GRADIENT OF CONSTRAINT 4. */
    jac[11] = 1.0;
    jac[12] = -1.0;    
    jac[13] = -2.0;
    /*---- GRADIENT OF CONSTRAINT 5. */
    jac[14] = 1.0;
    jac[15] = 1.0;
    
    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION computeH                                            */
/*------------------------------------------------------------------*/
/** Compute the Hessian of the Lagrangian at x and lambda.
 *
 *  For more information about the arguments, refer to the Knitro
 *  manual, especially the section on the Callable Library.
 */
void  computeH (const double * const  x,
                const double          objScaler,
                const double * const  lambda,
                      double * const  hess)
{
    double  dTmp1;
    double  dTmp2;
    
    dTmp1 = x[0] - x[1] + 1.0;
    dTmp2 = x[1] + 1.0;
    hess[0] = objScaler*(19.2 / (dTmp1*dTmp1))
        + lambda[0]*(-0.96 / (dTmp1*dTmp1))
        + lambda[1]*(-1.2 / (dTmp1*dTmp1));
    hess[1] = objScaler*(-19.2 / (dTmp1*dTmp1))
        + lambda[0]*(0.96 / (dTmp1*dTmp1))
        + lambda[1]*(1.2 / (dTmp1*dTmp1));
    hess[2] = objScaler*((19.2 / (dTmp1*dTmp1)) + (18.0 / (dTmp2*dTmp2)))
        + lambda[0]*((-0.96 / (dTmp1*dTmp1)) - (0.8 / (dTmp2*dTmp2)))
        + lambda[1]*((-1.2 / (dTmp1*dTmp1)) - (1.0 / (dTmp2*dTmp2)));

    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION computeHV                                           */
/*------------------------------------------------------------------*/
/** Compute the Hessian of the Lagrangian times vector at x and lambda.
 *  Return the result in vector.
 *
 *  For more information about the arguments, refer to the Knitro
 *  manual, especially the section on the Callable Library.
 */
void  computeHV (const double * const  x,
                 const double          objScaler,
                 const double * const  lambda,
                       double * const  vector)
{
    double  dTmp1;
    double  dTmp2;
    double  daTmpVec[2];
    
    dTmp1 = x[0] - x[1] + 1.0;
    dTmp2 = x[1] + 1.0;

    /*---- H[0,0]*v[0] + H[0,1]*v[1]. */
    daTmpVec[0] = (  objScaler*(19.2 / (dTmp1*dTmp1))
                   + lambda[0]*(-0.96 / (dTmp1*dTmp1))        
                   + lambda[1]*(-1.2 / (dTmp1*dTmp1)) ) * vector[0]        
                + (  objScaler*(-19.2 / (dTmp1*dTmp1))
                   + lambda[0]*(0.96 / (dTmp1*dTmp1))
                   + lambda[1]*(1.2 / (dTmp1*dTmp1)) ) * vector[1];

    /*---- H[1,0]*v[0] + H[1,1]*v[1]. */
    daTmpVec[1] = (  objScaler*(-19.2 / (dTmp1*dTmp1))
                   + lambda[0]*(0.96 / (dTmp1*dTmp1))
                   + lambda[1]*(1.2 / (dTmp1*dTmp1)) ) * vector[0]
                + (  objScaler*((19.2 / (dTmp1*dTmp1)) + (18.0 / (dTmp2*dTmp2)))
                   + lambda[0]*((-0.96 / (dTmp1*dTmp1)) - (0.8 / (dTmp2*dTmp2)))
                   + lambda[1]*((-1.2 / (dTmp1*dTmp1)) - (1.0 / (dTmp2*dTmp2))) ) * vector[1];
                  
    vector[0] = daTmpVec[0];
    vector[1] = daTmpVec[1];
    vector[2] = 0.0;
    vector[3] = 0.0;
    vector[4] = 0.0;
    vector[5] = 0.0;
    
    return;
}


/*----- End of source code -----------------------------------------*/
