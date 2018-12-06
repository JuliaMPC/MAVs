/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This file contains routines to implement problemDef.h for
 *  test problem HS15 from the Hock & Schittkowski collection.
 *
 *  min   100 (x2 - x1^2)^2 + (1 - x1)^2
 *  s.t.  x1 x2 >= 1
 *        x1 + x2^2 >= 0
 *        x1 <= 0.5
 *
 *  The standard start point (-2, 1) usually converges to the standard
 *  minimum at (0.5, 2.0), with final objective = 306.5.
 *  Sometimes the solver converges to another local minimum
 *  at (-0.79212, -1.26243), with final objective = 360.4.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


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
    *n = 2;
    *m = 2;
    *nnzJ = 4;
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
    xLoBnds[0] = -KTR_INFBOUND;
    xLoBnds[1] = -KTR_INFBOUND;
    xUpBnds[0] = 0.5;
    xUpBnds[1] = KTR_INFBOUND;
    xInitial[0] = -2.0;
    xInitial[1] =  1.0;

    /*---- DEFINE THE CONSTRAINT FUNCTIONS. */
    cType[0] = KTR_CONTYPE_QUADRATIC;
    cType[1] = KTR_CONTYPE_QUADRATIC;
    cLoBnds[0] = 1.0;
    cLoBnds[1] = 0.0;
    cUpBnds[0] = KTR_INFBOUND;
    cUpBnds[1] = KTR_INFBOUND;

    /*---- PROVIDE FIRST DERIVATIVE STRUCTURAL INFORMATION. */
    jacIndexCons[0] = 0;
    jacIndexCons[1] = 0;
    jacIndexCons[2] = 1;
    jacIndexCons[3] = 1;
    jacIndexVars[0] = 0;
    jacIndexVars[1] = 1;
    jacIndexVars[2] = 0;
    jacIndexVars[3] = 1;

    /*---- PROVIDE SECOND DERIVATIVE STRUCTURAL INFORMATION. */
    hessRows[0] = 0;
    hessRows[1] = 0;
    hessRows[2] = 1;
    hessCols[0] = 0;
    hessCols[1] = 1;
    hessCols[2] = 1;

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
    double  dTmp;

    dTmp = x[1] - x[0]*x[0];
    dObj = 100.0 * (dTmp*dTmp) + ((1.0 - x[0])*(1.0 - x[0]));
    c[0] = x[0] * x[1];
    c[1] = x[0] + (x[1]*x[1]);

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
    double  dTmp;

    dTmp = x[1] - x[0]*x[0];
    objGrad[0] = (-400.0 * dTmp * x[0]) - (2.0 * (1.0 - x[0]));
    objGrad[1] = 200.0 * dTmp;

    jac[0] = x[1];
    jac[1] = x[0];
    jac[2] = 1.0;
    jac[3] = 2.0 * x[1];

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
    hess[0] = objScaler * ( (-400.0 * x[1]) + (1200.0 * x[0]*x[0]) + 2.0);
    hess[1] = (objScaler * (-400.0 * x[0])) + lambda[0];
    hess[2] = (objScaler * 200.0) + (lambda[1] * 2.0);

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
    double  daTmpVec[2];

    /*---- H[0,0]*v[0] + H[0,1]*v[1]. */
    daTmpVec[0] =   (objScaler*(((-400.0 * x[1]) + (1200.0 * x[0]*x[0]) + 2.0))) * vector[0]
                  + (objScaler*(-400.0 * x[0]) + lambda[0])                      * vector[1];

    /*---- H[1,0]*v[0] + H[1,1]*v[1]. */
    daTmpVec[1] =   (objScaler*(-400.0 * x[0]) + lambda[0]) * vector[0]
                  + (objScaler*200.0 + (lambda[1] * 2.0))   * vector[1];

    vector[0] = daTmpVec[0];
    vector[1] = daTmpVec[1];
    return;
}


/*----- End of source code -----------------------------------------*/
