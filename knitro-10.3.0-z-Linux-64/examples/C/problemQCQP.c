/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This file contains routines to implement problemDef.h for a small
 *  QCQP (quadratically constrained quadratic programming) test problem.
 *
 *  min   1000 - x0^2 - 2 x1^2 - x2^2 - x0 x1 - x0 x2
 *  s.t.  8 x0 + 14 x1 + 7 x2 - 56 = 0
 *        x0^2 + x1^2 + x2^2 - 25 >= 0
 *        x0 >= 0, x1 >= 0, x2 >= 0
 *
 *  The start point (2, 2, 2) converges to the minimum at (0, 0, 8),
 *  with final objective = 936.0.  From a different start point,
 *  Knitro may converge to an alternate local solution at (7, 0, 0),
 *  with objective = 951.0.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include "problemDef.h"
#include "knitro.h"


/*------------------------------------------------------------------*/
/*     FUNCTION getProblemSizes                                     */
/*------------------------------------------------------------------*/
/** Define sizes for the problem.
 */
void  getProblemSizes (int *  const  n,
                       int *  const  m,
                       int *  const  nnzJ,
                       int *  const  nnzH)
{
    *n = 3;
    *m = 2;
    *nnzJ = 6;
    *nnzH = 5;

    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION loadProblemArrays                                   */
/*------------------------------------------------------------------*/
/** Load Knitro arrays for the problem.
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
    *objType = KTR_OBJTYPE_QUADRATIC;
    *objGoal = KTR_OBJGOAL_MINIMIZE;
    xLoBnds[0] = 0.0;
    xUpBnds[0] = KTR_INFBOUND;
    xLoBnds[1] = 0.0;
    xUpBnds[1] = KTR_INFBOUND;
    xLoBnds[2] = 0.0;
    xUpBnds[2] = KTR_INFBOUND;
    xInitial[0] = 2.0;
    xInitial[1] = 2.0;
    xInitial[2] = 2.0;

    /*---- DEFINE THE CONSTRAINT FUNCTIONS. */
    cType[0] = KTR_CONTYPE_LINEAR;
    cType[1] = KTR_CONTYPE_QUADRATIC;
    cLoBnds[0] = 0.0;
    cUpBnds[0] = 0.0;
    cLoBnds[1] = 0.0;
    cUpBnds[1] = KTR_INFBOUND;

    /*---- PROVIDE FIRST DERIVATIVE STRUCTURAL INFORMATION. */
    jacIndexCons[0] = 0;
    jacIndexVars[0] = 0;
    jacIndexCons[1] = 0;
    jacIndexVars[1] = 1;
    jacIndexCons[2] = 0;
    jacIndexVars[2] = 2;
    jacIndexCons[3] = 1;
    jacIndexVars[3] = 0;
    jacIndexCons[4] = 1;
    jacIndexVars[4] = 1;
    jacIndexCons[5] = 1;
    jacIndexVars[5] = 2;

    /*---- PROVIDE SECOND DERIVATIVE STRUCTURAL INFORMATION;
     *---- ONLY THE NONZEROES OF THE UPPER TRIANGLE OF THE HESSIAN. */
    hessRows[0] = 0;
    hessCols[0] = 0;
    hessRows[1] = 0;
    hessCols[1] = 1;
    hessRows[2] = 0;
    hessCols[2] = 2;
    hessRows[3] = 1;
    hessCols[3] = 1;
    hessRows[4] = 2;
    hessCols[4] = 2;

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

    dObj = 1.0e3 - x[0]*x[0] - 2.0e0*x[1]*x[1] - x[2]*x[2]
                 - x[0]*x[1] - x[0]*x[2];

    /*---- LINEAR EQUALITY CONSTRAINT. */
    c[0] = 8.0e0*x[0] + 14.0e0*x[1] + 7.0e0*x[2] - 56.0e0;

    /*---- QUADRATIC INEQUALITY CONSTRAINTS. */
    c[1] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] - 25.0e0;

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
    /*---- GRADIENT OF THE OBJECTIVE FUNCTION. */
    objGrad[0] = -2.0e0*x[0] - x[1] - x[2];
    objGrad[1] = -4.0e0*x[1] - x[0];
    objGrad[2] = -2.0e0*x[2] - x[0];

    /*---- GRADIENT OF THE FIRST CONSTRAINT, c[0]. */
    jac[0] =  8.0e0;
    jac[1] = 14.0e0;
    jac[2] =  7.0e0;


    /*---- GRADIENT OF THE SECOND CONSTRAINT, c[1]. */
    jac[3] = 2.0e0*x[0];
    jac[4] = 2.0e0*x[1];
    jac[5] = 2.0e0*x[2];

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
    hess[0] = -2.0e0*objScaler + 2.0e0*lambda[1];
    hess[1] = -1.0e0*objScaler;
    hess[2] = -1.0e0*objScaler;
    hess[3] = -4.0e0*objScaler + 2.0e0*lambda[1];
    hess[4] = -2.0e0*objScaler + 2.0e0*lambda[1];

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
    double  tmp[3];
    int     i;

    tmp[0] = (-2.0e0*objScaler + 2.0e0*lambda[1])*vector[0]
             - objScaler*vector[1] - objScaler*vector[2];
    tmp[1] = -objScaler*vector[0] + (-4.0e0*objScaler + 2.0e0*lambda[1])*vector[1];
    tmp[2] = -objScaler*vector[0] + (-2.0e0*objScaler + 2.0e0*lambda[1])*vector[2];

    for (i = 0; i < 3; i++)
        vector[i] = tmp[i];

    return;
}


/*----- End of source code -----------------------------------------*/
