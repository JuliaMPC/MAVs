/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#ifndef PROBLEM_DEF_H__
#define PROBLEM_DEF_H__


/** This is just one example of how to present optimization problem
 *  information to a C language solver driver.  Problem information
 *  includes:
 *  - defining sizes (number unknowns, sparsity nonzeroes, etc.)
 *  - loading problem definition arrays (bounds, sparsity structure, etc.)
 *  - providing routines that compute:
 *    - function and constraint values at a point
 *    - function and constraint first derivatives at a point
 *    - function and constraint second derivatives at a point (optional)
 *  This example provides routines for all derivatives.
 */

/*------------------------------------------------------------------*/
/*     FUNCTION DECLARATIONS                                        */
/*------------------------------------------------------------------*/

/** Define sizes for the problem.
 */
void  getProblemSizes (int *  const  n,         /*-- NUMBER UNKNOWNS */
                       int *  const  m,         /*-- NUMBER CONSTRAINTS */
                       int *  const  nnzJ,      /*-- JACOBIAN NONZEROES */
                       int *  const  nnzH);     /*-- HESSIAN NONZEROES */

 void getLsqProblemSizes (int * const n,
                          int * const m, 
                          int * const nnzJ);

/** Load Knitro arrays for the problem definition.
 *  The function assumes array memory has been allocated to match
 *  the problem sizes.
 */
void  getProblemData (int    * const  objType,       /*-- SCALAR */
                      int    * const  objGoal,       /*-- SCALAR */
                      double * const  xLoBnds,       /*-- ARRAY LENGTH n */
                      double * const  xUpBnds,       /*-- ARRAY LENGTH n */
                      double * const  xInitial,      /*-- ARRAY LENGTH n */
                      int    * const  cType,         /*-- ARRAY LENGTH m */
                      double * const  cLoBnds,       /*-- ARRAY LENGTH m */
                      double * const  cUpBnds,       /*-- ARRAY LENGTH m */
                      int    * const  jacIndexVars,  /*-- ARRAY LENGTH nnzJ */
                      int    * const  jacIndexCons,  /*-- ARRAY LENGTH nnzJ */
                      int    * const  hessRows,      /*-- ARRAY LENGTH nnzH */
                      int    * const  hessCols,      /*-- ARRAY LENGTH nnzH */
                      int    * const  objFnType,     /*-- MIP SCALAR */
                      int    * const  xType,         /*-- MIP ARRAY LENGTH n */
                      int    * const  cFnType);      /*-- MIP ARRAY LENGTH m */
                 
/** Load Knitro arrays for the definition of nonlinear
* least-squares problems. The function assumes array memory has been allocated to match
* the problem sizes.
*/
void getLsqProblemData ( double * const xLoBnds,      /*-- ARRAY LENGTH n */
                         double * const xUpBnds,      /*-- ARRAY LENGTH n */
                         double * const xInitial,     /*-- ARRAY LENGTH n */
                         int    * const jacIndexVars, /*-- ARRAY LENGTH nnzJ */
                         int    * const jacIndexCons);/*-- ARRAY LENGTH nnzJ */ 

/** Compute the function and constraint values at x.
 */
double  computeFC (const double * const  x,
                         double * const  c);

/** Compute the residual values at x.
 */
void  computeR ( const double * const x,
                       double * const res);

/** Compute the function and constraint first derivatives at x.
 */
void  computeGA (const double * const  x,
                       double * const  objGrad,
                       double * const  jac);

/** Compute the residuals jacobian at x. 
 */
void  computeJ (const double * const x,
                      double * const jac);

/** Compute the Hessian of the Lagrangian at x and lambda.
 */
void  computeH (const double * const  x,
                const double          objScaler,
                const double * const  lambda,
                      double * const  hess);


/** Compute the Hessian of the Lagrangian times vector at x and lambda.
 *  Return the result in vector.
 */
void  computeHV (const double * const  x,
                 const double          objScaler,
                 const double * const  lambda,
                       double * const  vector);


#endif     /*-- PROBLEM_DEF_H__ */
