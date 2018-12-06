/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#ifndef KNITRO_BLAS_LAPACK_H__
#define KNITRO_BLAS_LAPACK_H__


/** This C header file declares BLAS and LAPACK functions used by KNITRO.
 *  The KNITRO libraries already include an implementation of the functions,
 *  but this header file provides a way for your application to dynamically
 *  load a custom BLAS/LAPACK implementation.  Please refer to the user
 *  option "blasoption" in the KNITRO User Manual.
 *
 *  To supply your own BLAS/LAPACK, you must provide a dynamic library
 *  that implements every function in this file, using call signatures that
 *  are compatible with the declarations in this header.  Incompatible call
 *  signatures may cause KNITRO to crash.
 *  (Note: absence of "const" qualifiers does not cause incompatibilities.)
 *  If your BLAS/LAPACK implementation has different calling signatures,
 *  then you must create wrapper functions that implement the calls
 *  in this header file.
 *
 *  KNITRO uses only a small subset of BLAS and LAPACK functions.  Future
 *  releases of KNITRO may add more BLAS/LAPACK functions.
 *  There should be no problem linking an implementation with extra
 *  BLAS and LAPACK functions; they simply will not be called by KNITRO.
 *
 *  The file examples/C/blasAcmlExample.c shows how to create a wrapper
 *  for the AMD Core Math Library (ACML).
 */


/*------------------------------------------------------------------*/
/*     IMPORT MACROS                                                */
/*------------------------------------------------------------------*/
/** This macro defines the functions as exports from your DLL to
 *  KNITRO.  The cdecl convention is used on Windows because it is
 *  the default for the Microsoft Visual C++ compiler.
 */

#if defined(_WIN32)
  #define KNITRO_EXPORT  __declspec(dllexport) __cdecl
#else
  #define KNITRO_EXPORT
#endif


/*------------------------------------------------------------------*/
/*     BLAS LEVEL 1 FUNCTION DECLARATIONS                           */
/*------------------------------------------------------------------*/

/** Return the sum of the absolute values of vector xx.
 */
double  KNITRO_EXPORT  KTR_dasum (const int             n,
                                  const double * const  x,
                                  const int             incx);

/** Compute y = alpha*x + y.
 */
void    KNITRO_EXPORT  KTR_daxpy (const int             n,
                                  const double          alpha,
                                  const double * const  x,
                                  const int             incx,
                                        double * const  y,
                                  const int             incy);

/** Copy vector x to vector y.
 */
void    KNITRO_EXPORT  KTR_dcopy (const int             n,
                                  const double * const  x,
                                  const int             incx,
                                        double * const  y,
                                  const int             incy);

/** Return the dot product of vectors x and y.
 */
double  KNITRO_EXPORT  KTR_ddot  (const int             n,
                                  const double * const  x,
                                  const int             incx,
                                  const double * const  y,
                                  const int             incy);

/** Return the L2-norm of the vector x.
 */
double  KNITRO_EXPORT  KTR_dnrm2 (const int             n,
                                  const double * const  x,
                                  const int             incx);

/** Multiply the vector x by alpha.
 */
void    KNITRO_EXPORT  KTR_dscal (const int             n,
                                  const double          alpha,
                                        double * const  x,
                                  const int             incx);

/** Swap vectors x and y.
 */
void    KNITRO_EXPORT  KTR_dswap (const int             n,
                                        double * const  x,
                                  const int             incx,
                                        double * const  y,
                                  const int             incy);

/** Return the index of the element of x with largest absolute magnitude.
 *  Indexes must be numbered from zero to n-1.
 */
int     KNITRO_EXPORT KTR_idamax (const int             n,
                                  const double * const  x,
                                  const int             incx);


/*------------------------------------------------------------------*/
/*     BLAS LEVEL 2 FUNCTION DECLARATIONS                           */
/*------------------------------------------------------------------*/

/** Compute  y = alpha*A*x + beta*y   or   y = alpha*A'*x + beta*y,
 *  depending on whether "trans" is 111 or 112.
 *  Matrix "A" must be dense.  The value of "order" is 101 if A is
 *  in row major order, or 102 if in column major order.
 *  (Values for "order" and "trans" correspond to Intel MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dgemv (const int             order,
                                const int             trans,
                                const int             m,
                                const int             n,
                                const double          alpha,
                                const double * const  A,
                                const int             lda,
                                const double * const  x,
                                const int             incx,
                                const double          beta,
                                      double * const  y,
                                const int             incy);

/** Solves A*x = b   or   A'*x = b, depending on whether "trans" is
 *  111 or 112.  Matrix "A" is an n by n unit, or non-unit, upper or
 *  lower triangular matrix, supplied in packed form.  On entry, "x"
 *  contains the  right-hand side vector b; on exit it contains the
 *  solution vector x.
 *  The value of "order" is 101 if A is in row major order, or 102 if
 *  in column major order.  The value of "uplo" is 121 if A is upper
 *  triangular, or 122 if lower triangular.  The value of "diag" is
 *  131 if A is NOT assumed to be unit triangular and 132 if A is
 *  assumed to be unit triangular.
 *  (Values for "order", "uplo", "trans" and "diag" correspond to Intel
 *  MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dtpsv (const int             order,
                                const int             uplo,
                                const int             trans,
                                const int             diag,
                                const int             n,
                                const double * const  A,
                                      double * const  x,
                                const int             incx);

/** Solves A*x = b   or   A'*x = b, depending on whether "trans" is
 *  111 or 112.  Matrix "A" is an n by n unit, or non-unit, upper or
 *  lower triangular matrix.  On entry, "x" contains the  right-hand
 *  side vector b; on exit it contains the solution vector x.
 *  The value of "order" is 101 if A is in row major order, or 102 if
 *  in column major order.  The value of "uplo" is 121 if A is upper
 *  triangular, or 122 if lower triangular.  The value of "diag" is
 *  131 if A is NOT assumed to be unit triangular and 132 if A is
 *  assumed to be unit triangular.
 *  (Values for "order", "uplo", "trans" and "diag" correspond to Intel
 *  MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dtrmv (const int             order,
                                const int             uplo,
                                const int             trans,
                                const int             diag,
                                const int             n,
                                const double * const  A,
                                const int             lda,
                                      double * const  x,
                                const int             incx);

/** Performs the rank 1 operation
 *     A = alpha*x*y' + A
 *  where alpha is a scalar, x is an m element vector, y is an n
 *  element vector and A is an m by n matrix.
 *  The value of "order" is 101 if A is in row major order, or 102 if
 *  in column major order.
 *  (The value for "order" corresponds to Intel MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dger (const int             order,
                               const int             m,
                               const int             n,
                               const double          alpha,
                               const double * const  x,
                               const int             incx,
                               const double * const  y,
                               const int             incy,
                                     double * const  A,
                               const int             lda); 

/** Performs the matrix-vector operation 
 *     y = alpha*A*x + beta*y, 
 *  where alpha and beta are scalars, x and y are n element vectors
 *  and A is an n by n symmetric matrix, supplied in packed form.
 *  The value of "order" is 101 if A is in row major order, or 102 if
 *  in column major order. The value of "uplo" is 121 if A is upper
 *  triangular, or 122 if lower triangular.
 *  (The values for "order" and "uplo" corresponds to Intel MKL
 *  definitions.)
 */
void  KNITRO_EXPORT  KTR_dspmv (const int             order,
                                const int             uplo,
                                const int             n,
                                const double          alpha,
                                const double * const  A,
                                const double * const  x,
                                const int             incx,
                                const double          beta,
                                      double * const  y,
                                const int             incy);


/*------------------------------------------------------------------*/
/*     BLAS LEVEL 3 FUNCTION DECLARATIONS                           */
/*------------------------------------------------------------------*/

/** Compute C = alpha*op(A)*op(B) + beta*C  where
 *    op(A) = A if transA=111 or op(A) = A' if transA=112, and 
 *    op(B) = B if transB=111 or op(B) = B' if transB=112 
 *  Here op(A) is an m by k matrix, op(B) is a k by n matrix and C is
 *  an m by n matrix.  The value of "order" is 101 if the matrices
 *  are provided in row major order, or 102 if provided in column
 *  major order.
 *  (Values for "order", "transA" and "transB" correspond to Intel
 *  MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dgemm (const int             order,
                                const int             transA,
                                const int             transB,
                                const int             m,
                                const int             n,
                                const int             k,
                                const double          alpha,
                                const double * const  A,
                                const int             lda,
                                const double * const  B,
                                const int             ldb,
                                const double          beta,
                                      double * const  C,
                                const int             ldc);

/** Compute  B = alpha*op( A )*B,  if "side" = 141  or
 *           B = alpha*B*op( A ),  if "side" = 142 
 *  Here 
 *    op(A) = A if transA=111 or op(A) = A' if transA=112
 *  The value of "order" is 101 if A is in row major order, or 102 if
 *  in column major order.  The value of "uplo" is 121 if A is upper
 *  triangular, or 122 if lower triangular.  The value of "diag" is
 *  131 if A is NOT assumed to be unit triangular and 132 if A is
 *  assumed to be unit triangular.
 *  (Values for "order", "side", "uplo", "transA" and "diag" correspond
 *  to Intel MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dtrmm (const int             order,
                                const int             side,
                                const int             uplo,
                                const int             transA,
                                const int             diag, 
                                const int             m,
                                const int             n,
                                const double          alpha,
                                const double * const  A,
                                const int             lda,
                                      double * const  B,
                                const int             ldb);

/** Compute  op( A )*X = alpha*B,  if "side" = 141  or
 *           X*op( A ) = alpha*B,  if "side" = 142
 *  The matrix X is overwritten on B. Here 
 *    op(A) = A if transA=111 or op(A) = A' if transA=112
 *  The value of "order" is 101 if A is in row major order, or 102 if
 *  in column major order.  The value of "uplo" is 121 if A is upper
 *  triangular, or 122 if lower triangular.  The value of "diag" is
 *  131 if A is NOT assumed to be unit triangular and 132 if A is
 *  assumed to be unit triangular.
 *  (Values for "order", "side", "uplo", "transA" and "diag" correspond
 *  to Intel MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dtrsm (const int             order,
                                const int             side,
                                const int             uplo,
                                const int             transA,
                                const int             diag, 
                                const int             m,
                                const int             n,
                                const double          alpha,
                                const double * const  A,
                                const int             lda,
                                      double * const  B,
                                const int             ldb);

/** Compute  C = alpha*A*A' + beta*C, if trans=111, or
 *           C = alpha*A'*A + beta*C  if trans=112
 *  The value of "order" is 101 if matrices are provided in
 *  row major order, or 102 if in column major order.
 *  The value of "uplo" is 121 if C is upper triangular, or
 *  122 if lower triangular.
 *  (Values for "order", "uplo", and "trans" correspond to 
 *  Intel MKL definitions.)
 */
void KNITRO_EXPORT KTR_dsyrk (const int               order,
                              const int               uplo,
                              const int               trans,
                              const int               n,
                              const int               k,
                              const double            alpha,
                              const double   * const  A,
                              const int               lda,
                              const double            beta,
                                    double   * const  C,
                              const int               ldc);

#endif     /*-- KNITRO_BLAS_LAPACK_H__ */
