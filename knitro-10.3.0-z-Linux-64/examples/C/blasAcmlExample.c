/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This file wraps the BLAS/LAPACK functions of the AMD Core Math
 *  Library (ACML), making them available to Knitro as a dynamic library.
 *  Knitro imports the functions declared in "include/blas_lapack.h".
 *  ACML functions are identical except they are named differently,
 *  and "idamax" returns a Fortran-like index (numbered from one)
 *  instead of the required C-like index (numbered from zero).
 *
 *  This wrapper must be compiled and linked with the ACML into a
 *  dynamic library.  Then Knitro can use it by setting "blasoption=2"
 *  and setting "blasoptionlib" to the dynamic library.
 *  Currently, AMD provides the ACML for free with registration at
 *  their web site.  Several ACML versions are available, and users
 *  must decide which one is most appropriate for their machine.
 *
 *  Example using ACML 3.6 on 32-bit Windows:
 *    - Download acml3.6.0-32-ifort.exe and install to C:\Program Files\AMD
 *    - For convenience, set an environment variable:
 *        set ACML=C:\Program Files\AMD\acml3.6.0\ifort32
 *    - Compile this file:
 *        cl.exe /c /O2 -I..\..\include -I%ACML%\include blasAcmlExample.c
 *        (ignore warnings about 'const' qualifiers)
 *    - Create the wrapper DLL:
 *        link.exe /DLL /out:myacml.dll
 *                 blasAcmlExample.obj  %ACML%\lib\libacml_dll.lib
 *    - Before executing, add the ACML library to %PATH%:
 *        set PATH=%PATH%;%ACML%\lib
 *    - Use the Knitro API and set user options to:
 *        blasoption    = 2
 *        blasoptionlib = myacml.dll
 *
 *  Example using ACML 2.7 on 32-bit Linux:
 *    - Download acml2.7.0-gnu-32bit.tgz and install to /opt/acml2.7.0
 *      (Use ACML 2.7 because the machine runs an AMD Athlon XP 3200,
 *       which has no SSE2 assembler instructions.  ACML 3.6 requires SSE2
 *       and causes a core dump on this older processor.)
 *    - For convenience, set an environment variable:
 *        export ACML=/opt/acml2.7.0/gnu32_nosse2
 *    - Compile this file:
 *        gcc -c -O -I../../include -I$ACML/include blasAcmlExample.c
 *        (ignore 'discards qualifiers' warnings)
 *    - Create the wrapper shared object:
 *        gcc -shared -o myacml.so blasAcmlExample.o $ACML/lib/libacml.so -lg2c
 *    - Before executing, add the ACML library to $LD_LIBRARY_PATH:
 *        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACML/lib
 *    - Use the Knitro API and set user options to:
 *        blasoption    = 2
 *        blasoptionlib = myacml.so
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <stdlib.h>
#include "blas_lapack.h"     /*-- Knitro FUNCTION DECLARATIONS */
#include "acml.h"            /*-- ACML   FUNCTION DECLARATIONS */


/*------------------------------------------------------------------*/ 
/*     FUNCTIONS THAT TRIVIALLY WRAP ACML FUNCTIONS                 */
/*------------------------------------------------------------------*/

double  KNITRO_EXPORT  KTR_dasum (const int             n,
                                  const double * const  x,
                                  const int             incx)
{
    return( dasum (n, x, incx) );
}

void    KNITRO_EXPORT  KTR_daxpy (const int             n,
                                  const double          alpha,
                                  const double * const  x,
                                  const int             incx,
                                        double * const  y,
                                  const int             incy)
{
    daxpy (n, alpha, x, incx, y, incy);
    return;
}

void    KNITRO_EXPORT  KTR_dcopy (const int             n,
                                  const double * const  x,
                                  const int             incx,
                                        double * const  y,
                                  const int             incy)
{
    dcopy (n, x, incx, y, incy);
    return;
}

double  KNITRO_EXPORT  KTR_ddot  (const int             n,
                                  const double * const  x,
                                  const int             incx,
                                  const double * const  y,
                                  const int             incy)
{
    return( ddot (n, x, incx, y, incy) );
}

double  KNITRO_EXPORT  KTR_dnrm2 (const int             n,
                                  const double * const  x,
                                  const int             incx)
{
    return( dnrm2 (n, x, incx) );
}

void    KNITRO_EXPORT  KTR_dscal (const int             n,
                                  const double          alpha,
                                        double * const  x,
                                  const int             incx)
{
    dscal (n, alpha, x, incx);
    return;
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION KTR_idamax                                          */
/*------------------------------------------------------------------*/
/** Return the index of the element of x with largest absolute magnitude.
 *  ACML returns an index numbered from 1 to n, but Knitro requires
 *  an index numbered from zero to n-1.
 */
int     KNITRO_EXPORT KTR_idamax (const int             n,
                                  const double * const  x,
                                  const int             incx)
{
    return( idamax (n, x, incx) - 1 );
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION KTR_dgemv                                           */
/*------------------------------------------------------------------*/
/** Compute  y = alpha*A*x + beta*y   or   y = alpha*A'*x + beta*y,
 *  depending whether "trans" is 111 or 112.
 *  Matrix "A" must be dense.  The value of "order" is 101 if A is
 *  in row major order, or 102 if in column major order.
 *  (Values for "order" and "trans" correspond to Intel MKL definitions.)
 */
void  KNITRO_EXPORT  KTR_dgemv (const char            order,
                                const char            trans,
                                const int             m,
                                const int             n,
                                const double          alpha,
                                const double * const  A,
                                const int             lda,
                                const double * const  x,
                                const int             incx,
                                const double          beta,
                                      double * const  y,
                                const int             incy)
{
    char      codeTranspose;
    double *  newA;
    int       i, j, k;


    if ((n <= 0) || (m <= 0))
        return;

    if (trans == 111)
        codeTranspose = 'N';
    else if (trans == 112)
        codeTranspose = 'T';
    else
        /*---- NO WAY TO REPORT THIS ERROR. */
        return;

    if (order == 102)
    {
        /*---- A IS IN COLUMN MAJOR ORDER, AS EXPECTED BY ACML. */
        dgemv (codeTranspose, m, n, alpha, A, lda, x, incx, beta, y, incy);
        return;
    }
    else if (order != 101)
        /*---- NO WAY TO REPORT THIS ERROR. */
        return;

    /*---- MUST CHANGE A FROM ROW MAJOR ORDER TO COLUMN MAJOR. */
    newA = (double *) malloc (m * lda * sizeof (double));
    k = 0;
    for (j = 0; j < n; j++)
        for (i = 0; i < m; i++)
        {
            newA[k] = A[(lda * i) + j];
            k++;
        }
    dgemv (codeTranspose, m, n, alpha, newA, m, x, incx, beta, y, incy);
    free (newA);

    return;
}


/*----- End of source code -----------------------------------------*/
