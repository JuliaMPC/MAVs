/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This file contains an example wrapper for the C language Knitro
 *  application programming interface (API).  The wrapper holds the
 *  KTR_context structure, which cannot be passed to Fortran.  
 *  Function names are exported in the form required for a Fortran
 *  linker (no capital letters, trailing underscores).
 *
 *  The wrapper assumes array indices that embody the structure of
 *  sparse problem derivatives follow the C language convention and
 *  start numbering from zero.
 *
 *  This file is merely an example.  Not all Knitro API calls are made
 *  available.  Consult the Knitro manual and "knitro.h" for other calls.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*------------------------------------------------------------------*/
/*     INCLUDES                                                     */
/*------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "knitro.h"


/*------------------------------------------------------------------*/
/*     FILE GLOBAL ITEMS                                            */
/*------------------------------------------------------------------*/

const char  szName[] = { "--- (knitro_fortran) " };

static KTR_context *  g_kc = NULL;



/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_open_instance                                  */
/*------------------------------------------------------------------*/
/** Create a new Knitro instance.
 *  Exit with an error message if it failed.
 */
#if defined(_WIN32)
void  KTRF_OPEN_INSTANCE (void)
#else
void  ktrf_open_instance_ (void)
#endif
{
    if (g_kc != NULL)
        {
        fprintf (stderr, "%sKnitro solver instance already exists.\n", szName);
        exit( -1 );
        }
    g_kc = KTR_new();
    if (g_kc == NULL)
        {
        fprintf (stderr, "%sFailed to create Knitro solver instance.\n", szName);
        exit( -1 );
        }
    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_load_param_file                                */
/*------------------------------------------------------------------*/
/** Tell Knitro to load user options from the standard parameter file.
 */
#if defined(_WIN32)
void  KTRF_LOAD_PARAM_FILE (void)
#else
void  ktrf_load_param_file_ (void)
#endif
{
    int  nStatus;

    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }

    nStatus = KTR_load_param_file (g_kc, "knitro.opt");
    if (nStatus != 0)
        {
        fprintf (stderr, "%sKnitro returned error code %d from KTR_load_param_file.\n",
                 szName, nStatus);
        exit( -1 );
        }
    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_init_problem                                   */
/*------------------------------------------------------------------*/
/** Pass the problem definition to Knitro.
 *  Assume array indices in jacIndexVars, jacIndexCons, hessIndexRows,
 *  and hessIndexCols follow the C convention and start numbering
 *  from zero (the Fortran convention is to start from one).
 */
#if defined(_WIN32)
void  KTRF_INIT_PROBLEM
#else
void  ktrf_init_problem_
#endif
                          (int    * const  n,
                           int    * const  objGoal,
                           int    * const  objType,
                           double * const  xLoBnds,
                           double * const  xUpBnds,
                           int    * const  m,
                           int    * const  cType,
                           double * const  cLoBnds,
                           double * const  cUpBnds,
                           int    * const  nnzJ,
                           int    * const  jacIndexVars,
                           int    * const  jacIndexCons,
                           int    * const  nnzH,
                           int    * const  hessIndexRows,
                           int    * const  hessIndexCols,
                           double * const  xInitial)
{
    int  nStatus;

    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }

    nStatus =  KTR_init_problem (g_kc, *n, *objGoal, *objType,
                                 xLoBnds, xUpBnds,
                                 *m, cType, cLoBnds, cUpBnds,
                                 *nnzJ, jacIndexVars, jacIndexCons,
                                 *nnzH, hessIndexRows, hessIndexCols,
                                 xInitial, NULL);
    if (nStatus != 0)
        {
        fprintf (stderr, "%sKnitro returned error code %d from KTR_init_problem.\n",
                 szName, nStatus);
        exit( -1 );
        }

    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_solve                                          */
/*------------------------------------------------------------------*/
/** Invoke the solver to compute its next iteration.
 */
#if defined(_WIN32)
void  KTRF_SOLVE
#else
void  ktrf_solve_
#endif
                   (double * const  x,            /*--       OUTPUT */
                    double * const  lambda,       /*--       OUTPUT */
                    int    * const  evalStatus,   /*-- REV. COMM    */
                    double * const  obj,          /*-- INPUT OUTPUT */
                    double * const  c,            /*-- REV. COMM.   */
                    double * const  objGrad,      /*-- REV. COMM.   */
                    double * const  jac,          /*-- REV. COMM.   */
                    double * const  hess,         /*-- REV. COMM.   */
                    double * const  hvector,      /*-- REV. COMM.   */
                    int    * const  status)       /*-- REV. COMM.   */

{
    int nStatus;
    
    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }

    nStatus = KTR_solve (g_kc, x, lambda, 0, obj,
                         NULL,NULL,NULL,NULL,NULL, NULL);

    return;
}


/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_close_instance                                 */
/*------------------------------------------------------------------*/
/** Delete the Knitro instance.
 *  Exit with an error message if it failed.
 */
#if defined(_WIN32)
void  KTRF_CLOSE_INSTANCE (void)
#else
void  ktrf_close_instance_ (void)
#endif
{
    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }
    if (KTR_free (&g_kc) != 0)
        {
        fprintf (stderr, "%sFailed to close Knitro solver instance.\n", szName);
        exit( -1 );
        }
    g_kc = NULL;
    return;
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalFC                                      */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "obj" and "c" are modified.
 */

/*fortran function*/
void compute_fc_(const double * const x, double * const obj, double * const c); 


int  callbackEvalFC (const int             evalRequestCode,
                     const int             n,
                     const int             m,
                     const int             nnzJ,
                     const int             nnzH,
                     const double * const  x,
                     const double * const  lambda,
                           double * const  obj,
                           double * const  c,
                           double * const  objGrad,
                           double * const  jac,
                           double * const  hessian,
                           double * const  hessVector,
                           void   *        userParams)
{
    if (evalRequestCode != KTR_RC_EVALFC)
    {
        printf ("*** callbackEvalFC incorrectly called with eval code %d\n",
                evalRequestCode);
        return( -1 );
    }

    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemQCQP.f. */
    compute_fc_ (x, obj, c);
    return( 0 );
}


/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_set_func_callback                              */
/*------------------------------------------------------------------*/
/** Set function callback.
 */
#if defined(_WIN32)
void  KTRF_SET_FUNC_CALLBACK (void)
#else
void  ktrf_set_func_callback_ (void)
#endif
{
    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }

    if (KTR_set_func_callback (g_kc, callbackEvalFC) != 0)
        {
        fprintf (stderr, "%sFailed to set function callback.\n", szName);
        exit( -1 );
        }

    return;
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalGA                                      */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "objGrad" and "jac" are modified.
 */

/*fortran function*/
void compute_ga_(const double * const x, double * const objGrad, double * const jac); 

int  callbackEvalGA (const int             evalRequestCode,
                     const int             n,
                     const int             m,
                     const int             nnzJ,
                     const int             nnzH,
                     const double * const  x,
                     const double * const  lambda,
                           double * const  obj,
                           double * const  c,
                           double * const  objGrad,
                           double * const  jac,
                           double * const  hessian,
                           double * const  hessVector,
                           void   *        userParams)
{
    if (evalRequestCode != KTR_RC_EVALGA)
    {
        printf ("*** callbackEvalGA incorrectly called with eval code %d\n",
                evalRequestCode);
        return( -1 );
    }

    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemQCQP.f. */
    compute_ga_ (x, objGrad, jac);
    return( 0 );
}

/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_set_grad_callback                              */
/*------------------------------------------------------------------*/
/** Set gradient callback.
 */
#if defined(_WIN32)
void  KTRF_SET_GRAD_CALLBACK (void)
#else 
void  ktrf_set_grad_callback_ (void)
#endif
{
    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }

    if (KTR_set_grad_callback (g_kc, callbackEvalGA) != 0)
        {
        fprintf (stderr, "%sFailed to set gradient callback.\n", szName);
        exit( -1 );
        }

    return;
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalH                                       */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "hessian" or "hessVector" are modified.
 */

/*fortran functions*/
void compute_h_(const double * const x, const double * const objscaler,
                const double * const lambda, double * const hessian); 
void compute_hv_(const double * const x, const double * const objscaler,
                 const double * const lambda, double * const hessian);

int  callbackEvalH (const int             evalRequestCode,
                     const int             n,
                     const int             m,
                     const int             nnzJ,
                     const int             nnzH,
                     const double * const  x,
                     const double * const  lambda,
                           double * const  obj,
                           double * const  c,
                           double * const  objGrad,
                           double * const  jac,
                           double * const  hessian,
                           double * const  hessVector,
                           void   *        userParams)
{
    double objscaler;
    
    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemQCQP.f. */
    switch (evalRequestCode)
    {
    case KTR_RC_EVALH:
        objscaler = 1.0;
        compute_h_ (x, &objscaler, lambda, hessian);
        break;
    case KTR_RC_EVALH_NO_F:
        objscaler = 0.0;
        compute_h_ (x, &objscaler, lambda, hessian);
        break;
    case KTR_RC_EVALHV:
        objscaler = 1.0;
        compute_hv_ (x, &objscaler, lambda, hessVector);
        break;
    case KTR_RC_EVALHV_NO_F:
        objscaler = 0.0;
        compute_hv_ (x, &objscaler, lambda, hessVector);
        break;        
    default:
        printf ("*** callbackEvalHess incorrectly called with eval code %d\n",
                evalRequestCode);
        return( -1 );
    }
    return( 0 );
}

/*------------------------------------------------------------------*/
/*     FUNCTION ktrf_set_hess_callback                              */
/*------------------------------------------------------------------*/
/** Set hessian callback.
 */
#if defined(_WIN32)
void  KTRF_SET_HESS_CALLBACK (void)
#else
void  ktrf_set_hess_callback_ (void)
#endif
{
    if (g_kc == NULL)
        {
        fprintf (stderr, "%sKnitro solver instance does not exist.\n", szName);
        exit( -1 );
        }

    if (KTR_set_hess_callback (g_kc, callbackEvalH) != 0)
        {
        fprintf (stderr, "%sFailed to set hessian callback.\n", szName);
        exit( -1 );
        }

    return;
}


/*---- End of source code ------------------------------------------*/
