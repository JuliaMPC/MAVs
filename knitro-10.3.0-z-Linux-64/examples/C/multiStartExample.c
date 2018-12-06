/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  Knitro multi-start example driver defining separate callback
 *  functions for each evaluation request, as well as a callback
 *  function to perform some user-defined task after each multi-start
 *  solve.
 *
 *  This executable invokes Knitro to solve a simple nonlinear
 *  optimization test problem.  The purpose is to
 *  illustrate how to invoke Knitro using the C language API.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "knitro.h"
#include "problemDef.h"


/*------------------------------------------------------------------*/
/*     FUNCTION callbackEvalFC                                      */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "obj" and "c" are modified.
 */
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

    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemDef.h. */
    *obj = computeFC (x, c);
    return( 0 );
}


/*------------------------------------------------------------------*/
/*     FUNCTION callbackEvalGA                                      */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "objGrad" and "jac" are modified.
 */
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

    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemDef.h. */
    computeGA (x, objGrad, jac);
    return( 0 );
}


/*------------------------------------------------------------------*/
/*     FUNCTION callbackEvalHess                                    */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "hessian" or "hessVector" is modified.
 */
int  callbackEvalHess (const int             evalRequestCode,
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
    /*---- IN THIS EXAMPLE, CALL THE ROUTINES IN problemDef.h. */
    switch (evalRequestCode)
    {
    case KTR_RC_EVALH:
        computeH (x, 1.0, lambda, hessian);
        break;
    case KTR_RC_EVALH_NO_F:
        computeH (x, 0.0, lambda, hessian);
        break;
    case KTR_RC_EVALHV:
        computeHV (x, 1.0, lambda, hessVector);
        break;
    case KTR_RC_EVALHV_NO_F:
        computeHV (x, 0.0, lambda, hessVector);
        break;
    default:
        printf ("*** callbackEvalHess incorrectly called with eval code %d\n",
                evalRequestCode);
        return( -1 );
    }

    return( 0 );
}


/*------------------------------------------------------------------*/
/*     FUNCTION callbackMSProcess                                   */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 */
int  callbackMSProcess (const int             evalRequestCode,
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
    int i;
    
    /*---- PRINT SOLUTION OF THE JUST COMPLETED MULTI-START SOLVE. */
    printf ("callbackMSProcess: \n");
    printf ("    Last solution: obj=%e\n", *obj);
    for (i=0; i<n; i++)
        printf ("                   x[%d]=%e\n",i,x[i]);     

    return( 0 );
}

/*------------------------------------------------------------------*/
/*     main                                                         */
/*------------------------------------------------------------------*/
int  main (int  argc, char  *argv[])
{
    int  nStatus;
    int  nHessOpt;

    /*---- DECLARE VARIABLES THAT ARE PASSED TO KNITRO. */
    KTR_context  *kc;
    int          n, m, nnzJ, nnzH, objGoal, objType;
    int          *cType;
    int          *jacIndexVars, *jacIndexCons, *hessRows, *hessCols;
    double       obj, *x, *lambda, *xInitial;
    double       *xLoBnds, *xUpBnds, *cLoBnds, *cUpBnds;

    /*---- FETCH THE SIZES OF THE PROBLEM TO BE SOLVED. */
    getProblemSizes (&n, &m, &nnzJ, &nnzH);

    /*---- ALLOCATE MEMORY FOR THE PROBLEM DEFINITION. */
    xInitial 		 = (double *) malloc (n * sizeof(double));
    xLoBnds      = (double *) malloc (n * sizeof(double));
    xUpBnds      = (double *) malloc (n * sizeof(double));
    cType        = (int    *) malloc (m * sizeof(int));
    cLoBnds      = (double *) malloc (m * sizeof(double));
    cUpBnds      = (double *) malloc (m * sizeof(double));
    jacIndexVars = (int    *) malloc (nnzJ * sizeof(int));
    jacIndexCons = (int    *) malloc (nnzJ * sizeof(int));
    hessRows     = (int    *) malloc (nnzH * sizeof(int));
    hessCols     = (int    *) malloc (nnzH * sizeof(int));

    /*---- FETCH THE DEFINITION OF THE PROBLEM TO BE SOLVED. */
    getProblemData (&objType, &objGoal, xLoBnds, xUpBnds, xInitial,
                    cType, cLoBnds, cUpBnds,
                    jacIndexVars, jacIndexCons,
                    hessRows, hessCols, NULL, NULL, NULL);
                    
    /*---- NOW THAT WE HAVE THE PROBLEM SIZE, ALLOCATE ARRAYS THAT ARE
     *---- PASSED TO KNITRO DURING THE SOLVE.
     *----
     *---- NOTICE lambda HAS MULTIPLIERS FOR BOTH CONSTRAINTS AND BOUNDS
     *---- (A COMMON MISTAKE IS TO ALLOCATE ITS SIZE AS m).
     */
    x       = (double *) malloc (n     * sizeof(double));
    lambda  = (double *) malloc ((m+n) * sizeof(double));

    /*---- CREATE A NEW KNITRO SOLVER INSTANCE. */
    kc = KTR_new();
    if (kc == NULL)
    {
        printf ("Failed to find a valid license.\n");
        return( -1 );
    }

    /*---- REGISTER THE CALLBACK FUNCTIONS THAT PERFORM PROBLEM EVALUATION.
     *---- THE HESSIAN CALLBACK ONLY NEEDS TO BE REGISTERED FOR SPECIFIC
     *---- HESSIAN OPTIONS (E.G., IT IS NOT REGISTERED IF THE OPTION FOR
     *---- BFGS HESSIAN APPROXIMATIONS IS SELECTED).
     */

    KTR_set_int_param_by_name (kc, "gradopt", KTR_GRADOPT_EXACT);
    KTR_set_int_param_by_name (kc, "hessopt", KTR_HESSOPT_EXACT);
    KTR_set_int_param_by_name (kc, "outlev", 6);
 
    if (KTR_set_func_callback (kc, &callbackEvalFC) != 0)
        exit( -1 );

    if (KTR_set_grad_callback (kc, &callbackEvalGA) != 0)
        exit( -1 );

    if (KTR_get_int_param_by_name (kc, "hessopt", &nHessOpt) != 0)
        exit( -1 );

    if ((nHessOpt == KTR_HESSOPT_EXACT) || (nHessOpt == KTR_HESSOPT_PRODUCT))
    {
        if (KTR_set_hess_callback (kc, &callbackEvalHess) != 0)
            exit( -1 );
    }

    /*---- ENABLE THE MULTISTART PROCEDURE */
    KTR_set_int_param (kc, KTR_PARAM_MULTISTART, KTR_MULTISTART_YES);
    /*---- RUN THE MULTISTART PROCEDURE IN PARALLEL WITH 4 THREADS */
    KTR_set_int_param (kc, KTR_PARAM_PAR_NUMTHREADS, 4);
    
    /*---- REGISTER THE CALLBACK FUNCTION THAT PERFORMS SOME TASK AFTER
     *---- EACH MULTISTART SOLVE. */
    if (KTR_set_ms_process_callback (kc, &callbackMSProcess) != 0)
        exit( -1 );
		
    /*---- INITIALIZE KNITRO WITH THE PROBLEM DEFINITION. */
    nStatus = KTR_init_problem (kc, n, objGoal, objType,
                                xLoBnds, xUpBnds,
                                m, cType, cLoBnds, cUpBnds,
                                nnzJ, jacIndexVars, jacIndexCons,
                                nnzH, hessRows, hessCols, xInitial, NULL);

    /*---- KNITRO KEEPS ITS OWN COPY OF THE PROBLEM DEFINITION,
     *---- SO THE LOCAL MEMORY CAN BE FREED IMMEDIATELY. */
    free (xLoBnds);
    free (xUpBnds);
    free (cType);
    free (cLoBnds);
    free (cUpBnds);
    free (jacIndexVars);
    free (jacIndexCons);
    free (hessRows);
    free (hessCols);

    /*---- SOLVE THE PROBLEM.
     *----
     *---- RETURN STATUS CODES ARE DEFINED IN "knitro.h" AND DESCRIBED
     *---- IN THE KNITRO MANUAL.
     */
    nStatus = KTR_solve (kc, x, lambda, 0, &obj,
                             NULL, NULL, NULL, NULL, NULL, NULL);

    printf ("\n\n");
    if (nStatus != 0)
        printf ("Knitro failed to solve the problem, final status = %d\n",
                nStatus);
    else
    {
        /*---- AN EXAMPLE OF OBTAINING SOLUTION INFORMATION. */
                /*---- AN EXAMPLE OF OBTAINING SOLUTION INFORMATION. */
        printf ("Knitro successful, feasibility violation    = %e\n",
                KTR_get_abs_feas_error (kc));
        printf ("                   KKT optimality violation = %e\n",
                KTR_get_abs_opt_error (kc));
    }
    
    /*---- DELETE THE KNITRO SOLVER INSTANCE. */
    KTR_free (&kc);

    free (x);
    free (lambda);

    return( 0 );
}

/*----- End of source code -----------------------------------------*/
