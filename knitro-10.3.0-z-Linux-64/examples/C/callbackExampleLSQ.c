/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  Knitro example driver using callback mode, defining separate
 *  callback functions for each evaluation request.
 *  See callbackExample2.c for an example that combines callback functions.
 *
 *  This executable invokes Knitro to solve a simple nonlinear
 *  least-squares test problem.  The purpose is to illustrate how to
 *  invoke Knitro using the C language API.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <stdlib.h>

#include "knitro.h"
#include "problemDef.h"

/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalRv                                     */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_lsq_callback in knitro.h.
 *  Only "res" is modified.
 */
int callbackEvalR ( const int            n,
                    const int            m,
                    const int            nnzJ,
                    const double * const x,
                          double * const res,
                          double * const jac,
                          void *         userParams )
{
    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemDef.h. */
    computeR (x, res);
    return( 0 );
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalJv                                      */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_lsq_callback in knitro.h.
 *  Only "jac" is modified.
 */
int callbackEvalJ  ( const int            n,
                     const int            m,
                     const int            nnzJ,
                     const double * const x,
                           double * const res,
                           double * const jac,
                           void *         userParams )
{
    /*---- IN THIS EXAMPLE, CALL THE ROUTINE IN problemDef.h. */
    computeJ (x, jac);
    return( 0 );
}

/*------------------------------------------------------------------*/
/*     main                                                         */
/*------------------------------------------------------------------*/
int  main (int  argc, char  *argv[])
{
    int  nStatus;

    /*---- DECLARE VARIABLES THAT ARE PASSED TO KNITRO. */
    KTR_context  *kc;
    int          n, m, nnzJ;
    int          *jacIndexVars, *jacIndexRes;
    double       obj, *x, *lambda;
    double       *xLoBnds, *xUpBnds, *xInitial;

    /*---- FETCH THE SIZES OF THE PROBLEM TO BE SOLVED. */
    getLsqProblemSizes (&n, &m, &nnzJ);

    /*---- ALLOCATE MEMORY FOR THE PROBLEM DEFINITION. */
    xLoBnds      = (double *) malloc (n * sizeof(double));
    xUpBnds      = (double *) malloc (n * sizeof(double));
    xInitial     = (double *) malloc (n * sizeof(double));
    jacIndexVars = (int    *) malloc (nnzJ * sizeof(int));
    jacIndexRes  = (int    *) malloc (nnzJ * sizeof(int));

    /*---- FETCH THE DEFINITION OF THE PROBLEM TO BE SOLVED. */
    getLsqProblemData ( xLoBnds, xUpBnds, xInitial,
                        jacIndexVars, jacIndexRes );

    /*---- NOW THAT WE HAVE THE PROBLEM SIZE, ALLOCATE ARRAYS THAT ARE
     *---- PASSED TO KNITRO DURING THE SOLVE.
     *----
     *---- NOTICE lambda HAS MULTIPLIERS FOR BOTH BOUNDS ONLY IN THE CASE OF
     *---- BOUND CONSTRAINED NONLINEAR LEAST-SQUARES PROBLEMS.
     */
    x      = (double *) malloc (n     * sizeof(double));
    lambda = (double *) malloc (n     * sizeof(double));

    /*---- CREATE A NEW KNITRO SOLVER INSTANCE. */
    kc = KTR_new();
    if (kc == NULL)
    {
        printf ("Failed to find a valid license.\n");
        return( -1 );
    }

    /*---- REGISTER THE CALLBACK FUNCTIONS THAT PERFORM PROBLEM EVALUATION.
     */
    if (KTR_lsq_set_res_callback(kc, &callbackEvalR) != 0)
        return( -1 );
    if (KTR_lsq_set_jac_callback(kc, &callbackEvalJ) != 0)
        return( -1 );

    /*---- PERFORM A DERIVATIVE CHECK. */
    if (KTR_set_int_param (kc, KTR_PARAM_DERIVCHECK, KTR_DERIVCHECK_ALL) != 0)
        return( -1 );
            
    /*---- INITIALIZE KNITRO WITH THE PROBLEM DEFINITION. */
    nStatus = KTR_lsq_init_problem( kc, n,
                                    xLoBnds, xUpBnds,
                                    m, NULL, nnzJ, 
                                    jacIndexVars, jacIndexRes,
                                    xInitial, NULL);

    /*---- KNITRO KEEPS ITS OWN COPY OF THE PROBLEM DEFINITION,
     *---- SO THE LOCAL MEMORY CAN BE FREED IMMEDIATELY. */
    free (xLoBnds);
    free (xUpBnds);
    free (xInitial);
    free (jacIndexVars);
    free (jacIndexRes);

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
