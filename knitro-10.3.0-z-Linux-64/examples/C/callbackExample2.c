/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  Knitro example driver using callback mode, defining a single
 *  callback function for EVALFC / EVALGA requests, and a second
 *  callback for EVALH / EVALHV request.
 *  See callbackExample1.c for an example that uses separate callbacks.
 *
 *  This executable invokes Knitro to solve a simple nonlinear
 *  optimization test problem.  The purpose is to illustrate how to
 *  invoke Knitro using the C language API.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <stdio.h>
#include <stdlib.h>

#include "knitro.h"
#include "problemDef.h"


/*---- A POINTER TO THIS STRUCTURE WILL BE PASSED THRU KNITRO TO
 *---- REACH THE CALLBACK ROUTINE.
 */
typedef struct
{
    int  counter;
} UserCallbackType;


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalFCorGA                                  */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  If evalRequestCode is EVALFC, then all quantities are evaluated.
 *  If evalRequestCode is EVALGA, then it simply returns, because
 *  "x" will not change from the previous EVALFC request.
 */
int  callbackEvalFCorGA (const int             evalRequestCode,
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
    if (evalRequestCode == KTR_RC_EVALFC)
        {
        /*---- IN THIS EXAMPLE, CALL THE ROUTINES IN problemDef.h. */
        *obj = computeFC (x, c);
        computeGA (x, objGrad, jac);
        return( 0 );
        }
    else if (evalRequestCode == KTR_RC_EVALGA)
        return( 0 );

    printf ("*** callbackEvalFCorGA incorrectly called with eval code %d\n",
            evalRequestCode);
    return( -1 );
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackEvalHess                                    */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 *  Only "hessian" or "hessVector" is modified.
 *    The "userParams" argument simply counts how many Hessian evaluations
 *  have been called.  Its use illustrates how callbacks can store
 *  information while Knitro iterates.
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
    UserCallbackType *  pUser = (UserCallbackType *) userParams;

    /*---- IN THIS EXAMPLE, CALL THE ROUTINES IN problemDef.h. */
    switch (evalRequestCode)
    {
    case KTR_RC_EVALH:
        computeH (x, 1.0, lambda, hessian);
        pUser->counter++;        
        break;
    case KTR_RC_EVALH_NO_F:
        computeH (x, 0.0, lambda, hessian);
        pUser->counter++;
        break;
    case KTR_RC_EVALHV:
        computeHV (x, 1.0, lambda, hessVector);
        pUser->counter++;
        break;
    case KTR_RC_EVALHV_NO_F:
        computeHV (x, 0.0, lambda, hessVector);
        pUser->counter++;
        break;        
    default:
        printf ("*** callbackEvalHess incorrectly called with eval code %d\n",
                evalRequestCode);
        return( -1 );
    }

    /*---- THIS IS NOT VERY INTERESTING, BUT IT ILLUSTRATES userParams. */
    if (pUser->counter % 10 == 0)
        printf (">> Hessian computations have occurred %d times (msg from callbackExample2)\n",
                pUser->counter);

    return( 0 );
}


/*------------------------------------------------------------------*/ 
/*     FUNCTION callbackNewPoint                                    */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_newpt_callback in
 *  knitro.h.  Nothing should be modified.  This example prints out
 *  that Knitro has iterated to a new point (x, lambda) that it
 *  considers an improvement over the previous iterate, and prints
 *  out the current feasibility error and number of evaluations.
 *  To exercise it, edit "knitro.opt" and set the the "newpoint"
 *  option to "user".  The demonstration looks best if the "outlev"
 *  option is set to 5 or 6.
 */
int  callbackNewPoint (KTR_context_ptr       kc,
                       const int             n,
                       const int             m,
                       const int             nnzJ,
                       const double * const  x,
                       const double * const  lambda,
                       const double          obj,
                       const double * const  c,
                       const double * const  objGrad,
                       const double * const  jac,
                             void   *        userParams)
{
    int  i;
    int nFC;
    double dFeasError;
    UserCallbackType *  pUser = (UserCallbackType *) userParams;

    printf (">> New point computed by Knitro: (");
    for (i = 0; i < (n - 1); i++)
        printf ("%20.12e, ", x[i]);
    printf ("%20.12e)\n", x[n-1]);

    /*---- QUERY INFORMATION ABOUT THE CURRENT PROBLEM. */
    nFC = KTR_get_number_FC_evals(kc);
    printf("Number FC evals=%d, ",nFC);    
    dFeasError = KTR_get_abs_feas_error(kc);
    printf("Current feasError=%e\n",dFeasError);
    
    /*---- DEMONSTRATE USER-DEFINED TERMINATION */
    /*---- (Uncomment to activate) */
    /*
    if (   obj < 3.4e2
        && dFeasError <= 1.0e-6)
    {
        return(  KTR_RC_USER_TERMINATION );
    }
    */
    
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
    double       obj, *x, *lambda;
    double       *xLoBnds, *xUpBnds, *xInitial, *cLoBnds, *cUpBnds;

    UserCallbackType  userParams;


    /*---- FETCH THE SIZES OF THE PROBLEM TO BE SOLVED. */
    getProblemSizes (&n, &m, &nnzJ, &nnzH);

    /*---- ALLOCATE MEMORY FOR THE PROBLEM DEFINITION. */
    xLoBnds      = (double *) malloc (n * sizeof(double));
    xUpBnds      = (double *) malloc (n * sizeof(double));
    xInitial     = (double *) malloc (n * sizeof(double));
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
    x      = (double *) malloc (n     * sizeof(double));
    lambda = (double *) malloc ((m+n) * sizeof(double));

    /*---- CREATE A NEW KNITRO SOLVER INSTANCE. */
    kc = KTR_new();
    if (kc == NULL)
        {
        printf ("Failed to find a valid license.\n");
        return( -1 );
        }

    /*---- ILLUSTRATE HOW TO OVERRIDE DEFAULT OPTIONS BY READING FROM
     *---- THE knitro.opt FILE, AND THEN FIND THE CURRENT VALUE OF AN OPTION.
     *---- (SEE reverseCommExample.c FOR OTHER TECHNIQUES.)
     */
    if (KTR_load_param_file (kc, "knitro.opt") != 0)
        exit( -1 );
    if (KTR_get_int_param_by_name (kc, "hessopt", &nHessOpt) != 0)
        exit( -1 );

    /*---- SPECIFY THAT THE USER IS ABLE TO PROVIDE EVALUATIONS
     *---- OF THE HESSIAN MATRIX WITHOUT THE OBJECTIVE COMPONENT.
     *---- TURNED OFF BY DEFAULT BUT SHOULD BE ENABLED IF POSSIBLE. */
    if (KTR_set_int_param (kc, KTR_PARAM_HESSIAN_NO_F, KTR_HESSIAN_NO_F_ALLOW) != 0)
        return( -1 );

    /*---- REGISTER THE CALLBACK FUNCTIONS THAT PERFORM PROBLEM EVALUATION.
     */
    if (KTR_set_func_callback (kc, &callbackEvalFCorGA) != 0)
        exit( -1 );
    if (KTR_set_grad_callback (kc, &callbackEvalFCorGA) != 0)
        exit( -1 );
    if (KTR_set_hess_callback (kc, &callbackEvalHess) != 0)
        exit( -1 );

    /*---- SET THE NEW POINT CALLBACK */
    if (KTR_set_newpt_callback (kc, &callbackNewPoint) != 0)
        exit( -1 );

    /*---- COUNT HESSIAN CALLBACKS WITH userParams. */
    userParams.counter = 0;

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
    free (xInitial);
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
                         NULL, NULL, NULL, NULL, NULL, &userParams);

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
