/*******************************************************/
/* Copyright (c) 2016 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  Knitro MINLP example driver using using callback mode, defining
 *  separate callback functions for each evaluation request.
 *
 *  This executable invokes Knitro to solve a simple nonlinear
 *  mixed integer optimization test problem.  The purpose is to
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
/*     FUNCTION callbackProcessNode                                 */
/*------------------------------------------------------------------*/
/** The signature of this function matches KTR_callback in knitro.h.
 */
int  callbackProcessNode (const int             evalRequestCode,
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
    KTR_context_ptr   kc;
    double dIncumbentBound;
    
    /*---- THE KNITRO CONTEXT POINTER WAS PASSED IN THROUGH "userParams". */
    kc = userParams;
    
    /*---- PRINT INFO ABOUT THE STATUS OF THE MIP SOLUTION. */
    printf ("callbackProcessNode: \n");
    printf ("    Node number    = %d\n", KTR_get_mip_num_nodes (kc));
    printf ("    Node objective = %e\n", *obj);
    printf ("    Current relaxation bound = %e\n",
            KTR_get_mip_relaxation_bnd (kc));
    dIncumbentBound = KTR_get_mip_incumbent_obj (kc);
    if (fabs(dIncumbentBound) >= KTR_INFBOUND)
        printf ("    No integer feasible point found yet.\n");
    else {
        printf ("    Current incumbent bound  = %e\n", dIncumbentBound);
        printf ("    Absolute integrality gap = %e\n",  KTR_get_mip_abs_gap (kc));
        printf ("    Relative integrality gap = %e\n",  KTR_get_mip_rel_gap (kc));        
    }

    /*---- USER DEFINED TERMINATION EXAMPLE. */
    /*---- UNCOMMENT BELOW TO FORCE TERMINATION AFTER 3 NODES. */
    //if (KTR_get_mip_num_nodes (kc) == 3)
    //    return KTR_RC_USER_TERMINATION;
    
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
    int          n, m, nnzJ, nnzH, objGoal, objType, objFnType;
    int          *xType, *cType, *cFnType;
    int          *jacIndexVars, *jacIndexCons, *hessRows, *hessCols;
    double       obj, *x, *lambda;
    double       *xLoBnds, *xUpBnds, *cLoBnds, *cUpBnds;


    /*---- FETCH THE SIZES OF THE PROBLEM TO BE SOLVED. */
    getProblemSizes (&n, &m, &nnzJ, &nnzH);

    /*---- ALLOCATE MEMORY FOR THE PROBLEM DEFINITION. */
    xType        = (int    *) malloc (n * sizeof(int));
    xLoBnds      = (double *) malloc (n * sizeof(double));
    xUpBnds      = (double *) malloc (n * sizeof(double));
    cType        = (int    *) malloc (m * sizeof(int));
    cFnType      = (int    *) malloc (m * sizeof(int));    
    cLoBnds      = (double *) malloc (m * sizeof(double));
    cUpBnds      = (double *) malloc (m * sizeof(double));
    jacIndexVars = (int    *) malloc (nnzJ * sizeof(int));
    jacIndexCons = (int    *) malloc (nnzJ * sizeof(int));
    hessRows     = (int    *) malloc (nnzH * sizeof(int));
    hessCols     = (int    *) malloc (nnzH * sizeof(int));

    /*---- FETCH THE DEFINITION OF THE PROBLEM TO BE SOLVED. */
    getProblemData (&objType, &objGoal, xLoBnds, xUpBnds, NULL,
                    cType, cLoBnds, cUpBnds,
                    jacIndexVars, jacIndexCons,
                    hessRows, hessCols,
                    &objFnType, xType, cFnType);

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

    /*---- ILLUSTRATE HOW TO OVERRIDE DEFAULT OPTIONS.
     *---- OPTIONS MUST BE SET BEFORE CALLING KTR_mip_init_problem.
     *---- (SEE callbackExample1.c FOR OTHER TECHNIQUES.)
     */
    if (KTR_set_int_param_by_name (kc, "mip_method", KTR_MIP_METHOD_BB) != 0)
        return( -1 );
    if (KTR_set_int_param_by_name (kc, "algorithm", KTR_ALG_ACT_CG) != 0)
        return( -1 );    
    if (KTR_set_int_param_by_name (kc, "outmode", KTR_OUTMODE_SCREEN) != 0)
        return( -1 );
    if (KTR_set_int_param (kc, KTR_PARAM_OUTLEV, KTR_OUTLEV_ALL) != 0)
        return( -1 );
    if (KTR_set_int_param (kc, KTR_PARAM_MIP_OUTINTERVAL, 1) != 0)
        return( -1 );
    if (KTR_set_int_param (kc, KTR_PARAM_MIP_MAXNODES, 10000) != 0)
        return( -1 );

    /*---- SPECIFY THAT THE USER IS ABLE TO PROVIDE EVALUATIONS
     *---- OF THE HESSIAN MATRIX WITHOUT THE OBJECTIVE COMPONENT.
     *---- TURNED OFF BY DEFAULT BUT SHOULD BE ENABLED IF POSSIBLE. */
    if (KTR_set_int_param (kc, KTR_PARAM_HESSIAN_NO_F, KTR_HESSIAN_NO_F_ALLOW) != 0)
        return( -1 );

    
    /*---- REGISTER THE CALLBACK FUNCTIONS THAT PERFORM PROBLEM EVALUATION.
     *---- THE HESSIAN CALLBACK ONLY NEEDS TO BE REGISTERED FOR SPECIFIC
     *---- HESSIAN OPTIONS (E.G., IT IS NOT REGISTERED IF THE OPTION FOR
     *---- BFGS HESSIAN APPROXIMATIONS IS SELECTED).
     */
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

    /*---- REGISTER THE CALLBACK FUNCTION THAT PERFORMS SOME TASK AFTER
     *---- EACH COMPLETION OF EACH NODE IN BRANCH-AND-BOUND TREE. */
    if (KTR_set_mip_node_callback (kc, &callbackProcessNode) != 0)
        exit( -1 );

    
    /*---- INITIALIZE KNITRO WITH THE PROBLEM DEFINITION. */
    nStatus = KTR_mip_init_problem (kc, n, objGoal, objType, objFnType,
                                    xType, xLoBnds, xUpBnds,
                                    m, cType, cFnType, cLoBnds, cUpBnds,
                                    nnzJ, jacIndexVars, jacIndexCons,
                                    nnzH, hessRows, hessCols, NULL, NULL);

    /*---- KNITRO KEEPS ITS OWN COPY OF THE PROBLEM DEFINITION,
     *---- SO THE LOCAL MEMORY CAN BE FREED IMMEDIATELY. */
    free (xType);
    free (xLoBnds);
    free (xUpBnds);
    free (cType);
    free (cFnType);
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
    nStatus = KTR_mip_solve (kc, x, lambda, 0, &obj,
                             NULL, NULL, NULL, NULL, NULL, kc);

    printf ("\n\n");
    if (nStatus != 0)
        printf ("Knitro failed to solve the problem, final status = %d\n",
                nStatus);
    else
    {
        /*---- AN EXAMPLE OF OBTAINING SOLUTION INFORMATION. */
        printf ("Knitro successful, integrality gap   = %e\n",
                KTR_get_mip_abs_gap (kc));
    }


    /*---- DELETE THE KNITRO SOLVER INSTANCE. */
    KTR_free (&kc);

    free (x);
    free (lambda);

    return( 0 );
}

/*----- End of source code -----------------------------------------*/
