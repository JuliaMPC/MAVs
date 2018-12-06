/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemHS15.h"
#include "ProblemHS15UserTermination.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a problem using exact gradient and and Hessian evaluations.
 * When using exact derivative evaluations, no KTR_GRADOPT_* or KTR_HESSOPT_* parameter is needed.
 */
int main() {
    // Maximum number of calls to evalf fc authorized
    const int MAX_FC_CALLS = 10;

    // ----- Early termination using knitro options ----- //
    ProblemHS15* instance = new ProblemHS15();
    knitro::KTRSolver *solver = new knitro::KTRSolver(instance);

    solver->setParam(KTR_PARAM_MAXFEVALS, MAX_FC_CALLS); // evaluations limit
    int solveStatus = solver->solve();

    delete solver;
    delete instance;
    // 10 function evaluations performed //


    // ----- Early termination using user custom routine ----- //
    instance = new ProblemHS15UserTermination(MAX_FC_CALLS);
    solver = new knitro::KTRSolver(instance);

    solveStatus = solver->solve();

    delete solver;
    delete instance;
    // 11 function evaluation performed (11th returned USER_TERMINATION signal) //


    // ----- Early termination for each solve, using multistart ----- //
    instance = new ProblemHS15();
    solver = new knitro::KTRSolver(instance);

    solver->setParam(KTR_PARAM_MULTISTART, KTR_MULTISTART_YES); // Enable multistart
    solver->setParam(KTR_PARAM_MSMAXSOLVES, 5); // perform 5 runs
    solver->setParam(KTR_PARAM_MAXFEVALS, MAX_FC_CALLS); // function evaluation limit (per run)

    solveStatus = solver->solve();

    delete solver;
    delete instance;
    // 55 function evaluation performed: 5 multistart * (10 evaluations for each solve + 1 for each initial point)

    return 0;
}
