/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemQCQP.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a problem and using finite difference gradients and BFGS Hessian evaluation.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;

  // Create a solver
  knitro::KTRSolver solver(&instance, KTR_GRADOPT_FORWARD, KTR_HESSOPT_BFGS);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
