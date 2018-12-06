/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemRosenbrockExtended.h"
#include "ExampleMSProcessCallback.h"
#include "ExampleHelpers.h"

/**
 * An example of solving a problem and using a KTRMSProcessCallback. 
 * The callback is passed to the problem instance before passing the problem to the solver.
 */
int main() {
  // Create a problem instance.
  ProblemRosenbrockExtended instance(5);

  ExampleMSProcessCallback callback;

  instance.setMSProcessCallback(&callback);

  // Create a solver
  knitro::KTRSolver solver(&instance, KTR_GRADOPT_FORWARD, KTR_HESSOPT_BFGS);

  solver.useMSProcessCallback();

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
