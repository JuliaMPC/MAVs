/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemQCQP.h"
#include "ExampleNewPointCallback.h"
#include "ExampleHelpers.h"

/**
 * An example of solving a problem and using a KTRNewptCallback. 
 * The callback is passed to the problem instance before passing the problem to the solver.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;

  ExampleNewPointCallback callback;

  instance.setNewPointCallback(&callback);

  // Create a solver
  knitro::KTRSolver solver(&instance, KTR_GRADOPT_FORWARD, KTR_HESSOPT_BFGS);
  solver.useNewptCallback();

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
