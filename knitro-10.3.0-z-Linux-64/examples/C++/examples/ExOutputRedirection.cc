/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemQCQP.h"
#include "ExampleOutputRedirection.h"
#include "ExampleHelpers.h"

/**
 * An example of solving a problem and using a KTRPutString output redirection callback. 
 * The callback is passed to the problem instance before passing the problem to the solver.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;
  ExampleOutputRedirection callback;

  instance.setPutStringFunction(&callback);

  // Create a solver
  knitro::KTRSolver solver(&instance);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
