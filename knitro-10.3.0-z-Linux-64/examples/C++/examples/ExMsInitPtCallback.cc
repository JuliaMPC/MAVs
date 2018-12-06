/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#include "KTRSolver.h"

#include "ProblemQCQP.h"
#include "ExampleMSInitPtCallback.h"
#include "ExampleHelpers.h"

/**
 * An example of solving a problem and using a KTRMSInitPtCallback. 
 * The callback is passed to the problem instance before passing the problem to the solver.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;

  int min = -10;
  int max = 10;
  ExampleMSInitPtCallback callback(min, max);
  instance.setMSInitPtCallback(&callback);

  // Create a solver
  knitro::KTRSolver solver(&instance);

  // Activate multistart and use of init point node callback
  solver.useMSInitptCallback();

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
