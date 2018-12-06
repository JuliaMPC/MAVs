/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemMPEC.h"
#include "ExampleHelpers.h"

/**
 * An example of solving a problem with complementarity constraints.
 */
int main() {
  // Create a problem instance.
  ProblemMPEC instance;

  // Create a solver
  knitro::KTRSolver solver(&instance, KTR_GRADOPT_FORWARD, KTR_HESSOPT_BFGS);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
