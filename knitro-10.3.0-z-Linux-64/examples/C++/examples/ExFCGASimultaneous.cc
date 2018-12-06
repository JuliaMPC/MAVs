/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemRosenbrock.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a problem that evaluates objective function and gradients simultaneously.
 */
int main() {
  // Create a problem instance.
  ProblemRosenbrock instance;

  // Create a solver
  knitro::KTRSolver solver(&instance);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
