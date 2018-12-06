/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemHS15.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a problem using exact gradient and
 * user callback for hessian-vector product.
 */
int main() {
  // Create a problem instance.
  ProblemHS15 instance = ProblemHS15();

  // Create a solver
  knitro::KTRSolver solver(&instance, KTR_GRADOPT_EXACT, KTR_HESSOPT_PRODUCT);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
