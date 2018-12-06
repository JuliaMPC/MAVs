/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemMINLP.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a MINLP problem. 
 * Sets MIP parameters using parameter string names to choose the solution algorithm.
 */
int main() {
  // Create a problem instance.
  ProblemMINLP instance;

  // Create a solver
  knitro::KTRSolver solver(&instance);

  solver.setParam("mip_method", KTR_MIP_METHOD_MISQP);
  solver.setParam("algorithm", KTR_ALG_ACT_CG);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
