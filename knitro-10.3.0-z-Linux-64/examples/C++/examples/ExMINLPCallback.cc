/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemMINLP.h"
#include "ExampleMIPNodeCallback.h"
#include "ExampleHelpers.h"

/**
 * An example of solving a problem and using a KTRMipNodeCallback. 
 * The callback is passed to the problem instance before passing the problem to the solver.
 */
int main() {
  // Create a problem instance.
  ProblemMINLP instance;
  ExampleMipNodeCallback callback;
  instance.setMipNodeCallback(&callback);

  // Create a solver
  knitro::KTRSolver solver(&instance);
  solver.useMipNodeCallback();

  solver.setParam("mip_method", KTR_MIP_METHOD_BB);
  solver.setParam("algorithm", KTR_ALG_ACT_CG);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
