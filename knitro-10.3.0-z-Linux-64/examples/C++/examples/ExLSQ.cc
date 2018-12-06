/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolverLSQ.h"

#include "ProblemLSQ.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a problem using exact gradient and and Hessian evaluations.
 * When using exact derivative evaluations, no KTR_GRADOPT_* or KTR_HESSOPT_* parameter is needed.
 */
int main() {
  // Create an LSQ problem instance.
  ProblemLSQ instance = ProblemLSQ();
  std::vector<int> types;
  types.assign((size_t)instance.getNumRes(), KTR_RESTYPE_GENERAL);
  instance.setResTypes(types);

  // Create an LSQ solver
  knitro::KTRSolverLSQ solver(&instance);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
