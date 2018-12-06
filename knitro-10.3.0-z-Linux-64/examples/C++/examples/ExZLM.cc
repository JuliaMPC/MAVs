/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#include "KTRSolver.h"
#include "zlm.h"
#include "ProblemQCQP.h"
#include "ExampleHelpers.h"
#include "ExampleOutputRedirection.h"

/**
 * An example of using the Ziena License Manager to use KNITRO with a network license.
 * If no network license is found, KNITRO will look for a local license.
 * If neither is found, the ZLM object will throw  a KNITRO exception.
 */
int main() {
  knitro::ZLM zlm;

  // Create a problem instance.
  ProblemQCQP instance = ProblemQCQP();

  // Create a solver
  knitro::KTRSolver solver(&zlm, &instance);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
