/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemQCQP.h"
#include "ExampleHelpers.h"

/**
 * An example of solving the problem, changing variable bounds, and resolving.
 * The variable bounds are reset by changing the bounds in the KTRIProblem object.
 * Upper bounds for all variables are set to 7, making the original optimal solution infeasible.
 * Although any of the KTRIProblem object properties can be modified between calling solve(),
 * only variable bounds and KNITRO parameters (not including gradient and Hessian evaluation types)
 * will be reflected in future calls to solve.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;

  // Create a solver
  knitro::KTRSolver solver(&instance);

  int solveStatus = solver.solve();
  printSolutionResults(solver, solveStatus);

  // changing upper bounds makes previous optimal solution infeasible
  instance.setVarUpBnds(7.0);

  solveStatus = solver.solve();
  printSolutionResults(solver, solveStatus);

  return 0;
}
