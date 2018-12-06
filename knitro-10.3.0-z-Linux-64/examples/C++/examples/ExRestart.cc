/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#include <iostream>
#include "KTRSolver.h"
#include "ProblemQCQP.h"

/**
 * An example of solving a problem multiple times with different KNITRO parameters.
 * Calls solver.restart() and sets parameters between successive solves.
 * Empty vectors are passed to restart so that KNITRO will choose initial variable values.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;

  // Create a solver
  knitro::KTRSolver solver(&instance);
  int solveStatus;
  solver.solve();

  for (int i = 0; i < 7; i++) {
    solver.restart(std::vector<double>(), std::vector<double>());
    solver.setParam("bar_murule", i);
    std::cout << "bar_murule= " << i;
    solveStatus = solver.solve();

    if (solveStatus != 0) {
      std::cout << "solver did not find an optimal solution." << std::endl;
      std::cout << "status = " << solveStatus << std::endl;
    } else {
      std::cout << "solved with " << solver.getNumberIters() << " major iterations, ";
      std::cout << solver.getNumberFCEvals() << " FC evaluations" << std::endl;
    }
  }

  return 0;
}
