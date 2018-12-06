/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <iostream>
#include <iomanip>
#include "KTRISolver.h"

inline void printSolutionResults(knitro::KTRISolver & solver, int solveStatus) {
  if (solveStatus != 0) {
    std::cout << "Failed to solve problem, final status = " << solveStatus << std::endl;
	return;
  }
  std::cout << "---------- Solution found ----------" << std::endl << std::endl;

  std::cout.precision(2);
  std::cout << std::scientific;

  // Objective value
  std::cout << std::right << std::setw(28) << "Objective value = " << solver.getObjValue() << std::endl;
  
  // Solution point
  std::cout << std::right << std::setw(29) << "Solution point = (";
  const std::vector<double>& point = solver.getXValues();
  std::vector<double>::const_iterator it = point.begin();
  while ( it != point.end()) {
	  std::cout << *it;
	  if (++it != point.end())
		  std::cout << ", ";
  }
  std::cout << ")" << std::endl;

  if (!((solver.getProblem())->isMipProblem()))
  {
	  std::cout << std::right << std::setw(28) << "Feasibility violation = " << solver.getAbsFeasError() << std::endl;
	  std::cout << std::right << std::setw(28) << "KKT optimality violation = " << solver.getAbsOptError() << std::endl;
  }
  else {
	  std::cout << std::right << std::setw(28) << "Absolute integrality gap = " << solver.getMipAbsGap() << std::endl;
  }
  std::cout << std::endl;
}

