/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#include <vector> 

#include "KTRSolver.h"
#include "ProblemHS15.h"
#include "ExampleHelpers.h"

/**
 * An example of loading and solving a problem using exact gradient and and Hessian evaluations.
 * When using exact derivative evaluations, no KTR_GRADOPT_* or KTR_HESSOPT_* parameter is needed.
 */
int main() {
  // Create a problem instance.
  ProblemHS15 instance = ProblemHS15();

  // Create a solver
  knitro::KTRSolver solver(&instance);

  // Scale variables
  std::vector<double> xScales( instance.getNumVars() );
  for (int i = 0 ; i<instance.getNumVars() ; ++i)  
    xScales[i] = 0.01;
  std::vector<double> xCenters( instance.getNumVars() );
  for (int i = 0 ; i<instance.getNumVars() ; ++i)
    xCenters[i] = -0.3;
  solver.setVarScaling(xScales, xCenters);
  
  // Scale constraints
  std::vector<double> conScales( instance.getNumCons() );
  for (int i = 0 ; i<instance.getNumCons() ; ++i)
    conScales[i] = 2.;
  std::vector<double> compConScales( instance.getNumCompCons() );
  for (int i = 0 ; i<instance.getNumCompCons() ; ++i)  
    compConScales[i] = 1.;
  solver.setConScaling(conScales, compConScales);

  // Scale objective
  solver.setObjScaling(1.0);

  int solveStatus = solver.solve();

  printSolutionResults(solver, solveStatus);

  return 0;
}
