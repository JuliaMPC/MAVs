/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#include "KTRSolver.h"

#include "ProblemQCQP.h"
#include "ExampleHelpers.h"

/**
 * An example of loading KNITRO tuner options and using the KNITRO tuner.
 * In order to use the KNITRO tuner, the KTR_PARAM_TUNER parameter must be set to value KTR_TUNER_ON.
 */
int main() {
  // Create a problem instance.
  ProblemQCQP instance;

  // Create a solver
  knitro::KTRSolver solver(&instance);

  solver.setParam(KTR_PARAM_TUNER, KTR_TUNER_ON);

  // Any non-default options set by the user will be respected
  // (i.e., remain fixed) by the KNITRO tuner.
  solver.loadParamFile("tuner-fixed.opt");

  // Use solver.loadTunerFile() to specify the options and option
  // values that should be tuned by the KNITRO tuner. The
  // KNITRO tuner will systematically test all combinations
  // of options specified by the file loaded here (while respecting
  // options in the regular way). If solver.loadTunerFile() is not
  // called, then the KNITRO tuner will automatically determine
  // which options to tune.
  solver.loadTunerFile("tuner-explore.opt");

  // Run tuner
  int solveStatus = solver.solve();

  return 0;
}
