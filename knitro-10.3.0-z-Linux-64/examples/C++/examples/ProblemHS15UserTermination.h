/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/

#pragma once

#include <atomic>
#include "KTRSolver.h"
#include "KTRProblem.h"
#include "ProblemHS15.h"

/**
 *  This file contains routines to implement KTRProblem for
 *  test problem HS15 from the Hock & Schittkowski collection with a maximum number of evaluations.
 *  This shows how to use KTR_RC_USER_TERMINATION callback return code.
 */
class ProblemHS15UserTermination : public ProblemHS15 {

 private:
  std::atomic_int evaluations_counter;
  int max_evals;

 public:
  ProblemHS15UserTermination(int max_evaluations)
      : ProblemHS15() {
    evaluations_counter = 0;
    max_evals = max_evaluations;
  }

  int evaluateFC(const std::vector<double>& x, double& obj, std::vector<double>& c,
                    std::vector<double>& objGrad, std::vector<double>& jac) {
    // Locking a mutex to check number of function evaluations
    if (evaluations_counter >= max_evals) {
      return KTR_RC_USER_TERMINATION;
    } else {
      ++evaluations_counter;
    }

    obj = ProblemHS15::evaluateFC(x, c, objGrad, jac);
    return 0;
  }
};
