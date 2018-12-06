/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _PROBLEMROSENBROCK_EXTENDED_H_
//#define _PROBLEMROSENBROCK_EXTENDED_H_
#pragma once
#include "KTRProblem.h"

/**
 *  This file contains routines to implement KTRProblem for
 *  a multidemnsional Rosenbrock function.
 *
 *  The function is defined as 
 *  min sum(i = 1 to n-1) [(1 - x[i])^2 + 100(x[i+1]-x[i]^2)^2]
 */
class ProblemRosenbrockExtended : public knitro::KTRProblem {

 public:
  explicit ProblemRosenbrockExtended(int nVars)
      : KTRProblem(nVars, 0) {
    setObjectiveProperties();
    setVariableProperties();
  }

  // Objective function.
  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) {
    double z = 1 - x[0];
    double w = x[1] - x[0] * x[0];

    double obj = 0;
    for (int i = 0; i < getNumVars() - 1; i++) {
      obj += (1 - x[i]) * (1 - x[i]) + 100 * (x[i + 1] - x[i] * x[i]) * (x[i + 1] - x[i] * x[i]);
    }

    return obj;
  }

 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjGeneral);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    setVarLoBnds(-KTR_INFBOUND);
    setVarUpBnds(KTR_INFBOUND);
  }

};

//#endif
