/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _PROBLEMROSENBROCK_H_
//#define _PROBLEMROSENBROCK_H_
#pragma once

#include <vector>
#include "KTRSolver.h"
#include "KTRProblem.h"

/**
 *  This file contains routines to implement KTRProblem for
 *  the Rosenbrock function with a = 1 and b = 100.
 *
 *  This problem definition is an example of evaluating the objective
 *  function and gradient simultaneously.
 *  In this example, function to evaluate both is defined in the EvalFC
 *  function, with the EvalGA function returning zero and not modifying
 *  the objGrad vector.
 * 
 *  The function is defined as 
 *  min (a - x)^2 + b(y-x^2)^2
 */
class ProblemRosenbrock : public knitro::KTRProblem {

 public:
  ProblemRosenbrock()
      : KTRProblem(2, 0, 0, 3) {
    setObjectiveProperties();
    setVariableProperties();
    setDerivativeProperties();
  }

  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) {

    double z = 1 - x[0];
    double w = x[1] - x[0] * x[0];

    // Evaluate objective function and constraint simultaneously.

    objGrad[0] = -2 * z - 400 * x[0] * w;
    objGrad[1] = 200 * w;

    return z * z + 100 * w * w;
  }

  int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac) {
    // When evaluating gradient and jacobian in evaluateFC, make sure to implement
    // evaluateGA callback and return KTR_RC_EVALFCGA.
    // Also, note that the values of gradient and jacobian computed in evaluateFC will not be copied
    // into parameters objGrad and jac from this method.
    return KTR_RC_EVALFCGA;
  }

  int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                   std::vector<double>& hess) {

    hess[0] = (2 - 400 * x[1] + 1200 * x[0] * x[0]) * objScaler;
    hess[1] = -400 * x[0] * objScaler;
    hess[2] = 200 * objScaler;
    return 0;
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

  void setDerivativeProperties() {
    std::vector<int> hessCols(this->getNNZH());
    std::vector<int> hessRows(this->getNNZH());

    hessRows[0] = 0;
    hessCols[0] = 0;
    hessRows[1] = 0;
    hessCols[1] = 1;
    hessRows[2] = 1;
    hessCols[2] = 1;

    this->setHessIndexCols(hessCols);
    this->setHessIndexRows(hessRows);
  }
};

//#endif
