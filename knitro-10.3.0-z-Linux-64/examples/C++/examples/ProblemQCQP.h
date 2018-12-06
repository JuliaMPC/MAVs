/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _PROBLEMQCQP_H_
//#define _PROBLEMQCQP_H_
#pragma once
#include "KTRSolver.h"
#include "KTRProblem.h"

/**
 *  This file contains routines to implement KTRProblem for a small
 *  QCQP (quadratically constrained quadratic programming) test problem.
 *
 *  min   1000 - x0^2 - 2 x1^2 - x2^2 - x0 x1 - x0 x2
 *  s.t.  8 x0 + 14 x1 + 7 x2 - 56 = 0
 *        x0^2 + x1^2 + x2^2 - 25 >= 0
 *        x0 >= 0, x1 >= 0, x2 >= 0
 *
 *  The start point (2, 2, 2) converges to the minimum at (0, 0, 8),
 *  with final objective = 936.0.  From a different start point,
 *  KNITRO may converge to an alternate local solution at (7, 0, 0),
 *  with objective = 951.0.
 */
class ProblemQCQP : public knitro::KTRProblem {

 public:
  ProblemQCQP()
      : KTRProblem(3, 2, 6, 5) {
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    setDerivativeProperties();
  }

  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) {

    // Linear equality constraint.
    c[0] = 8.0e0 * x[0] + 14.0e0 * x[1] + 7.0e0 * x[2] - 56.0e0;

    // Quadratic inequality constraint.
    c[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - 25.0e0;

    return 1.0e3 - x[0] * x[0] - 2.0e0 * x[1] * x[1] - x[2] * x[2] - x[0] * x[1] - x[0] * x[2];
  }

  int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac) {
    objGrad[0] = -2.0e0 * x[0] - x[1] - x[2];
    objGrad[1] = -4.0e0 * x[1] - x[0];
    objGrad[2] = -2.0e0 * x[2] - x[0];

    // Gradient of the first constraint, c[0].
    jac[0] = 8.0e0;
    jac[1] = 14.0e0;
    jac[2] = 7.0e0;

    // Gradient of the second constraint, c[1].
    jac[3] = 2.0e0 * x[0];
    jac[4] = 2.0e0 * x[1];
    jac[5] = 2.0e0 * x[2];

    return 0;
  }

  int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                   std::vector<double>& hess) {

    hess[0] = -2.0e0 * objScaler + 2.0e0 * lambda[1];
    hess[1] = -1.0e0 * objScaler;
    hess[2] = -1.0e0 * objScaler;
    hess[3] = -4.0e0 * objScaler + 2.0e0 * lambda[1];
    hess[4] = -2.0e0 * objScaler + 2.0e0 * lambda[1];
    return 0;
  }

 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjQuadratic);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    setVarLoBnds(0, 0.0);
    setVarLoBnds(1, 0.0);
    setVarLoBnds(2, 0.0);

    setVarUpBnds(0, KTR_INFBOUND);
    setVarUpBnds(1, KTR_INFBOUND);
    setVarUpBnds(2, KTR_INFBOUND);
    
    setXInitial(0, 2.0);
    setXInitial(1, 2.0);
    setXInitial(2, 2.0);
  }

  void setConstraintProperties() {
    setConTypes(0, knitro::KTREnums::ConstraintType::ConLinear);
    setConTypes(1, knitro::KTREnums::ConstraintType::ConQuadratic);

    setConLoBnds(0, 0.0);
    setConLoBnds(1, 0.0);

    setConUpBnds(0, 0.0);
    setConUpBnds(1, KTR_INFBOUND);
  }

  void setDerivativeProperties() {
    setJacIndexCons(0, 0);
    setJacIndexCons(1, 0);
    setJacIndexCons(2, 0);
    setJacIndexCons(3, 1);
    setJacIndexCons(4, 1);
    setJacIndexCons(5, 1);

    setJacIndexVars(0, 0);
    setJacIndexVars(1, 1);
    setJacIndexVars(2, 2);
    setJacIndexVars(3, 0);
    setJacIndexVars(4, 1);
    setJacIndexVars(5, 2);

    // Provide second derivative structural information;
    // Only the nonzeros of the upper triangle of the Hessian.

    setHessIndexRows(0, 0);
    setHessIndexRows(1, 0);
    setHessIndexRows(2, 0);
    setHessIndexRows(3, 1);
    setHessIndexRows(4, 2);

    setHessIndexCols(0, 0);
    setHessIndexCols(1, 1);
    setHessIndexCols(2, 2);
    setHessIndexCols(3, 1);
    setHessIndexCols(4, 2);
  }
};

//#endif
