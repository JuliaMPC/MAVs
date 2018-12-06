/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _PROBLEMMINLP_H_
//#define _PROBLEMMINLP_H_
#pragma once

#include <cmath>

#include "KTRSolver.h"
#include "KTRProblem.h"

/**
 *  This file contains routines to implement KTRProblem for
 *  test problem 1 (Synthesis of processing system) in
 *  M. Duran & I.E. Grossmann,  "An outer approximation algorithm for
 *  a class of mixed integer nonlinear programs", Mathematical
 *  Programming 36, pp. 307-339, 1986.  The problem also appears as
 *  problem synthes1 in the MacMINLP test set.
 *
 *
 *  min   5 x4 + 6 x5 + 8 x6 + 10 x1 - 7 x3 -18 log(x2 + 1)
 *       - 19.2 log(x1 - x2 + 1) + 10
 *  s.t.  0.8 log(x2 + 1) + 0.96 log(x1 - x2 + 1) - 0.8 x3 >= 0
 *        log(x2 + 1) + 1.2 log(x1 - x2 + 1) - x3 - 2 x6 >= -2
 *        x2 - x1 <= 0
 *        x2 - 2 x4 <= 0
 *        x1 - x2 - 2 x5 <= 0
 *        x4 + x5 <= 1
 *        0 <= x1 <= 2
 *        0 <= x2 <= 2
 *        0 <= x3 <= 1
 *        x1, x2, x3 continuous
 *        x4, x5, x6 binary
 *
 *
 *  The solution is (1.30098, 0, 1, 0, 1, 0).
 */
class ProblemMINLP : public knitro::KTRProblem {

 public:
  ProblemMINLP()
      : KTRProblem(6, 6, 16, 3) {
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    setDerivativeProperties();
  }

  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) {
    double obj;
    double tmp1;
    double tmp2;

    tmp1 = x[0] - x[1] + 1.0;
    tmp2 = x[1] + 1.0;
    obj = 5.0 * x[3] + 6.0 * x[4] + 8.0 * x[5] + 10.0 * x[0] - 7.0 * x[2] - 18.0 * std::log(tmp2)
        - 19.2 * std::log(tmp1) + 10.0;
    c[0] = 0.8 * std::log(tmp2) + 0.96 * std::log(tmp1) - 0.8 * x[2];
    c[1] = std::log(tmp2) + 1.2 * std::log(tmp1) - x[2] - 2 * x[5];
    c[2] = x[1] - x[0];
    c[3] = x[1] - 2 * x[3];
    c[4] = x[0] - x[1] - 2 * x[4];
    c[5] = x[3] + x[4];

    return obj;
  }

  int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac) {
    double tmp1;
    double tmp2;

    tmp1 = x[0] - x[1] + 1.0;
    tmp2 = x[1] + 1.0;
    objGrad[0] = 10.0 - (19.2 / tmp1);
    objGrad[1] = (-18.0 / tmp2) + (19.2 / tmp1);
    objGrad[2] = -7.0;
    objGrad[3] = 5.0;
    objGrad[4] = 6.0;
    objGrad[5] = 8.0;

    // Gradient of constraint 0.
    jac[0] = 0.96 / tmp1;
    jac[1] = (-0.96 / tmp1) + (0.8 / tmp2);
    jac[2] = -0.8;

    // Gradient of constraint 1.
    jac[3] = 1.2 / tmp1;
    jac[4] = (-1.2 / tmp1) + (1.0 / tmp2);
    jac[5] = -1.0;
    jac[6] = -2.0;

    // Gradient of constraint 2.
    jac[7] = -1.0;
    jac[8] = 1.0;

    // Gradient of constraint 3.
    jac[9] = 1.0;
    jac[10] = -2.0;

    // Gradient of constraint 4.
    jac[11] = 1.0;
    jac[12] = -1.0;
    jac[13] = -2.0;

    // Gradient of constraint 5.
    jac[14] = 1.0;
    jac[15] = 1.0;

    return 0;
  }

  int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                   std::vector<double>& hess) {

    double tmp1;
    double tmp2;

    tmp1 = x[0] - x[1] + 1.0;
    tmp2 = x[1] + 1.0;
    hess[0] = objScaler * (19.2 / (tmp1 * tmp1)) + lambda[0] * (-0.96 / (tmp1 * tmp1))
        + lambda[1] * (-1.2 / (tmp1 * tmp1));
    hess[1] = objScaler * (-19.2 / (tmp1 * tmp1)) + lambda[0] * (0.96 / (tmp1 * tmp1))
        + lambda[1] * (1.2 / (tmp1 * tmp1));
    hess[2] = objScaler * ((19.2 / (tmp1 * tmp1)) + (18.0 / (tmp2 * tmp2)))
        + lambda[0] * ((-0.96 / (tmp1 * tmp1)) - (0.8 / (tmp2 * tmp2)))
        + lambda[1] * ((-1.2 / (tmp1 * tmp1)) - (1.0 / (tmp2 * tmp2)));

    return 0;
  }

  int evaluateHessianVector(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                            std::vector<double>& vector) {
    double tmpVec[2];

    double tmp1 = x[0] - x[1] + 1.0;
    double tmp2 = x[1] + 1.0;

    // H[0,0]*v[0] + H[0,1]*v[1].
    tmpVec[0] = (objScaler * (19.2 / (tmp1 * tmp1)) + lambda[0] * (-0.96 / (tmp1 * tmp1))
        + lambda[1] * (-1.2 / (tmp1 * tmp1))) * vector[0]
        + (objScaler * (-19.2 / (tmp1 * tmp1)) + lambda[0] * (0.96 / (tmp1 * tmp1)) + lambda[1] * (1.2 / (tmp1 * tmp1)))
            * vector[1];

    // H[1,0]*v[0] + H[1,1]*v[1].
    tmpVec[1] = (objScaler * (-19.2 / (tmp1 * tmp1)) + lambda[0] * (0.96 / (tmp1 * tmp1))
        + lambda[1] * (1.2 / (tmp1 * tmp1))) * vector[0]
        + (objScaler * ((19.2 / (tmp1 * tmp1)) + (18.0 / (tmp2 * tmp2)))
            + lambda[0] * ((-0.96 / (tmp1 * tmp1)) - (0.8 / (tmp2 * tmp2)))
            + lambda[1] * ((-1.2 / (tmp1 * tmp1)) - (1.0 / (tmp2 * tmp2)))) * vector[1];

    vector[0] = tmpVec[0];
    vector[1] = tmpVec[1];
    vector[2] = 0.0;
    vector[3] = 0.0;
    vector[4] = 0.0;
    vector[5] = 0.0;
    return 0;
  }

 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjGeneral);
    setObjFnType(knitro::KTREnums::FunctionType::Convex);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    setVarLoBnds(0.0);
    setVarUpBnds(0, 2.0);
    setVarUpBnds(1, 2.0);
    setVarUpBnds(2, 1.0);
    setVarUpBnds(3, 1.0);
    setVarUpBnds(4, 1.0);
    setVarUpBnds(5, 1.0);

    setVarTypes(0, knitro::KTREnums::VariableType::Continuous);
    setVarTypes(1, knitro::KTREnums::VariableType::Continuous);
    setVarTypes(2, knitro::KTREnums::VariableType::Continuous);
    setVarTypes(3, knitro::KTREnums::VariableType::Binary);
    setVarTypes(4, knitro::KTREnums::VariableType::Binary);
    setVarTypes(5, knitro::KTREnums::VariableType::Binary);
  }

  void setConstraintProperties() {
    setConTypes(0, knitro::KTREnums::ConstraintType::ConGeneral);
    setConTypes(1, knitro::KTREnums::ConstraintType::ConGeneral);
    setConTypes(2, knitro::KTREnums::ConstraintType::ConLinear);
    setConTypes(3, knitro::KTREnums::ConstraintType::ConLinear);
    setConTypes(4, knitro::KTREnums::ConstraintType::ConLinear);
    setConTypes(5, knitro::KTREnums::ConstraintType::ConLinear);

    setConLoBnds(0, 0.0);
    setConLoBnds(1, -2.0);
    setConLoBnds(2, -KTR_INFBOUND);
    setConLoBnds(3, -KTR_INFBOUND);
    setConLoBnds(4, -KTR_INFBOUND);
    setConLoBnds(5, -KTR_INFBOUND);

    setConUpBnds(0, KTR_INFBOUND);
    setConUpBnds(1, KTR_INFBOUND);
    setConUpBnds(2, 0.0);
    setConUpBnds(3, 0.0);
    setConUpBnds(4, 0.0);
    setConUpBnds(5, 1.0);

    setConFnTypes(0, knitro::KTREnums::FunctionType::Convex);
    setConFnTypes(1, knitro::KTREnums::FunctionType::Convex);
    setConFnTypes(2, knitro::KTREnums::FunctionType::Convex);
    setConFnTypes(3, knitro::KTREnums::FunctionType::Convex);
    setConFnTypes(4, knitro::KTREnums::FunctionType::Convex);
    setConFnTypes(5, knitro::KTREnums::FunctionType::Convex);
  }

  void setDerivativeProperties() {
    setJacIndexCons(0, 0);
    setJacIndexCons(1, 0);
    setJacIndexCons(2, 0);

    setJacIndexCons(3, 1);
    setJacIndexCons(4, 1);
    setJacIndexCons(5, 1);
    setJacIndexCons(6, 1);

    setJacIndexCons(7, 2);
    setJacIndexCons(8, 2);

    setJacIndexCons(9, 3);
    setJacIndexCons(10, 3);

    setJacIndexCons(11, 4);
    setJacIndexCons(12, 4);
    setJacIndexCons(13, 4);

    setJacIndexCons(14, 5);
    setJacIndexCons(15, 5);

    setJacIndexVars(0, 0);
    setJacIndexVars(1, 1);
    setJacIndexVars(2, 2);

    setJacIndexVars(3, 0);
    setJacIndexVars(4, 1);
    setJacIndexVars(5, 2);

    setJacIndexVars(6, 5);
    setJacIndexVars(7, 0);
    setJacIndexVars(8, 1);
    setJacIndexVars(9, 1);

    setJacIndexVars(10, 3);
    setJacIndexVars(11, 0);
    setJacIndexVars(12, 1);
    setJacIndexVars(13, 4);
    setJacIndexVars(14, 3);
    setJacIndexVars(15, 4);

    // Provide second derivative structural information;
    // Only the nonzeros of the upper triangle of the Hessian.

    setHessIndexRows(0, 0);
    setHessIndexRows(1, 0);
    setHessIndexRows(2, 1);

    setHessIndexCols(0, 0);
    setHessIndexCols(1, 1);
    setHessIndexCols(2, 1);
  }
};

//#endif
