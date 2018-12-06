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
 *  This file contains routines to implement KTRProblem for
 *  test problem HS15 from the Hock & Schittkowski collection.
 *
 *  min   100 (x2 - x1^2)^2 + (1 - x1)^2
 *  s.t.  x1 x2 >= 1
 *        x1 + x2^2 >= 0
 *        x1 <= 0.5
 *
 *  The standard start point (-2, 1) usually converges to the standard
 *  minimum at (0.5, 2.0), with final objective = 306.5.
 *  Sometimes the solver converges to another local minimum
 *  at (-0.79212, -1.26243), with final objective = 360.4.
 */
class ProblemHS15 : public knitro::KTRProblem {

 public:
  ProblemHS15()
      : KTRProblem(2, 2, 4, 3) {
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    setDerivativeProperties();
  }

  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) {
                      
    double  tmp = x[1] - x[0]*x[0];
    double obj = 100.0 * (tmp*tmp) + ((1.0 - x[0])*(1.0 - x[0]));
    c[0] = x[0] * x[1];
    c[1] = x[0] + (x[1]*x[1]);

    return obj;
  }

  int evaluateGA(const std::vector<double>& x, std::vector<double>& objGrad, std::vector<double>& jac) {
    double tmp = x[1] - x[0]*x[0];
    objGrad[0] = (-400.0 * tmp * x[0]) - (2.0 * (1.0 - x[0]));
    objGrad[1] = 200.0 * tmp;

    jac[0] = x[1];
    jac[1] = x[0];
    jac[2] = 1.0;
    jac[3] = 2.0 * x[1];

    return 0;
  }

  int evaluateHess(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                   std::vector<double>& hess) {
    hess[0] = objScaler * ( (-400.0 * x[1]) + (1200.0 * x[0]*x[0]) + 2.0);
    hess[1] = (objScaler * (-400.0 * x[0])) + lambda[0];
    hess[2] = (objScaler * 200.0) + (lambda[1] * 2.0);
    return 0;
  }
  
  int evaluateHessianVector(const std::vector<double>& x, double objScaler, const std::vector<double>& lambda,
                   std::vector<double>& vector) {
    double tmp[2];

    /*---- H[0,0]*v[0] + H[0,1]*v[1]. */
    tmp[0] =   (objScaler*(((-400.0 * x[1]) + (1200.0 * x[0]*x[0]) + 2.0))) * vector[0]
                  + (objScaler*(-400.0 * x[0]) + lambda[0])                      * vector[1];

    /*---- H[1,0]*v[0] + H[1,1]*v[1]. */
    tmp[1] =   (objScaler*(-400.0 * x[0]) + lambda[0]) * vector[0]
                  + (objScaler*200.0 + (lambda[1] * 2.0))   * vector[1];

    vector[0] = tmp[0];
    vector[1] = tmp[1];
    return 0;
  }
  
 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjGeneral);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    setVarLoBnds(0, -KTR_INFBOUND);
    setVarLoBnds(1, -KTR_INFBOUND);

    setVarUpBnds(0, 0.5);
    setVarUpBnds(1, KTR_INFBOUND);
    
    setXInitial(0, -2.0);
    setXInitial(1, 1.0);
  }

  void setConstraintProperties() {
    setConTypes(0, knitro::KTREnums::ConstraintType::ConQuadratic);
    setConTypes(1, knitro::KTREnums::ConstraintType::ConQuadratic);

    setConLoBnds(0, 1.0);
    setConLoBnds(1, 0.0);

    setConUpBnds(0, KTR_INFBOUND);
    setConUpBnds(1, KTR_INFBOUND);
  }

  void setDerivativeProperties() {
    setJacIndexCons(0, 0);
    setJacIndexCons(1, 0);
    setJacIndexCons(2, 1);
    setJacIndexCons(3, 1);

    setJacIndexVars(0, 0);
    setJacIndexVars(1, 1);
    setJacIndexVars(2, 0);
    setJacIndexVars(3, 1);

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
