/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _PROBLEMMPEC_H_
//#define _PROBLEMMPEC_H_
#pragma once

#include "KTRSolver.h"
#include "KTRProblem.h"

/**
 *  This file contains routines to implement KTRProblem for an
 *  example MPEC problem, as described in the KNITRO user manual 
 *  section on complementarity constraints.
 *  
 *  This is an example of using complementarity constraints and 
 *  adding them to the problem with the setComplementarity() function.
 * 
 *  The problem is defined below. Note that constraints that are
 *  complementary to each other are defined as auxiliary variables
 *  as KNITRO can only define complementarity between variables
 *  and not constraints.
 * 
 *  min (x0 - 5)^2 + (2x1 + 1)^2
 *        
 *  s.t.
 *
 *  2(x1 - 1) - 1.5x0 + x2 - 0.5x3 + x4 = 0
 *  3x0 - x1 - 3 - x5 = 0
 *  -x0 + 0.5x1 + 4 - x6 = 0
 *  -x0 - x1 + 7 - x7 = 0
 *  0 <= x5 complements x2 >= 0
 *  0 <= x6 complements x3 >= 0
 *  0 <= x7 complements x4 >= 0
 *
 *  xi >= 0, i = 0 to 7
 */
class ProblemMPEC : public knitro::KTRProblem {

 public:
  ProblemMPEC()
      : KTRProblem(8, 4) {
    setObjectiveProperties();
    setVariableProperties();
    setConstraintProperties();
    setComplementarityProperties();
  }

  // Objective function.
  double evaluateFC(const std::vector<double>& x, std::vector<double>& c, std::vector<double>& objGrad,
                    std::vector<double>& jac) {

    c[0] = 2 * (x[1] - 1) - 1.5 * x[0] + x[2] - 0.5 * x[3] + x[4];
    c[1] = 3 * x[0] - x[1] - 3 - x[5];
    c[2] = -x[0] + 0.5 * x[1] + 4 - x[6];
    c[3] = -x[0] - x[1] + 7 - x[7];

    return (x[0] - 5) * (x[0] - 5) + (2 * x[1] + 1) * (2 * x[1] + 1);
  }

 private:
  void setObjectiveProperties() {
    setObjType(knitro::KTREnums::ObjectiveType::ObjGeneral);
    setObjGoal(knitro::KTREnums::ObjectiveGoal::Minimize);
  }

  void setVariableProperties() {
    setVarLoBnds(0.0);
    setVarUpBnds(KTR_INFBOUND);
  }

  void setConstraintProperties() {
    setConTypes(knitro::KTREnums::ConstraintType::ConLinear);
    setConLoBnds(0.0);
    setConUpBnds(0.0);
  }

  void setComplementarityProperties() {
    std::vector<int> indexList1;
    std::vector<int> indexList2;
    indexList1.push_back(2);
    indexList2.push_back(5);
    indexList1.push_back(3);
    indexList2.push_back(6);
    indexList1.push_back(4);
    indexList2.push_back(7);

    setComplementarity(indexList1, indexList2);
  }
};

//#endif
