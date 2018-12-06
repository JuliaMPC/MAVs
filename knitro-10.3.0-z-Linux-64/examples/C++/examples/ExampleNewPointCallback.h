/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _EXAMPLENEWPOINTCALLBACK_H_
//#define _EXAMPLENEWPOINTCALLBACK_H_
#pragma once

#include <iostream>
#include "KTRNewptCallback.h"
#include "KTRISolver.h"

/**
 * An example implementation of KTRnewptCallback. Each time KNITRO computes a new (possibly infeasible) estimate of the solution, this callback is called and prints the primal variable values, the total number of function and costraint evaluations completed, and the current abbsolute feasibility error.
 */
class ExampleNewPointCallback : public knitro::KTRNewptCallback {
 public:

  int CallbackFunction(const std::vector<double>& x, const std::vector<double>& lambda, double obj,
                       const std::vector<double>& c, const std::vector<double>& objGrad, const std::vector<double>& jac,
                       knitro::KTRISolver * solver)
                       {
    int n = static_cast<int>(x.size());
    std::cout << ">> New point computed by KNITRO: (";
    for (int i = 0; i < n - 1; i++) {
      std::cout << x[i] << ", ";
    }

    std::cout << x[n - 1] << std::endl;

    std::cout << "Number FC evals= " << (dynamic_cast<knitro::KTRSolver*>(solver))->getNumberFCEvals() << std::endl;
    std::cout << "Current feasError= " << (dynamic_cast<knitro::KTRSolver*>(solver))->getAbsFeasError() << std::endl;

    return 0;
  }
};

//#endif
