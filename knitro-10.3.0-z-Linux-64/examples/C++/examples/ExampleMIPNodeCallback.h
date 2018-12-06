/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _EXAMPLEMIPNODECALLBACK_H_
//#define _EXAMPLEMIPNODECALLBACK_H_
#pragma once

#include <cmath>
#include <iostream>

#include "KTRSolver.h"
#include "KTRMipNodeCallback.h"

/*
 * An example implementation of KTRMipNodeCallback. Each time KNITRO processes a MIP node, this callback is called and prints information about the MIP solution up to this point.
 */
class ExampleMipNodeCallback : public knitro::KTRMipNodeCallback {
 public:
  int CallbackFunction(const std::vector<double>& x, const std::vector<double>& lambda, const double obj,
                       const std::vector<double>& c, knitro::KTRSolver* solver)
                       {

    /*---- PRINT INFO ABOUT THE STATUS OF THE MIP SOLUTION. */
    std::cout << "callbackProcessNode:" << std::endl;
    std::cout << "    Node number    = " << solver->getMipNumNodes() << std::endl;
    std::cout << "    Node objective = " << obj << std::endl;
    std::cout << "    Current relaxation bound = " << solver->getMipRelaxationBnd() << std::endl;

    double incumbentBound = solver->getMipIncumbentObj();

    if (std::fabs(incumbentBound) >= KTR_INFBOUND) {
      std::cout << "    No integer feasible point found yet." << std::endl;
    } else {
      std::cout << "    Current incumbent bound  = " << incumbentBound << std::endl;
      std::cout << "    Absolute integrality gap = " << solver->getMipAbsGap() << std::endl;
      std::cout << "    Relative integrality gap = " << solver->getMipRelGap() << std::endl;
    }

    return 0;
  }
};

//#endif
