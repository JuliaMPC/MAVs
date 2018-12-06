/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _EXAMPLEMSPROCESSCALLBACK_H_
//#define _EXAMPLEMSPROCESSCALLBACK_H_
#pragma once

#include "KTRMSProcessCallback.h"
#include <iostream>

/**
 * An example implementation of KTRMSProcessCallback. After each multistart solve (each solve with a single starting point), this callback is called and prints the objective function value and the final primal variable values from the solution.
 */
class ExampleMSProcessCallback : public knitro::KTRMSProcessCallback {
 public:

  int CallbackFunction(const std::vector<double>& x, const std::vector<double>& lambda, const double obj,
                       const std::vector<double>& c, knitro::KTRISolver* solver)
                       {
    std::cout << "callbackMSProcess: " << std::endl;
    std::cout << "    Last solution: obj=" << obj << std::endl;
    for (int i = 0; i < x.size(); i++) {
      std::cout << "x[" << i << "]=" << x[i] << std::endl;
    }

    return 0;
  }
};

//#endif
