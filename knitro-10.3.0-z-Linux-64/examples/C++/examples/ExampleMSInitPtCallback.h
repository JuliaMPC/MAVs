/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
//#ifndef _EXAMPLEMSINITPTCALLBACK_H_
//#define _EXAMPLEMSINITPTCALLBACK_H_
#pragma once

#include "KTRSolver.h"
#include "KTRMsInitptCallback.h"
#include <iostream>

/**
 * An example implementation of KTRMSInitptCallback. Each time the multistart solve is initialized, this callback is called and generates a random initial primal variable value, with each component of variable randomly generated in the interval [minimum, maximum], rather than the default values.
 */
class ExampleMSInitPtCallback : public knitro::KTRMSInitptCallback {
 public:

  ExampleMSInitPtCallback(double minimum, double maximum)
      : _minimum(minimum),
        _maximum(maximum) {
  }

  int CallbackFunction(int nSolveNumber, const std::vector<double>& xLoBnds, const std::vector<double>& xUpBnds,
                       std::vector<double>& x, std::vector<double>& lambda, knitro::KTRISolver* userParams)
                       {
    std::cout << "using user-generated start points in range [" << _minimum << ", " << _maximum << "]" << std::endl;

    for (int i = 0; i < xLoBnds.size(); i++) {
      double u = static_cast<double>(rand()) / RAND_MAX;
      x[i] = _minimum + (_maximum - _minimum) * u;
    }

    return 0;
  }

 private:
  const double _minimum;
  const double _maximum;
};

//#endif
