/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include <string>
#include <iostream>
#include "KTRSolver.h"
#include "KTRPutString.h"

/**
 * An example of implementing output redirection. Writes KNITRO output to stderr instead of the default stdout.
 */
class ExampleOutputRedirection : public knitro::KTRPutString {
 public:

  int CallbackFunction(const std::string &str, knitro::KTRISolver * solver)
  {
    std::cerr << str;

    return static_cast<int>(str.length());
  }
};

