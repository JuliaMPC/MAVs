// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono configuration header file
//
// Automatically created during CMake configuration.
//
// =============================================================================

#ifndef CH_CONFIG_H
#define CH_CONFIG_H

// -----------------------------------------------------------------------------
// Macros specifying enabled Chrono modules
// -----------------------------------------------------------------------------

// If module CASCADE was enabled, define CHRONO_CASCADE
#undef CHRONO_CASCADE

// If module COSIMULATION was enabled, define CHRONO_COSIMULATION
#undef CHRONO_COSIMULATION

// If module FEA was enabled, define CHRONO_FEA
#define CHRONO_FEA

// If module IRRLICHT was enabled, define CHRONO_IRRLICHT
#define CHRONO_IRRLICHT

// If module MATLAB was enabled, define CHRONO_MATLAB
#undef CHRONO_MATLAB

// If module MKL was enabled, define CHRONO_MKL
#undef CHRONO_MKL

// If module OPENGL was enabled, define CHRONO_OPENGL
#undef CHRONO_OPENGL

// If module PARALLEL was enabled, define CHRONO_PARALLEL
#undef CHRONO_PARALLEL

// If module POSTPROCESS was enabled, define CHRONO_POSTPROCESS
#define CHRONO_POSTPROCESS

// If module PYTHON was enabled, define CHRONO_PYTHON
#undef CHRONO_PYTHON

// If module VEHICLE was enabled, define CHRONO_VEHICLE
#define CHRONO_VEHICLE

// If module FSI was enabled, define CHRONO_FSI
#undef CHRONO_FSI

// -----------------------------------------------------------------------------
// OpenMP settings
// -----------------------------------------------------------------------------

// If OpenMP is found the following define is set
//   #define CHRONO_OMP_FOUND
// Set the highest OpenMP version supported, one of:
//   #define CHRONO_OMP_VERSION "2.0"
//   #define CHRONO_OMP_VERSION "3.0"
//   #define CHRONO_OMP_VERSION "4.0"
// and define one or more of the following, as appropriate
//   #define CHRONO_OMP_20
//   #define CHRONO_OMP_30
//   #define CHRONO_OMP_40
#define CHRONO_OMP_FOUND
#define CHRONO_OMP_VERSION "4.0"
#define CHRONO_OMP_20
#define CHRONO_OMP_30
#define CHRONO_OMP_40

// If OpenMP support was enabled in the main ChronoEngine library, define CHRONO_OPENMP_ENABLED
#define CHRONO_OPENMP_ENABLED

// If TBB support was enabled in the main ChronoEngine library, define CHRONO_TBB_ENABLED
#undef CHRONO_TBB_ENABLED


// If using vectorized code


// -----------------------------------------------------------------------------
// SSE settings
// -----------------------------------------------------------------------------

// If SSE support was found, then
//   #define CHRONO_HAS_SSE
// and set the SSE level support, one of the following
//   #define CHRONO_SSE_LEVEL "1.0"
//   #define CHRONO_SSE_LEVEL "2.0"
//   #define CHRONO_SSE_LEVEL "3.0"
//   #define CHRONO_SSE_LEVEL "4.1"
//   #define CHRONO_SSE_LEVEL "4.2"
// and define one or more of the following, as appropriate
//   #define CHRONO_SSE_1.0
//   #define CHRONO_SSE_2.0
//   #define CHRONO_SSE_3.0
//   #define CHRONO_SSE_4.1
//   #define CHRONO_SSE_4.2

#define CHRONO_HAS_SSE
#define CHRONO_SSE_4_2






// -----------------------------------------------------------------------------

// If AVX support was found, then
//   #define CHRONO_HAS_AVX
// and set the SSE level support, one of the following
//   #define CHRONO_AVX_LEVEL "1.0"
//   #define CHRONO_AVX_LEVEL "2.0"
// and define one or more of the following, as appropriate
//   #define CHRONO_AVX_1.0
//   #define CHRONO_AVX_2.0

#define CHRONO_HAS_AVX
#define CHRONO_AVX_2_0




// If FMA support was found, then
//   #define CHRONO_HAS_FMA

#define CHRONO_HAS_FMA

#endif
