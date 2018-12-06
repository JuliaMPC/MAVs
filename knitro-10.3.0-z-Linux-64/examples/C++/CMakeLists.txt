######################################################################
#
#  Copyright (c) 2016-1017 by Artelys.
#
######################################################################

# make a directory build in KTR, go there and run “cmake ..”.
# may need to specify generator in Windows:
# e.g.: for Visual Studio 2013 on x64, run cmake -G "Visual Studio 12 Win64" ..
# need separate build folders for Win32 and Win64
# The makefile will be created there. It uses KNITRODIR environment variable.

cmake_minimum_required(VERSION 2.8.6)

project(KnitroExamples)

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include)
include_directories($ENV{KNITRODIR}/include)

set(KTR_RELEASE 1030)
set(KNITRO_LIB  libknitro${KTR_RELEASE}.so)

if (APPLE)
  set(KNITRO_LIB "libknitro.dylib")
endif()

if(WIN32)
  set(KNITRO_LIB "knitro${KTR_RELEASE}.lib")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

add_executable(ExBasic examples/ExBasic.cc)
target_link_libraries(ExBasic $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExHS15 examples/ExHS15.cc)
target_link_libraries(ExHS15 $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExHS15_scale examples/ExHS15_scale.cc)
target_link_libraries(ExHS15_scale $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExHS15_termination examples/ExHS15_termination.cc)
if (UNIX)
  # This target requires C++ 11 (or older c++ 0x)
  set_target_properties(ExHS15_termination PROPERTIES COMPILE_FLAGS "-std=c++0x")
endif()
target_link_libraries(ExHS15_termination $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExHessVect examples/ExHessVect.cc)
target_link_libraries(ExHessVect $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExDerivatives examples/ExDerivatives.cc)
target_link_libraries(ExDerivatives $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExFCGASimultaneous examples/ExFCGASimultaneous.cc)
target_link_libraries(ExFCGASimultaneous $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMPEC examples/ExMPEC.cc)
target_link_libraries(ExMPEC $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMINLP examples/ExMINLP.cc)
target_link_libraries(ExMINLP $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMINLP_misqp examples/ExMINLP_misqp.cc)
target_link_libraries(ExMINLP_misqp $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMINLP_reform examples/ExMINLP_reform.cc)
target_link_libraries(ExMINLP_reform $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMINLPCallback examples/ExMINLPCallback.cc)
target_link_libraries(ExMINLPCallback $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMsInitPtCallback examples/ExMsInitPtCallback.cc)
target_link_libraries(ExMsInitPtCallback $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExMsProcessCallback examples/ExMsProcessCallback.cc)
target_link_libraries(ExMsProcessCallback $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExNewptCallback examples/ExNewptCallback.cc)
target_link_libraries(ExNewptCallback $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExOutputRedirection examples/ExOutputRedirection.cc)
target_link_libraries(ExOutputRedirection $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExRestart examples/ExRestart.cc)
target_link_libraries(ExRestart $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExResetBounds examples/ExResetBounds.cc)
target_link_libraries(ExResetBounds $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExTuner examples/ExTuner.cc)
target_link_libraries(ExTuner $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

add_executable(ExLSQ examples/ExLSQ.cc)
target_link_libraries(ExLSQ $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})

# Copy tuner files
file(GLOB KTR_PARAM_FILES examples/*.opt)
file(COPY ${KTR_PARAM_FILES} DESTINATION .)

add_executable(ExZLM examples/ExZLM.cc)
target_link_libraries(ExZLM $ENV{KNITRODIR}/lib/${KNITRO_LIB} ${CMAKE_DL_LIBS})
