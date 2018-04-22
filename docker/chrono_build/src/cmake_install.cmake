# Install script for directory: /opt/chrono/chrono_source/chrono/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/opt/chrono/chrono_source/chrono/src/chrono_thirdparty" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.cuh$" REGEX "/[^/]*\\.hpp$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono" TYPE FILE FILES "/opt/chrono/chrono_build/chrono/ChConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono" TYPE FILE FILES "/opt/chrono/chrono_build/chrono/ChVersion.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib64/cmake" TYPE FILE RENAME "ChronoConfig.cmake" FILES "/opt/chrono/chrono_build/cmake/ChronoConfig.cmake.install")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/opt/chrono/chrono_build/src/chrono/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_mkl/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_matlab/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_cascade/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_irrlicht/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_postprocess/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_cosimulation/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_fea/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_python/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_parallel/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_opengl/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_vehicle/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_fsi/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/chrono_models/cmake_install.cmake")
  include("/opt/chrono/chrono_build/src/demos/cmake_install.cmake")

endif()

