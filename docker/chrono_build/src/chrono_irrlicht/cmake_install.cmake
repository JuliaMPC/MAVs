# Install script for directory: /opt/chrono/chrono_source/chrono/src/chrono_irrlicht

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib64" TYPE SHARED_LIBRARY FILES "/opt/chrono/chrono_build/lib64/libChronoEngine_irrlicht.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so"
         OLD_RPATH "/opt/chrono/chrono_build/lib64:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_irrlicht.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_irrlicht" TYPE FILE FILES
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChApiIrr.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChBodySceneNode.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChBodySceneNodeTools.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrTools.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrAppInterface.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrApp.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrCamera.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrMeshTools.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrWizard.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrParticlesSceneNode.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrAssetConverter.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrNode.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrNodeProxyToAsset.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrNodeAsset.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_irrlicht/ChIrrEffects.h"
    )
endif()

