# Install script for directory: /opt/chrono/chrono_source/chrono/src/chrono_fea

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib64" TYPE SHARED_LIBRARY FILES "/opt/chrono/chrono_build/lib64/libChronoEngine_fea.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so"
         OLD_RPATH "/opt/chrono/chrono_build/lib64:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib64/libChronoEngine_fea.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/chrono_fea" TYPE FILE FILES
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChApiFEA.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChBeamSection.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChBuilderBeam.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChContinuumPoisson3D.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChContinuumThermal.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChContinuumElectrostatics.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAbase.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAxyz.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAxyzrot.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAxyzP.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAxyzD.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAxyzDD.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChNodeFEAcurv.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBase.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementGeneric.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementCorotational.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementSpring.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBar.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBeam.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBeamANCF.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBeamEuler.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementCableANCF.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBrick.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementBrick_9.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElement3D.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementTetrahedron.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementTetra_4.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementTetra_10.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementHexahedron.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementHexa_8.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementHexa_20.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementShell.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementShellANCF.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChElementShellReissner4.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChFaceTetra_4.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChFaceBrick_9.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChFaceHexa_8.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLoadsBeam.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChGaussIntegrationRule.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChGaussPoint.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChMesh.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChMeshFileLoader.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChMatterMeshless.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChProximityContainerMeshless.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChPolarDecomposition.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChMatrixCorotation.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChVisualizationFEAmesh.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLinkInterface.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLinkPointFrame.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLinkDirFrame.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLinkPointPoint.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLinkPointTriface.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChContactSurface.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChContactSurfaceNodeCloud.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChContactSurfaceMesh.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChMeshSurface.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChUtilsFEA.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChLoadContactSurfaceMesh.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChRotUtils.h"
    "/opt/chrono/chrono_source/chrono/src/chrono_fea/ChMaterialShellReissner.h"
    )
endif()

