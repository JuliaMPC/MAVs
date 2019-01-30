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
// Demonstration of using a RigidTerrain constructed from different patches.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
// C/C++ library
#include <vector>

// ROS include library
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

std::string data_path("/opt/chrono/chrono_build/data/vehicle/");

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------------
  	// Initialize ROS node handle
  	// ------------------------------
  	ros::init(argc, argv, "terrain");
  	ros::NodeHandle node;

    // --------------
    // Create systems
    // --------------

    // Simulation step sizes
    double step_size = 3e-3;
    double tire_step_size = 1e-3;

    // Create the HMMWV vehicle, set parameters, and initialize
    double x0, y0, z0; // Initial global position
    node.getParam("case/actual/X0/x", x0); // initial x
    node.getParam("case/actual/X0/yVal", y0); // initial y
  	node.getParam("case/actual/X0/z", z0); // initial z
    ChVector<> initLoc(x0, y0, z0);

    HMMWV_Reduced my_hmmwv;
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisCollisionType(ChassisCollisionType::NONE);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, ChQuaternion<>(1, 0, 0, 0)));
    my_hmmwv.SetPowertrainType(PowertrainModelType::SIMPLE);
    my_hmmwv.SetDriveType(DrivelineType::RWD);
    my_hmmwv.SetTireType(TireModelType::RIGID);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    my_hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the terrain patches programatically
  	RigidTerrain terrain(my_hmmwv.GetSystem());
  	std::vector<double> terrainPosition;
  	double terrainLength;  // size in X direction
  	double terrainWidth;   // size in Y direction
  	double frictCoeff, restCoeff;

  	node.getParam("system/chrono/terrain/position", terrainPosition);
  	node.getParam("system/chrono/terrain/length", terrainLength);
  	node.getParam("system/chrono/terrain/width", terrainWidth);
  	node.getParam("system/chrono/terrain/frictCoeff", frictCoeff);
  	node.getParam("system/chrono/terrain/restCoeff", restCoeff);

  	auto patch = terrain.AddPatch	(ChCoordsys<>(ChVector<>(terrainPosition[0], terrainPosition[1], terrainPosition[2]), QUNIT),
  															 ChVector<>(terrainLength, terrainWidth, 10)); //, true, 1, true -> does not work!
  	patch->SetContactFrictionCoefficient(frictCoeff);
  	patch->SetContactRestitutionCoefficient(restCoeff);
  	patch->SetContactMaterialProperties(2e7f, 0.3f);
  	patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
  	patch->SetTexture(data_path + "terrain/textures/tile4.jpg", 200, 200);
  	terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"Rigid Terrain Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);
    driver.Initialize();

    // Set the time response for steering and throttle keyboard inputs.
    double render_step_size = 1.0 / 50;  // FPS = 50

    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    ////utils::CSV_writer out(" ");
    ////for (int ix = 0; ix < 20; ix++) {
    ////    double x = ix * 1.0;
    ////    for (int iy = 0; iy < 100; iy++) {
    ////        double y = iy * 1.0;
    ////        double z = terrain.CalcHeight(x, y);
    ////        out << x << y << z << std::endl;
    ////    }
    ////}
    ////out.write_to_file("terrain.out");

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    double time = 0;

    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        app.EndScene();
    }

    return 0;
}
