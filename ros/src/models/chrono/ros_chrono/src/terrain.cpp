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

//#define USE_JSON

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------------
	// Initialize ROS node handle
	// ------------------------------
	ros::init(argc, argv, "path_follower");
	ros::NodeHandle node;

    // --------------
    // Create systems
    // --------------

    // Simulation step sizes
    double step_size = 3e-3;
    double tire_step_size = 1e-3;

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced my_hmmwv;
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetChassisCollisionType(ChassisCollisionType::NONE);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
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

#ifdef USE_JSON
    // Create the terrain from JSON specification file
    RigidTerrain terrain(my_hmmwv.GetSystem(), vehicle::GetDataFile("terrain/RigidPatches.json"), true);
#else
    // get parameters for terrain
//double pax;
//    double pay;
//    double paz;
//    double sax;
//    double say;
//    double saz;
    std::vector<double> pa;
    std::vector<double> sa;
    std::vector<double> pb;
    std::vector<double> sb;
    std::vector<double> pc;
    std::vector<double> sc;
    std::vector<double> pd;
    std::vector<double> sd;
    node.getParam("system/chrono/field/pa", pa);
    node.getParam("system/chrono/field/sa", sa);
    node.getParam("system/chrono/field/pb", pb);
    node.getParam("system/chrono/field/sb", sb);
    node.getParam("system/chrono/field/pc", pc);
    node.getParam("system/chrono/field/sc", sc);
    node.getParam("system/chrono/field/pd", pd);
    node.getParam("system/chrono/field/sd", sd);

    // Create the terrain patches programatically
    RigidTerrain terrain(my_hmmwv.GetSystem());

    auto patch1 = terrain.AddPatch(ChCoordsys<>(ChVector<>(pa[1], pa[2], pa[3]), QUNIT), ChVector<>(sa[1],sa[2], sa[3]));
    patch1->SetContactFrictionCoefficient(0.9f);
    patch1->SetContactRestitutionCoefficient(0.01f);
    patch1->SetContactMaterialProperties(2e7f, 0.3f);
    patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch1->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), pa[1], pa[2]);

    auto patch2 = terrain.AddPatch(ChCoordsys<>(ChVector<>(pb[1], pb[2], pb[3]), QUNIT), ChVector<>(sb[1],sb[2], sb[3]));
    patch2->SetContactFrictionCoefficient(0.9f);
    patch2->SetContactRestitutionCoefficient(0.01f);
    patch2->SetContactMaterialProperties(2e7f, 0.3f);
    patch2->SetColor(ChColor(1.0f, 0.5f, 0.5f));
    patch2->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), pb[1], pb[2]);

    auto patch3 = terrain.AddPatch(ChCoordsys<>(ChVector<>(pc[1], pc[2], pc[3]), QUNIT), ChVector<>(sc[1],sc[2],sc[3]));
    patch3->SetContactFrictionCoefficient(0.9f);
    patch3->SetContactRestitutionCoefficient(0.01f);
    patch3->SetContactMaterialProperties(2e7f, 0.3f);
    patch3->SetColor(ChColor(0.5f, 0.5f, 0.8f));
    patch3->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), pc[1], pc[2]);

    auto patch4 = terrain.AddPatch(ChCoordsys<>(ChVector<>(pd[1], pd[2], pd[3]), QUNIT), ChVector<>(sd[1],sd[2], sd[3]));
    patch4->SetContactFrictionCoefficient(0.9f);
    patch4->SetContactRestitutionCoefficient(0.01f);
    patch4->SetContactMaterialProperties(2e7f, 0.3f);
    patch4->SetColor(ChColor(0.5f, 0.5f, 0.8f));
    patch4->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), pd[1], pd[2]);

    terrain.Initialize();
#endif

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
