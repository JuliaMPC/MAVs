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
// Demonstration of a steering path-follower PID controller.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

// C/C++ library
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <unistd.h>

// Computing tool library
#include "interpolation.h"
#include "PID.h"

// ROS include library
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nloptcontrol_planner/Control.h"
#include "ros_chrono_msgs/veh_status.h"

// Chrono include library
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace alglib;

// Default Chrono vehicle model parameters data path
std::string data_path("/opt/chrono/chrono_build/data/vehicle/");

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 500.0;  // size in X direction
double terrainWidth = 500.0;   // size in Y direction

// ROS Control Input (Topic)
std::vector<double> traj_t(2,0);
std::vector<double> traj_x(2,0);
std::vector<double> traj_y(2,0);
std::vector<double> traj_psi(2,0);
std::vector<double> traj_sa(2,0);
std::vector<double> traj_vx(2,0);

// Interpolator
double traj_sa_interp = 0.0;
double traj_vx_interp = 0.0;

// ------
// Chrono
// ------
// Contact method type
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;

// Type of tire model (RIGID, LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::RIGID;

// Type of powertrain model (SHAFTS or SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::RWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
// Note: Compliant steering requires higher PID gains.
SteeringType steering_type = SteeringType::PITMAN_ARM;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::PRIMITIVES;
VisualizationType tire_vis_type = VisualizationType::NONE;

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// =============================================================================

void plannerCallback(const nloptcontrol_planner::Control::ConstPtr& control_msgs) {
    traj_t = control_msgs->t;
    traj_x = control_msgs->x;
    traj_y = control_msgs->y;
    traj_psi = control_msgs->psi;
    traj_sa = control_msgs->sa;
    traj_vx = control_msgs->vx;
}

// =============================================================================

int main(int argc, char* argv[]) {
    
    // ------------------------------
    // Initialize ROS node handle
    // ------------------------------
    ros::init(argc, argv, "velocity_controller");
    ros::NodeHandle node;
    
    // Declare ROS subscriber to subscribe planner topic
    std::string planner_namespace;
    node.getParam("system/planner", planner_namespace);
    ros::Subscriber planner_sub = node.subscribe(planner_namespace+"/control", 100, plannerCallback);

    // Declare ROS publisher to advertise vehicleinfo topic
    std::string chrono_namespace;
    node.getParam("system/chrono/namespace", chrono_namespace);
    ros::Publisher vehicleinfo_pub = node.advertise<ros_chrono_msgs::veh_status>(chrono_namespace+"/vehicleinfo", 1);
    ros_chrono_msgs::veh_status vehicleinfo_data;

    // Load system parameters
    double goal_tol = 0.1; // Path following tolerance
    double target_speed = 0.0; // Desired velocity
    double x0 = 0.0, y0 = 0.0, z0 = 0.5; // Initial global position
    double roll0 = 0.0, pitch0 = 0.0, yaw0 = 0.0; // Initial global orientation
    double x = 0, y = 0;
    double goal_x = 0, goal_y = 0; // Goal position
    double step_size = 5e-3; // Simulation step size 
    double tire_step_size = step_size / 2.0;
    
    node.getParam("system/params/step_size", step_size); // ROS loop rate and Chrono step size
    node.getParam("system/params/goal_tol",goal_tol);

    node.getParam("state/chrono/X0/v_des", target_speed); // desired velocity
    node.getParam("state/chrono/X0/theta", pitch0); // initial pitch
    node.getParam("state/chrono/X0/phi", roll0); // initial roll
    node.getParam("state/chrono/X0/z", z0); // initial z
    node.getParam("state/chrono/x", x); // global x position
    node.getParam("state/chrono/yVal", y); // global y position
    
    node.getParam("case/actual/X0/psi",yaw0); // initial yaw
    node.getParam("case/actual/X0/x",x0); // initial x
    node.getParam("case/actual/X0/yVal",y0); // initial y
    
    node.getParam("case/goal/x",goal_x);
    node.getParam("case/goal/yVal",goal_y);

    // Load chrono vehicle_params
    double frict_coeff, rest_coeff, gear_ratios;
    std::vector<double> centroidLoc, centroidOrientation;
    double chassisMass;
    std::vector<double> chassisInertia;
    std::vector<double> driverLoc, driverOrientation;
    std::vector<double> motorBlockDirection, axleDirection;
    double driveshaftInertia, differentialBoxInertia, conicalGearRatio, differentialRatio;
    std::vector<double> gearRatios;
    double steeringLinkMass, steeringLinkRadius, steeringLinkLength;
    std::vector<double> steeringLinkInertia;
    double pinionRadius, pinionMaxAngle, maxBrakeTorque;
    node.getParam("vehicle/chrono/vehicle_params/frict_coeff", frict_coeff);
    node.getParam("vehicle/chrono/vehicle_params/rest_coeff", rest_coeff);
    node.getParam("vehicle/chrono/vehicle_params/gearRatios", gear_ratios);
    node.getParam("vehicle/chrono/vehicle_params/centroidLoc", centroidLoc);
    node.getParam("vehicle/chrono/vehicle_params/centroidOrientation", centroidOrientation);
    node.getParam("vehicle/chrono/vehicle_params/chassisMass", chassisMass);
    node.getParam("vehicle/chrono/vehicle_params/chassisInertia", chassisInertia);
    node.getParam("vehicle/chrono/vehicle_params/driverLoc", driverLoc);
    node.getParam("vehicle/chrono/vehicle_params/driverOrientation", driverOrientation);
    node.getParam("vehicle/chrono/vehicle_params/motorBlockDirection", motorBlockDirection);
    node.getParam("vehicle/chrono/vehicle_params/axleDirection", axleDirection);
    node.getParam("vehicle/chrono/vehicle_params/driveshaftInertia", driveshaftInertia);
    node.getParam("vehicle/chrono/vehicle_params/differentialBoxInertia", differentialBoxInertia);
    node.getParam("vehicle/chrono/vehicle_params/conicalGearRatio", conicalGearRatio);
    node.getParam("vehicle/chrono/vehicle_params/differentialRatio", differentialRatio);
    node.getParam("vehicle/chrono/vehicle_params/gearRatios", gearRatios);
    node.getParam("vehicle/chrono/vehicle_params/steeringLinkMass", steeringLinkMass);
    node.getParam("vehicle/chrono/vehicle_params/steeringLinkInertia", steeringLinkInertia);
    node.getParam("vehicle/chrono/vehicle_params/steeringLinkRadius", steeringLinkRadius);
    node.getParam("vehicle/chrono/vehicle_params/steeringLinkLength", steeringLinkLength);
    node.getParam("vehicle/chrono/vehicle_params/pinionRadius", pinionRadius);
    node.getParam("vehicle/chrono/vehicle_params/pinionMaxAngle", pinionMaxAngle);
    node.getParam("vehicle/chrono/vehicle_params/maxBrakeTorque", maxBrakeTorque);
    
    // Load PID controller parameters and set up PID controller
    double Kp = 0.5, Ki = 0.0, Kd = 0.0, Kw = 0.0, time_shift = 3.0; // PID controller parameter
    std::string windup_method("clamping"); // Anti-windup method
    double controller_output = 0.0;
    PID controller;
    node.getParam("controller/Kp",Kp);
    node.getParam("controller/Ki",Ki);
    node.getParam("controller/Kd",Kd);
    node.getParam("controller/Kw",Kw);
    node.getParam("controller/anti_windup", windup_method);
    node.getParam("controller/time_shift", time_shift);

    controller.set_PID(Kp, Ki, Kd, Kw);
    controller.set_step_size(step_size);
    controller.set_output_limit(-1.0, 1.0);
    controller.set_windup_metohd(windup_method);
    controller.initialize();
    
    // Declare loop rate
    ros::Rate loop_rate(int(1.0/step_size));

    // Set initial vehicle location and orientation
    ChVector<> initLoc(x0, y0, z0);

    // convert yaw pitch roll to quaternion
    double t0 = std::cos(yaw0 * 0.5f);
    double t1 = std::sin(yaw0 * 0.5f);
    double t2 = std::cos(roll0 * 0.5f);
    double t3 = std::sin(roll0 * 0.5f);
    double t4 = std::cos(pitch0 * 0.5f);
    double t5 = std::sin(pitch0 * 0.5f);

    double q0_0 = t0 * t2 * t4 + t1 * t3 * t5;
    double q0_1 = t0 * t3 * t4 - t1 * t2 * t5;
    double q0_2 = t0 * t2 * t5 + t1 * t3 * t4;
    double q0_3 = t1 * t2 * t4 - t0 * t3 * t5;

    ChQuaternion<> initRot(q0_0,q0_1,q0_2,q0_3);

    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetVehicleStepSize(step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    double maximum_steering_angle = my_hmmwv.GetVehicle().GetMaxSteeringAngle();

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                  ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetContactFrictionCoefficient(frict_coeff);
    patch->SetContactRestitutionCoefficient(rest_coeff);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(data_path+"terrain/textures/tile4.jpg", 200, 200);
    terrain.Initialize();

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"velocity_controller demo",
                        irr::core::dimension2d<irr::u32>(800, 640));
    
    app.SetHUDLocation(500, 20);
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.EnableGrid(false);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------
    
    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;

    // Vehicle velocity
    double speed = 0.0;
    // Collect controller output data from modules (for inter-module communication)
    double throttle_input = 0;
    double steering_input = 0;
    double braking_input = 0;

    while (ros::ok()) {
        
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();

        // --------------------------
        // interpolation using ALGLIB 
        // --------------------------
        // real_1d_array t_array;
        // real_1d_array sa_array;
        // real_1d_array vx_array;

        // t_array.setcontent(traj_t);
        // sa_array.setcontent(traj_sa);
        // vx_array.setcontent(traj_vx);
        // spline1dinterpolant s_vx;
        // spline1dinterpolant s_sa;
        // spline1dbuildcubic(t_array, vx_array, s_vx);
        // traj_vx_interp = spline1dcalc(s_vx, time + time_shift);
        // spline1dbuildcubic(t_array, sa_array, s_sa);
        // traj_sa_interp = spline1dcalc(s_sa, time);

        // PID controller output for throttle or brake
        // double vx_err = traj_vx_interp - speed;
        double vx_err = traj_vx[0] - speed;
        controller_output = controller.control(vx_err);
        if (controller_output > 0){
            throttle_input = controller_output;
            braking_input = 0;
        }
        else {
            throttle_input = 0;
            braking_input = -controller_output;
        }
        
        // Steering angle output
        // traj_sa_interp = traj_sa_interp;
        steering_input = traj_sa[0] / M_PI * 180.0 / maximum_steering_angle;

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Update modules (process inputs from other modules)
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        app.Synchronize("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        // Get vehicle information from Chrono vehicle model
        ChVector<> pos_global = my_hmmwv.GetVehicle().GetVehicleCOMPos();
        ChVector<> spd_global = my_hmmwv.GetChassisBody()->GetPos_dt();
        ChVector<> acc_global = my_hmmwv.GetChassisBody()->GetPos_dtdt();
        ChQuaternion<> rot_global = my_hmmwv.GetVehicle().GetVehicleRot(); // global orientation as quaternion
        ChVector<> rot_dt = my_hmmwv.GetChassisBody()->GetWvel_loc(); //global orientation as quaternion
        
        double slip_angle = my_hmmwv.GetTire(0)->GetSlipAngle();
        speed = my_hmmwv.GetVehicle().GetVehicleSpeedCOM();

        double q0 = rot_global[0];
        double q1 = rot_global[1];
        double q2 = rot_global[2];
        double q3 = rot_global[3];

        double yaw_val=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
        double theta_val=asin(2*(q0*q2-q3*q1));
        double phi_val= atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));

        // Update vehicleinfo_data
        vehicleinfo_data.t_chrono = time; // time in chrono simulation
        vehicleinfo_data.x_pos = pos_global[0];
        vehicleinfo_data.y_pos = pos_global[1];
        vehicleinfo_data.x_v = spd_global[0]; // speed measured at the origin of the chassis reference frame.
        vehicleinfo_data.y_v = spd_global[1];
        vehicleinfo_data.x_a = acc_global[0];
        vehicleinfo_data.yaw_curr = yaw_val; // in radians
        vehicleinfo_data.yaw_rate = -rot_dt[2];// yaw rate
        vehicleinfo_data.sa = slip_angle; // slip angle
        vehicleinfo_data.thrt_in = throttle_input; // throttle input in the range [0,+1]
        vehicleinfo_data.brk_in = braking_input; // braking input in the range [0,+1]
        vehicleinfo_data.str_in = steering_input; // steeering input in the range [-1,+1]

        // Publish current vehicle information
        vehicleinfo_pub.publish(vehicleinfo_data);

        app.EndScene();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
