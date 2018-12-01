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
#include <chrono>
// Computing tool library
#include "interpolation.h"
#include "PID.h"
// ROS include library
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nloptcontrol_planner/Trajectory.h"
#include "mavs_msgs/state.h"
#include "mavs_msgs/control.h"
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

std::string data_path("/opt/chrono/chrono_build/data/vehicle/");

// ROS Control Input (using ROS topic)
std::vector<double> traj_t;
std::vector<double> traj_x;
std::vector<double> traj_y;
std::vector<double> traj_psi;
std::vector<double> traj_sa;
std::vector<double> traj_ux;

// Interpolator
double traj_sa_interp = 0.0;
double traj_ux_interp = 0.0;
double traj_x_interp = 0.0;
double traj_y_interp = 0.0;
int current_index = 0;

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

// =============================================================================
void waitForLoaded(ros::NodeHandle &node){
    bool is_init;
    node.setParam("system/chrono/flags/initialized", true);
    node.getParam("system/flags/initialized", is_init);

    while (!is_init) {
        node.getParam("system/flags/initialized", is_init);
    }
}

ChVector<> global2veh(double yaw_angle, ChVector<> ChVector_global) {
    // Construct rotation matrix
    ChVector<> R1(std::cos(yaw_angle), std::sin(yaw_angle), 0.0);
    ChVector<> R2(-std::sin(yaw_angle), std::cos(yaw_angle), 0.0);
    ChVector<> R3(0.0, 0.0, 1.0);

    auto veh_x = R1 ^ ChVector_global; // dot product in chrono
    auto veh_y = R2 ^ ChVector_global; // dot product in chrono
    auto veh_z = R3 ^ ChVector_global; // dot product in chrono

    return ChVector<> (veh_x, veh_y, veh_z);
}


bool receive_flag = false;


void plannerCallback(const nloptcontrol_planner::Trajectory::ConstPtr& control_msgs) {
    traj_t = control_msgs->t;
    traj_x = control_msgs->x;
    traj_y = control_msgs->y;
    traj_psi = control_msgs->psi;
    traj_sa = control_msgs->sa;
    traj_ux = control_msgs->ux;
    receive_flag = true;

    if (traj_ux.empty() || traj_x.empty() || traj_x.size() <= 1) {
        ROS_INFO("Error: Trajectory only has one point or less!");
    }
    
}

int get_nearest_index(const ChVector<>& pos_global,
                      const std::vector<double>& traj_x,
                      const std::vector<double>& traj_y) {

    if (traj_x.empty() || traj_x.size() == 1) {
        return 0;
    }

    int index = 0;
    double min_dist_sqr = pow(pos_global[0] - traj_x[0], 2) + pow(pos_global[1] - traj_x[0], 2);

    for (int i = 1; i < traj_x.size() - 1; i++) {
        double dist_sqr = pow(pos_global[0] - traj_x[i], 2) + pow(pos_global[1] - traj_x[i], 2);
        if (dist_sqr < min_dist_sqr) {
            min_dist_sqr = dist_sqr;
            index = i;
        }
    }
    return index;
}

double get_PosError(const ChVector<>& pos_global,
                    const std::vector<double>& traj_x,
                    const std::vector<double>& traj_y,
                    int index) {

    if (traj_x.empty() || traj_x.size() == 1) {
        return 0;
    }

    double traj_dir_x = traj_x[index + 1] - traj_x[index];
    double traj_dir_y = traj_y[index + 1] - traj_y[index];
    double traj_len = std::sqrt(traj_dir_x * traj_dir_x + traj_dir_y * traj_dir_y);

    if (traj_len < 0.01) return 0;

    traj_dir_x /= traj_len;
    traj_dir_y /= traj_len;

    double car2traj_x = traj_x[index] - pos_global[0];
    double car2traj_y = traj_y[index] - pos_global[1];
    double PosError = car2traj_y * traj_dir_x - car2traj_x * traj_dir_y;

    return PosError;
}

double get_AnglError(double yaw_angle,
                     const std::vector<double>& traj_x,
                     const std::vector<double>& traj_y,
                     int index) {

    if (traj_x.empty() || traj_x.size() == 1) {
        return 0;
    }

    double yaw_err = atan2(traj_y[index + 1] - traj_y[index], traj_x[index + 1] - traj_x[index]) - yaw_angle;
    yaw_err = std::fmod(yaw_err + M_PI, 2*M_PI) - M_PI;

    return yaw_err;
}

// =============================================================================

int main(int argc, char* argv[]) {

    // ------------------------------
    // Initialize ROS node handle
    // ------------------------------
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle node;

    // Declare ROS subscriber to subscribe planner topic
    std::string planner_namespace;
    node.getParam("system/planner",planner_namespace);
    ros::Subscriber planner_sub = node.subscribe(planner_namespace + "/control", 100, plannerCallback);
    
    std::string chrono_namespace;
    node.getParam("system/chrono/namespace", chrono_namespace);
    //veh_status message is depricated, use state and control topics and its publishers in future version
    ros::Publisher state_pub = node.advertise<mavs_msgs::state>("/state", 1);
    ros::Publisher control_pub = node.advertise<mavs_msgs::control>("/control", 1);
    mavs_msgs::state state_data;
    mavs_msgs::control control_data;

    bool gui;
    node.getParam("system/chrono/flags/gui", gui);

    // Define variables for ROS parameters server
    double step_size;
    double tire_step_size;
    double x0, y0, z0; // Initial global position
    double roll0, pitch0, yaw0, ux0, ax0, sa0; // Initial global orientation
    double terrainHeight;
    double terrainLength;  // size in X direction
    double terrainWidth;   // size in Y direction

    // Get parameters from ROS Parameter Server
    node.getParam("system/params/step_size", step_size); // ROS loop rate and Chrono step size
    node.getParam("system/params/tire_step_size", tire_step_size);
    // Rigid terrain dimensions
    node.getParam("system/chrono/field/h", terrainHeight);
    node.getParam("system/chrono/field/l", terrainLength);
    node.getParam("system/chrono/field/w", terrainWidth);
    

    node.getParam("case/actual/X0/x", x0); // initial x
    node.getParam("case/actual/X0/yVal", y0); // initial y
    node.getParam("case/actual/X0/z", z0); // initial z
    //node.getParam("case/actual/X0/v", v); // lateral velocity
    //node.getParam("case/actual/X0/r", r); // lateral velocity
    node.getParam("case/actual/X0/theta", pitch0); // initial pitch
    node.getParam("case/actual/X0/phi", roll0); // initial roll
    node.getParam("case/actual/X0/psi", yaw0); // initial yaw angle
    node.getParam("case/actual/X0/sa", sa0);
    node.getParam("case/actual/X0/ux", ux0);
    node.getParam("case/actual/X0/ax", ax0);
 

    // Load chrono vehicle_params
    double frict_coeff, rest_coeff;
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

    // Load interpolation parameter
    double time_shift;
    node.getParam("planner/nloptcontrol_planner/misc/tex", time_shift);

    // ---------------------
    // Set up PID controller
    // ---------------------
    // Velocity PID controller
    double Kp_vel, Ki_vel, Kd_vel, Kw_vel, upper_lim_vel, lower_lim_vel;
    std::string windup_method_vel; // Anti-windup method
    PID vel_controller;

    node.getParam("vehicle/chrono/controller/velocity/Kp", Kp_vel);
    node.getParam("vehicle/chrono/controller/velocity/Ki", Ki_vel);
    node.getParam("vehicle/chrono/controller/velocity/Kd", Kd_vel);
    node.getParam("vehicle/chrono/controller/velocity/Kw", Kw_vel);
    node.getParam("vehicle/chrono/controller/velocity/upper_limit", upper_lim_vel);
    node.getParam("vehicle/chrono/controller/velocity/lower_limit", lower_lim_vel);
    node.getParam("vehicle/chrono/controller/velocity/anti_windup", windup_method_vel);
    vel_controller.set_PID(Kp_vel, Ki_vel, Kd_vel, Kw_vel);
    vel_controller.set_step_size(step_size);
    vel_controller.set_output_limit(lower_lim_vel, upper_lim_vel);
    vel_controller.set_windup_metohd(windup_method_vel);
    vel_controller.initialize();

    // Steering PID controller due to distance error
    double Kp_str_dist, Ki_str_dist, Kd_str_dist, Kw_str_dist, upper_lim_str_dist, lower_lim_str_dist;
    std::string windup_method_str_dist; // Anti-windup method
    PID str_controller_dist;

    node.getParam("vehicle/chrono/controller/steering/dist_error/Kp", Kp_str_dist);
    node.getParam("vehicle/chrono/controller/steering/dist_error/Ki", Ki_str_dist);
    node.getParam("vehicle/chrono/controller/steering/dist_error/Kd", Kd_str_dist);
    node.getParam("vehicle/chrono/controller/steering/dist_error/Kw", Kw_str_dist);
    node.getParam("vehicle/chrono/controller/steering/dist_error/upper_limit", upper_lim_str_dist);
    node.getParam("vehicle/chrono/controller/steering/dist_error/lower_limit", lower_lim_str_dist);
    node.getParam("vehicle/chrono/controller/steering/dist_error/anti_windup", windup_method_str_dist);
    str_controller_dist.set_PID(Kp_str_dist, Ki_str_dist, Kd_str_dist, Kw_str_dist);
    str_controller_dist.set_step_size(step_size);
    str_controller_dist.set_output_limit(lower_lim_str_dist, upper_lim_str_dist);
    str_controller_dist.set_windup_metohd(windup_method_str_dist);
    str_controller_dist.initialize();

    // Steering PID controller due to distance error
    double Kp_str_angl, Ki_str_angl, Kd_str_angl, Kw_str_angl, upper_lim_str_angl, lower_lim_str_angl;
    std::string windup_method_str_angl; // Anti-windup method
    PID str_controller_angl;

    node.getParam("vehicle/chrono/controller/steering/angle_error/Kp", Kp_str_angl);
    node.getParam("vehicle/chrono/controller/steering/angle_error/Ki", Ki_str_angl);
    node.getParam("vehicle/chrono/controller/steering/angle_error/Kd", Kd_str_angl);
    node.getParam("vehicle/chrono/controller/steering/angle_error/Kw", Kw_str_angl);
    node.getParam("vehicle/chrono/controller/steering/angle_error/upper_limit", upper_lim_str_angl);
    node.getParam("vehicle/chrono/controller/steering/angle_error/lower_limit", lower_lim_str_angl);
    node.getParam("vehicle/chrono/controller/steering/angle_error/anti_windup", windup_method_str_angl);
    str_controller_angl.set_PID(Kp_str_angl, Ki_str_angl, Kd_str_angl, Kw_str_angl);
    str_controller_angl.set_step_size(step_size);
    str_controller_angl.set_output_limit(lower_lim_str_angl, upper_lim_str_angl);
    str_controller_angl.set_windup_metohd(windup_method_str_angl);
    str_controller_angl.initialize();


    // Declare loop rate
    ros::Rate loop_rate(1.0/step_size);

    // Set initial vehicle location and orientation
    ChVector<> initLoc(x0, y0, z0);

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
    my_hmmwv.SetInitFwdVel(ux0);
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

    // read maximum_steering_angle
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

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"Path Follower",
                    irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                        100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                        100);
    app.EnableGrid(false);
    app.SetTimestep(step_size);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();
    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;

    // Vehicle steering angle
    double long_velocity = 0.0;
    ChVector<> VehicleCOMPos = my_hmmwv.GetVehicle().GetVehicleCOMPos();
    ChQuaternion<> VehicleRot = my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
    long_velocity = my_hmmwv.GetVehicle().GetVehicleSpeedCOM();
    double q0 = VehicleRot[0];
    double q1 = VehicleRot[1];
    double q2 = VehicleRot[2];
    double q3 = VehicleRot[3];
    double yaw_angle = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

    // Collect controller output data from modules (for inter-module communication)
    double throttle_input = 0;
    double steering_input = 0;
    double braking_input = 0;

    // to calculate control error:
    std::vector<double> x_cal(3, VehicleCOMPos[0]);
    std::vector<double> y_cal(3, VehicleCOMPos[1]);
    double steering_angle;

    // wait system loaded
    waitForLoaded(node);
    while (ros::ok()) {
        // Get chrono time
        double chrono_time = my_hmmwv.GetSystem()->GetChTime();
        
         // steering control
        if (traj_x.empty() || traj_x.size() == 1) {
            steering_angle = 0;
            steering_input = 0;
        }
        else {
            int nearest_index = get_nearest_index(VehicleCOMPos, traj_x, traj_y);
            double pos_err = get_PosError(VehicleCOMPos, traj_x, traj_y, nearest_index);
            double yaw_err = get_AnglError(yaw_angle, traj_x, traj_y, nearest_index);

            steering_angle = str_controller_dist.control(pos_err) +
                             str_controller_angl.control(yaw_err);

            steering_input = steering_angle / maximum_steering_angle;
            steering_input = std::max(-1.0, std::min(1.0, steering_input));
            steering_angle = steering_input * maximum_steering_angle; // steering angle (rad)
        }
        
        // Speed control
        // --------------------------
        // interpolation using ALGLIB
        // --------------------------
        if (traj_t.size() > 1) {
            real_1d_array t_arr, sa_arr, ux_arr;

            t_arr.setcontent(traj_t.size(), &(traj_t[0]));
            sa_arr.setcontent(traj_sa.size(), &(traj_sa[0]));
            ux_arr.setcontent(traj_ux.size(), &(traj_ux[0]));

            spline1dinterpolant s_ux;
            spline1dinterpolant s_sa;

            spline1dbuildlinear(t_arr, ux_arr, s_ux);
            traj_ux_interp = spline1dcalc(s_ux, chrono_time+time_shift);
            spline1dbuildcubic(t_arr, sa_arr, s_sa);
            traj_sa_interp = spline1dcalc(s_sa, chrono_time+time_shift);
        }
        else if (traj_t.size() == 1) {
            std::cout << "traj_size = 0" << std::endl;
            traj_ux_interp = traj_ux[0];
            traj_sa_interp = traj_sa[0];

        }

        
        
        // Control speed
        double ux_err = traj_ux_interp - long_velocity;
        double vel_controller_output = vel_controller.control(ux_err);
        if (vel_controller_output > 0){
            throttle_input = vel_controller_output;
            braking_input = 0;
        }
        else {
            throttle_input = 0;
            braking_input = -vel_controller_output;
        }
        

        

        if(gui) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
        }

        // Update modules (process inputs from other modules)
        terrain.Synchronize(chrono_time);
        my_hmmwv.Synchronize(chrono_time, steering_input, braking_input, throttle_input, terrain);
        
        if(gui) {
            app.Synchronize("", steering_input, throttle_input, braking_input);
        }

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        
        if(gui) {
            app.Advance(step);
        }
        
        VehicleCOMPos = my_hmmwv.GetVehicle().GetVehicleCOMPos();
        VehicleRot = my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
        ChVector<> rot_dt = my_hmmwv.GetChassisBody()->GetWvel_loc(); // actual angular speed (expressed in local coords)
        ChVector<> VehiclePos = my_hmmwv.GetVehicle().GetVehiclePos(); // gloabal vehicle frame origin location
        
        // Compute yaw angle
        double q0 = VehicleRot[0];
        double q1 = VehicleRot[1];
        double q2 = VehicleRot[2];
        double q3 = VehicleRot[3];
        double yaw_angle = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

        ChVector<> ORI2COM = global2veh(yaw_angle, VehicleCOMPos - VehiclePos);

        ChVector<> VehicleRot_dt = my_hmmwv.GetChassisBody()->GetWvel_loc(); // actual angular speed (expressed in local coords)
        ChVector<> VehicleCOMVel_global = my_hmmwv.GetVehicle().GetVehiclePointVelocity(ORI2COM); // vehicle COM velocity (m/s)
        ChVector<> VehicleCOMAcc = my_hmmwv.GetVehicle().GetVehicleAcceleration(ORI2COM); // vehicle COM acceleration (m/s^2)
        VehicleCOMAcc[0] = std::max(-1.5, std::min(1.5, VehicleCOMAcc[0])); // let vehicle acceleration bounded in [-1.5, 1.5] (temporary solution) 

        ChVector<> VehicleCOMVel = global2veh(yaw_angle, VehicleCOMVel_global);

        // Get vertical tire force
        std::vector<double> TireForceVertical;
        for (int i = 0; i < 4; i++) {
            ChVector<> TireForce = my_hmmwv.GetTire(i)->ReportTireForce(&terrain).force;
            TireForceVertical.push_back(TireForce[2]);
        }

        // Update vehicle state
        node.setParam("/state/t", chrono_time);
        node.setParam("/state/x", VehicleCOMPos[0]);       // global x position (m)
        node.setParam("/state/y", VehicleCOMPos[1]);       // global y position (m)
        node.setParam("/state/v", VehicleCOMVel[1]);       //// lateral velocity (m/s)
        node.setParam("/state/r", VehicleRot_dt[2]);       //// yaw rate (rad/s)
        node.setParam("/state/psi", yaw_angle);            // global heading angle (yaw angle) (rad)
        node.setParam("/state/sa", steering_angle);        // steering angle at the tire (rad)
        node.setParam("/state/ux", VehicleCOMVel[0]);      //// longitudinal velocity  (vehicle frame) (m/s)
        node.setParam("/state/ax", VehicleCOMAcc[0]);      //// longitudinal acceleration (vehicle frame) (m/s^2)
        node.setParam("/control/thr", throttle_input);
        node.setParam("/control/brk", braking_input);
        node.setParam("/control/str", steering_input);

        // In future we will shift to state and control variables
        // Update state and control data
        state_data.t = chrono_time; // time in chrono simulation
        state_data.x = VehicleCOMPos[0];
        state_data.y = VehicleCOMPos[1];
        state_data.ux = VehicleCOMVel[0];
        state_data.v = VehicleCOMVel[1];
        state_data.ax = VehicleCOMAcc[0];
        state_data.psi = yaw_angle; // yaw angle (rad)
        state_data.r = VehicleRot_dt[2];// yaw rate (rad/s)
        state_data.sa = steering_angle; // steering angle at the tire (rad)
        control_data.t = chrono_time;
        control_data.thrt_in = throttle_input; // throttle input in the range [0,+1]
        control_data.brk_in = braking_input; // braking input in the range [0,+1]
        control_data.str_in = steering_input; // steeering input in the range [-1,+1]

        state_pub.publish(state_data);
        control_pub.publish(control_data);
        if(gui) {
            app.EndScene();
        }

        ros::spinOnce();
        // loop_rate.sleep();
    }

    return 0;
}
