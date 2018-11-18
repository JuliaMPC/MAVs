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
#include "nloptcontrol_planner/Trajectory.h"
#include "ros_chrono_msgs/veh_status.h"
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

const double PI  = 3.14159265359;

std::string data_path("../../../src/models/chrono/ros_chrono/src/data/vehicle/");

// The extended steering controller only works inside the path limits
// =============================================================================
// Problem parameters

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
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::NONE;

// Input file names for the path-follower driver model
////std::string path_file("paths/straight.txt");
////std::string path_file("paths/curve.txt");
////std::string path_file("paths/NATO_double_lane_change.txt");
// std::string path_file("paths/ISO_double_lane_change.txt");
std::string path_file("paths/switch_lane.txt");

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 5e-3;
double tire_step_size = 2.5e-3;

// Simulation end time
double t_end = 100;

// Render FPS
double fps = 60;

int filter_window_size = 20;

// Control Input
std::vector<double> traj_t;
std::vector<double> traj_x;
std::vector<double> traj_y;
std::vector<double> traj_psi;
std::vector<double> traj_sa;
std::vector<double> traj_ux;

bool receive_flag = false;

// Control Output
double traj_sa_interp = 0.0;
double traj_ux_interp = 0.0;
double controller_output = 0.0;
PID controller;
PID path_follower_controller_dist;
PID path_follower_controller_angle;

// =============================================================================

// Custom Irrlicht event receiver for selecting current driver model.
class ChDriverSelector : public irr::IEventReceiver {
    public:
        ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriver* driver_follower, ChIrrGuiDriver* driver_gui)
            : m_vehicle(vehicle),
            m_driver_follower(driver_follower),
            m_driver_gui(driver_gui),
            m_driver(m_driver_follower),
            m_using_gui(false) {}

        ChDriver* GetDriver() { return m_driver; }
        bool UsingGUI() const { return m_using_gui; }

        virtual bool OnEvent(const irr::SEvent& event) {
            return false;
        }

    private:
        bool m_using_gui;
        const ChVehicle& m_vehicle;
        ChPathFollowerDriver* m_driver_follower;
        ChIrrGuiDriver* m_driver_gui;
        ChDriver* m_driver;
};

// =============================================================================

void plannerCallback(const nloptcontrol_planner::Trajectory::ConstPtr& control_msgs) {
    traj_t = control_msgs->t;
    traj_x = control_msgs->x;
    traj_y = control_msgs->y;
    traj_psi = control_msgs->psi;
    traj_sa = control_msgs->sa;
    traj_ux = control_msgs->ux;
    receive_flag = true;
}

double get_PosError(ChVector<> pos_global,
                    std::vector<double> traj_x,
                    std::vector<double> traj_y) {
    double traj_dir_x = traj_x[1] - traj_x[0];
    double traj_dir_y = traj_y[1] - traj_y[0];
    double traj_len = std::sqrt(traj_dir_x * traj_dir_x + traj_dir_y * traj_dir_y);

    if (traj_len < 0.01) return 0;

    traj_dir_x /= traj_len;
    traj_dir_y /= traj_len;

    double car2traj_x = traj_x[0] - pos_global[0];
    double car2traj_y = traj_y[0] - pos_global[1];
    double PosError = car2traj_y * traj_dir_x - car2traj_x * traj_dir_y;

    return PosError;
}

// =============================================================================

int main(int argc, char* argv[]) {

    // ------------------------------
    // Initialize ROS node handle
    // ------------------------------
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle node;

    // Rigid terrain dimensions
    double terrainHeight;
    double terrainLength;  // size in X direction
    double terrainWidth;   // size in Y direction

    node.getParam("system/terrain/terrainHeight",terrainHeight);
    node.getParam("system/terrain/terrainLength",terrainLength);
    node.getParam("system/terrain/terrainWidth",terrainWidth);

    // Declare ROS subscriber to subscribe planner topic
    std::string planner_namespace;
    node.getParam("system/planner",planner_namespace);
    ros::Subscriber planner_sub = node.subscribe(planner_namespace + "/control", 100, plannerCallback);
    //ros::Subscriber planner_sub = node.subscribe("/control", 100, plannerCallback);

    // Declare ROS publisher to advertise vehicleinfo topic
    // This topic is depricated and kept hee for backward compatibility
    ros::Publisher vehicleinfo_pub = node.advertise<ros_chrono_msgs::veh_status>("/vehicleinfo", 1);
    //We will use state_pub and control_pub in future
    ros::Publisher state_pub = node.advertise<mavs_msgs::state>("/state", 1);
    ros::Publisher control_pub = node.advertise<mavs_msgs::control>("/control", 1);
    ros_chrono_msgs::veh_status vehicleinfo_data;
    mavs_msgs::state state_data;
    mavs_msgs::control control_data;



    // Get parameters from ROS Parameter Server
    double goal_tol = 0.1;
    node.getParam("system/params/step_size",step_size);
    node.getParam("system/params/goal_tol",goal_tol);

    double target_speed = 12.0, pitch0 = 0, roll0 = 0, x = 0, y = 0, z0 = 0.5;
    node.getParam("state/chrono/X0/v_des",target_speed);
    node.getParam("state/chrono/X0/theta",pitch0);
    node.getParam("state/chrono/X0/phi",roll0);
    node.getParam("state/chrono/x",x);
    node.getParam("state/chrono/yVal",y);
    node.getParam("state/chrono/X0/z",z0);

    double yaw0 = 0, x0 = -115, y0 = -120;
    node.getParam("case/actual/X0/psi",yaw0);
    node.getParam("case/actual/X0/x",x0);
    node.getParam("case/actual/X0/yVal",y0);

    double goal_x = 0, goal_y = 0;
    node.getParam("case/goal/x",goal_x);
    node.getParam("case/goal/yVal",goal_y);

    double frict_coeff = 0, rest_coeff = 0, gear_ratios = 1;
    node.getParam("vehicle/common/frict_coeff",frict_coeff);
    node.getParam("vehicle/common/rest_coeff",rest_coeff);
    node.getParam("vehicle/chrono/vehicle_params/gearRatios",gear_ratios);

    // ---------------------
    // Set up PID controller
    // ---------------------
    double Kp = 0.5, Ki = 0.0, Kd = 0.0, Kw = 0.002, time_shift = 3.0;
    std::string windup_method("clamping");
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

    double Kp_dist = 0.05, Ki_dist = 0.0, Kd_dist = 0.0, Kw_dist = 0.002;
    path_follower_controller_dist.set_PID(Kp_dist, Ki_dist, Kd_dist, Kw_dist);
    path_follower_controller_dist.set_step_size(step_size);
    path_follower_controller_dist.set_output_limit(-0.5, 0.5);
    path_follower_controller_dist.set_windup_metohd(windup_method);
    path_follower_controller_dist.initialize();

    double Kp_angle = 0.5, Ki_angle = 0.0, Kd_angle = 0.0, Kw_angle = 0.002;
    path_follower_controller_angle.set_PID(Kp_angle, Ki_angle, Kd_angle, Kw_angle);
    path_follower_controller_angle.set_step_size(step_size);
    path_follower_controller_angle.set_output_limit(-0.5, 0.5);
    path_follower_controller_angle.set_windup_metohd(windup_method);
    path_follower_controller_angle.initialize();

    // Declare loop rate
    ros::Rate loop_rate(int(1/step_size));

    // Initial vehicle location and orientation
    ChVector<> initLoc(x0, y0, z0);
    // ChQuaternion<> initRot(q[0],q[1],q[2],q[3]);

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

    // ChQuaternion<> initRot(q0_0,q0_1,q0_2,q0_3);
    ChQuaternion<> initRot(1, 0, 0, 0);

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

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                  ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetContactFrictionCoefficient(0.8f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(data_path+"terrain/textures/tile4.jpg", 200, 200);
    terrain.Initialize();

    // ----------------------
    // Create the Bezier path
    // ----------------------

    // From data file
    auto path = ChBezierCurve::read(data_path+path_file);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"Steering PID Controller Demo",
                        irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.EnableGrid(false);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // -------------------------
    // Create the driver systems
    // -------------------------

    // Create both a GUI driver and a path-follower and allow switching between them
    ChIrrGuiDriver driver_gui(app);
    driver_gui.Initialize();

    ChPathFollowerDriver driver_follower(my_hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver_follower.GetSteeringController().SetLookAheadDistance(5);
    driver_follower.GetSteeringController().SetGains(0.8, 0, 0);
    driver_follower.GetSpeedController().SetGains(0.4, 0, 0);
    driver_follower.Initialize();

    // Create and register a custom Irrlicht event receiver to allow selecting the
    // current driver model.
    ChDriverSelector selector(my_hmmwv.GetVehicle(), &driver_follower, &driver_gui);

    app.SetUserEventReceiver(&selector);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;
    int render_frame = 0;

    double speed = 0.0;
    // Collect output data from modules (for inter-module communication)
    double throttle_input = 0;
    double steering_input = 0;
    double braking_input = 0;

    bool is_init;
    node.setParam("system/chrono/flags/initialized", true);

    node.getParam("system/flags/initialized", is_init);

    while (!is_init) {
        node.getParam("system/flags/initialized", is_init);
        std::cout << "waiting for waiting on obstacle_avoidance.jl in nloptcontrol_planner ..." << std::endl;
    }

    while (ros::ok()) {

        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();
        ChVector<> acc_CG = my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = my_hmmwv.GetVehicle().GetVehicleAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        // End simulation
        if (time >= t_end)
            break;

        // --------------------------
        // interpolation using ALGLIB
        // --------------------------
        // real_1d_array t_array;
        // real_1d_array sa_array;
        // real_1d_array ux_array;

        // t_array.setcontent(traj_t);
        // sa_array.setcontent(traj_sa);
        // ux_array.setcontent(traj_ux);
        // spline1dinterpolant s_ux;
        // spline1dinterpolant s_sa;
        // spline1dbuildcubic(t_array, ux_array, s_ux);
        // traj_ux_interp = spline1dcalc(s_ux, time + time_shift);
        // spline1dbuildcubic(t_array, sa_array, s_sa);
        // traj_sa_interp = spline1dcalc(s_sa, time);

        // throttle or brake output
        // double ux_err = traj_ux_interp - speed;


        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver_follower.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver_follower.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Update modules (process inputs from other modules)
        driver_follower.Synchronize(time);
        driver_gui.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        std::string msg = selector.UsingGUI() ? "GUI driver" : "Follower driver";
        app.Synchronize(msg, steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver_follower.Advance(step);
        driver_gui.Advance(step);
        terrain.Advance(step);
        my_hmmwv.Advance(step);
        app.Advance(step);

        // Get vehicle information from Chrono vehicle model
        ChVector<> pos_global = my_hmmwv.GetVehicle().GetVehicleCOMPos();
        std::cout << "pos_global: " << pos_global[0] << std::endl;
        ChVector<> spd_global = my_hmmwv.GetChassisBody()->GetPos_dt();
        ChVector<> acc_global = my_hmmwv.GetChassisBody()->GetPos_dtdt();
        ChQuaternion<> rot_global = my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
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

        // Control speed
        if (receive_flag) {
            double ux_err = traj_ux[traj_ux.size() - 1] - speed;

            // if ((traj_x[1] - traj_x[0])*(traj_x[1] - pos_global[0]) +
            //     (traj_y[1] - traj_y[0])*(traj_y[1] - pos_global[1]) < 0)
            //         ux_err = -speed;

            controller_output = controller.control(ux_err);
            if (controller_output > 0){
                throttle_input = controller_output;
                braking_input = 0;
            }
            else {
                throttle_input = 0;
                braking_input = -controller_output;
            }
        }
        else {
            throttle_input = 0;
            braking_input = 0;
        }


        // std::cout << "pos_global: " << pos_global[0] << ", traj_x: " << traj_x[0] << "traj_y: " << traj_y[0] <<std::endl;

        // Control angle
        if (receive_flag) {
            double pos_err = get_PosError(pos_global, traj_x, traj_y);

            std::cout << "atan2:      "<<atan2(traj_y[1] - traj_y[0],traj_x[1] - traj_x[0])<<"\n";
            double yaw_err = atan2(traj_y[1] - traj_y[0],traj_x[1] - traj_x[0]) - yaw_val;
            yaw_err = std::fmod(yaw_err + PI, 2*PI) - PI;
            steering_input = path_follower_controller_dist.control(pos_err) +
                             path_follower_controller_angle.control(yaw_err);

            std::cout << "pos_err: " << pos_err << ", yaw_err: " << yaw_err << "steering_input: " << steering_input <<std::endl;
        }
        else {
            steering_input = 0;
        }
        // steering angle output (There should a saturation threshold)
        // traj_sa_interp = traj_sa_interp;

        // Update vehicleinfo_data
        // This is depricated, use state and control instead
        vehicleinfo_data.t_chrono = time; // time in chrono simulation
        vehicleinfo_data.x_pos = pos_global[0];
        vehicleinfo_data.y_pos = pos_global[1];
        // vehicleinfo_data.x_v = spd_global[0]; // speed measured at the origin of the chassis reference frame.
        vehicleinfo_data.x_v = speed;
        vehicleinfo_data.y_v = spd_global[1];
        vehicleinfo_data.x_a = acc_global[0];
        vehicleinfo_data.yaw_curr = yaw_val; // in radians
        vehicleinfo_data.yaw_rate = -rot_dt[2];// yaw rate
        vehicleinfo_data.sa = slip_angle; // slip angle
        vehicleinfo_data.thrt_in = throttle_input; // throttle input in the range [0,+1]
        vehicleinfo_data.brk_in = braking_input; // braking input in the range [0,+1]
        vehicleinfo_data.str_in = steering_input; // steeering input in the range [-1,+1]

        // In future we will shift to state and control variables
        state_data.t= time; // time in chrono simulation
        state_data.x = pos_global[0];
        state_data.y = pos_global[1];
        state_data.ux= spd_global[0];
        state_data.v = spd_global[1];
        state_data.ax= acc_global[0];
        state_data.psi= yaw_val; // in radians
        state_data.r = -rot_dt[2];// yaw rate
        //state_data.sa = steering_input*maximum_steering_angle;
        state_data.sa = slip_angle;
        control_data.t = time;
        control_data.thrt_in = throttle_input; // throttle input in the range [0,+1]
        control_data.brk_in = braking_input; // braking input in the range [0,+1]
        control_data.str_in = steering_input; // steeering input in the range [-1,+1]

        // Publish current vehicle information
        vehicleinfo_pub.publish(vehicleinfo_data);
        state_pub.publish(state_data);
        control_pub.publish(control_data);

        app.EndScene();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
