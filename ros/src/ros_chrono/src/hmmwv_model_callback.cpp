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
// TO DO:
// Find proper yaw rate calculation
// Subsscribe to msg from obstacle_avoidance_chrono, callback to that to calculate ChBezierCurve
// =============================================================================

#include <fstream>
#include<iostream>
#include "boost/bind.hpp"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"
#include <ros/console.h>
#include <ros/callback_queue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_chrono_msgs/veh_status.h"
#include "traj_gen_chrono/Control.h"
#include <sstream>
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include <math.h>
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#define PI 3.1415926535
//using veh_status.msg
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;


// =============================================================================
// Problem parameters
// Main Data Path
//std::string data_path("/home/shreyas/.julia/v0.6/MAVs/catkin_ws/data/vehicle/");
std::string data_path("../../../src/system/chrono/ros_chrono/src/data/vehicle/");
//std::string data_path("src/system/chrono/ros_chrono/src/data/vehicle/");
// Contact method type
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;

// Type of tire model (RIGID, LUGRE, FIALA, or PACEJKA)
TireModelType tire_model = TireModelType::RIGID;

// Input file name for PACEJKA tires if they are selected
std::string pacejka_tire_file(data_path+"hmmwv/tire/HMMWV_pacejka.tir");
//std::string pacejka_tire_file("hmmwv/tire/HMMWV_pacejka.tir");
// Type of powertrain model (SHAFTS or SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SIMPLE;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::RWD;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;
//double callback_act=0;

// Input file names for the path-follower driver model
std::string steering_controller_file(data_path+"generic/driver/SteeringController.json");
//std::string steering_controller_file("generic/driver/SteeringController.json");
std::string speed_controller_file(data_path+"generic/driver/SpeedController.json");
//std::string speed_controller_file("generic/driver/SpeedController.json");
// std::string path_file("paths/straight.txt");
std::string path_file(data_path+"paths/my_path.txt");

// std::string path_file("paths/curve.txt");
// std::string path_file("paths/NATO_double_lane_change.txt");
//std::string path_file(data_path+"paths/ISO_double_lane_change.txt");
//std::string path_file("paths/ISO_double_lane_change.txt");



// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 1000.0;  // size in X direction
double terrainWidth = 1000.0;   // size in Y direction

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 1e-2;
double tire_step_size = step_size;

// Simulation end time
double t_end = 100;

// Render FPS
double fps = 60;

// Debug logging
bool debug_output = false;
double debug_fps = 10;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "STEERING_CONTROLLER";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = false;
int filter_window_size = 20;


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
        // Only interpret keyboard inputs.
        if (event.EventType != irr::EET_KEY_INPUT_EVENT)
            return false;

        // Disregard key pressed
        if (event.KeyInput.PressedDown)
            return false;

        switch (event.KeyInput.Key) {
            case irr::KEY_COMMA:
                if (m_using_gui) {
                    m_driver = m_driver_follower;
                    m_using_gui = false;
                }
                return true;
            case irr::KEY_PERIOD:
                if (!m_using_gui) {
                    m_driver_gui->SetThrottle(m_driver_follower->GetThrottle());
                    m_driver_gui->SetSteering(m_driver_follower->GetSteering());
                    m_driver_gui->SetBraking(m_driver_follower->GetBraking());
                    m_driver = m_driver_gui;
                    m_using_gui = true;
                }
                return true;
            case irr::KEY_HOME:
                if (!m_using_gui && !m_driver_follower->GetSteeringController().IsDataCollectionEnabled()) {
                    std::cout << "Data collection started at t = " << m_vehicle.GetChTime() << std::endl;
                    m_driver_follower->GetSteeringController().StartDataCollection();
                }
                return true;
            case irr::KEY_END:
                if (!m_using_gui && m_driver_follower->GetSteeringController().IsDataCollectionEnabled()) {
                    std::cout << "Data collection stopped at t = " << m_vehicle.GetChTime() << std::endl;
                    m_driver_follower->GetSteeringController().StopDataCollection();
                }
                return true;
            case irr::KEY_INSERT:
                if (!m_using_gui && m_driver_follower->GetSteeringController().IsDataAvailable()) {
                    char filename[100];
                    sprintf(filename, "controller_%.2f.out", m_vehicle.GetChTime());
                    std::cout << "Data written to file " << filename << std::endl;
                    m_driver_follower->GetSteeringController().WriteOutputFile(std::string(filename));
                }
                return true;
            default:
                break;
        }

        return false;
    }

  private:
    bool m_using_gui;
    const ChVehicle& m_vehicle;
    ChPathFollowerDriver* m_driver_follower;
    ChIrrGuiDriver* m_driver_gui;
    ChDriver* m_driver;
};

struct parameters
{
    RigidTerrain terrain;
    HMMWV_Reduced my_hmmwv;
    ChRealtimeStepTimer realtime_timer;
    int sim_frame;
    double callback_act;
    double steering_input;
    double throttle_input;
    double braking_input;
    irr::scene::IMeshSceneNode* ballS;
    irr::scene::IMeshSceneNode* ballT;
} ;
//----------------------callback

void controlCallback(const traj_gen_chrono::Control::ConstPtr &msg, parameters &hmmwv_params,ChVehicleIrrApp &app,ChIrrGuiDriver &driver_gui,
double &target_speed,double &time,ros_chrono_msgs::veh_status &data_out, ros::Publisher &vehicleinfo_pub){

  app.SetPaused(1);

  std::vector<double> x_vec=msg->x;
  std::vector<double> y_vec=msg->y;
//  ROS_INFO("I heard: [%f]", x_vec[0]);
  //ROS_INFO_STREAM(msg);
  double num_pts = x_vec.size();
  double num_cols = 3;
  double z_val = 0.5;
  std::ofstream myfile;
  myfile.open(path_file,std::ofstream::out | std::ofstream::trunc);

  myfile << ' ' << num_pts << ' '<< num_cols << '\n';
  for (int pt_cnt=0; pt_cnt<num_pts;pt_cnt=pt_cnt+1){
  myfile << ' ' << x_vec[pt_cnt] << ' '<< y_vec[pt_cnt] <<' ' << z_val << '\n';
}
  myfile.close();
  app.SetPaused(0);

  driver_gui.Initialize();
  auto path = ChBezierCurve::read(path_file);

  // Initialize driver follower
  //driver_follower.Reset();
  ChPathFollowerDriver driver_follower(hmmwv_params.my_hmmwv.GetVehicle(), steering_controller_file,
                                       speed_controller_file, path, "my_path", target_speed);
  driver_follower.Initialize();

  // Create and register a custom Irrlicht event receiver to allow selecting the
  // current driver model.
  ChDriverSelector selector(hmmwv_params.my_hmmwv.GetVehicle(), &driver_follower, &driver_gui);
  app.SetUserEventReceiver(&selector);

  // Finalize construction of visualization assets
  app.AssetBindAll();
  app.AssetUpdateAll();



  // -----------------
  // Initialize output
  // -----------------

  state_output = state_output || povray_output;

  if (state_output) {
      if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
          std::cout << "Error creating directory " << out_dir << std::endl;
          //return 1;
      }
  }

  if (povray_output) {
      if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
          std::cout << "Error creating directory " << pov_dir << std::endl;
        //  return 1;
      }
      driver_follower.ExportPathPovray(out_dir);
  }
  int sim_frame = 0;
  int render_frame = 0;

  while (app.GetDevice()->run()) {

    if (sim_frame == 0) {

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();
      /*
        if (povray_output) {
            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
        }

        if (state_output) {
            csv << time << steering_input << throttle_input << braking_input;
            csv << my_hmmwv.GetVehicle().GetVehicleSpeed();
            csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
            csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
            csv << std::endl;
        }*/

        render_frame++;
    }
    const ChVector<> pS = driver_follower.GetSteeringController().GetSentinelLocation();
    const ChVector<> pT = driver_follower.GetSteeringController().GetTargetLocation();
    hmmwv_params.ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
    hmmwv_params.ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));

    time = hmmwv_params.my_hmmwv.GetSystem()->GetChTime();
    // End simulation
    if (time >= t_end)
        break;
    hmmwv_params.throttle_input = selector.GetDriver()->GetThrottle();
    hmmwv_params.steering_input = selector.GetDriver()->GetSteering();
    hmmwv_params.braking_input = selector.GetDriver()->GetBraking();
    driver_follower.Synchronize(time);
    driver_gui.Synchronize(time);
    hmmwv_params.terrain.Synchronize(time);
    hmmwv_params.my_hmmwv.Synchronize(time, hmmwv_params.steering_input, hmmwv_params.braking_input, hmmwv_params.throttle_input, hmmwv_params.terrain);
    std::string msg1 = selector.UsingGUI() ? "GUI driver" : "Follower driver";
    app.Synchronize(msg1, hmmwv_params.steering_input, hmmwv_params.throttle_input, hmmwv_params.braking_input);

     // Advance simulation for one timestep for all modules
    double step = hmmwv_params.realtime_timer.SuggestSimulationStep(step_size);
    driver_follower.Advance(step);
    driver_gui.Advance(step);
    hmmwv_params.terrain.Advance(step);
    hmmwv_params.my_hmmwv.Advance(step);
    app.Advance(step);

    // Increment simulation frame number
    hmmwv_params.sim_frame++;
    hmmwv_params.callback_act=1;

    ChVector<> global_pos = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleCOMPos();//global location of chassis reference frame origin
    ChQuaternion<> global_orntn = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
    ChVector<> rot_dt = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetWvel_loc();//global orientation as quaternion
    ChVector<> global_velCOM = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dt();
    ChVector<> global_accCOM = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
    //ChVector<> euler_ang = global_orntn.Q_to_Rotv(); //convert to euler angles
    double q0 = global_orntn[0];
    double q1 = global_orntn[1];
    double q2 = global_orntn[2];
    double q3 = global_orntn[3];
    double yaw_val=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

    if (yaw_val<0){
      yaw_val=-yaw_val+PI/2;
    }
    else if (yaw_val>=0 && yaw_val<=PI/2){
      yaw_val=PI/2-yaw_val;
    }
    else if (yaw_val>PI/2 && yaw_val<=PI){
      yaw_val=5*PI/2-yaw_val;
    }
    //ChPacejkaTire<> slip_angle = GetSlipAngle()
    double slip_angle = hmmwv_params.my_hmmwv.GetTire(0)->GetLongitudinalSlip();


    data_out.t_chrono=time; //time in chrono simulation
    data_out.x_pos= global_pos[0] ;
    data_out.y_pos=global_pos[1];
    data_out.x_v= fabs(global_velCOM[0]); //speed measured at the origin of the chassis reference frame.
    data_out.y_v= global_velCOM[1];
    data_out.x_a= global_accCOM[0];
    data_out.yaw_curr=yaw_val; //in radians
    data_out.yaw_rate=-rot_dt[2];//(yaw_val-yaw_prev_val)/(time-t_prev_val); //in radians
    data_out.sa=slip_angle;
    data_out.thrt_in=hmmwv_params.throttle_input; //throttle input in the range [0,+1]
    data_out.brk_in=hmmwv_params.braking_input; //braking input in the range [0,+1]
    data_out.str_in=hmmwv_params.steering_input; //steeering input in the range [-1,+1]

    vehicleinfo_pub.publish(data_out);
    ros::spinOnce();
    }
}

// =============================================================================
int main(int argc, char* argv[]) {


char cwd[1024];
  if (getcwd(cwd, sizeof(cwd)) != NULL)
      fprintf(stdout, "Current working dir: %s\n", cwd);
  else
      perror("getcwd() error");
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath("opt/chrono/chrono/data/vehicle");
    // std::cout << GetChronoDataPath() << "\n"; check path of chrono data folder
    // Initialize ROS Chrono node and set node handle to n

    ros::init(argc, argv, "Chronode");
    ros::NodeHandle n;
    // Desired vehicle speed (m/s)
    double target_speed;
    n.getParam("v_des",target_speed);

    //Initial Position
    double x0, y0, z0, psi;
    bool gui_switch;
    n.getParam("x",x0);
    n.getParam("y",y0);
    n.getParam("z",z0);
    n.getParam("x",x0);
    n.getParam("x",x0);
    n.getParam("gui_status",gui_switch);

    // Initial vehicle location and orientation
    ChVector<> initLoc(x0, y0, z0);
    ChQuaternion<> initRot(cos(PI/4), 0, 0, sin(PI/4)); //initial yaw of pi/2

    ros::Publisher vehicleinfo_pub = n.advertise<ros_chrono_msgs::veh_status>("vehicleinfo", 1);
    //ros::Rate loop_rate(5);

    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetPacejkaParamfile(pacejka_tire_file);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    terrain.SetContactFrictionCoefficient(0.9f);
    terrain.SetContactRestitutionCoefficient(0.01f);
    terrain.SetContactMaterialProperties(2e7f, 0.3f);
    terrain.SetColor(ChColor(1, 1, 1));
    //terrain.SetTexture(chrono::vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.SetTexture(data_path+"terrain/textures/tile4.jpg", 200, 200);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

    // ----------------------
    // Create the Bezier path
    // ----------------------

    auto path = ChBezierCurve::read(path_file);


    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChVehicleIrrApp app(&my_hmmwv.GetVehicle(), &my_hmmwv.GetPowertrain(), L"Steering Controller Demo",
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

    /*
    ChPathFollowerDriver driver_follower(my_hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver_follower.GetSteeringController().SetLookAheadDistance(5);
    driver_follower.GetSteeringController().SetGains(0.5, 0, 0);
    driver_follower.GetSpeedController().SetGains(0.4, 0, 0);
    */
    ChPathFollowerDriver driver_follower(my_hmmwv.GetVehicle(), steering_controller_file,
                                         speed_controller_file, path, "my_path", target_speed);
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

    state_output = state_output || povray_output;

    if (state_output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        driver_follower.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;
    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);
    double debug_step_size = 1 / debug_fps;
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;
    int render_frame = 0;

    double callback_act;
    double throttle_input, steering_input, braking_input;
    parameters hmmwv_params{terrain,my_hmmwv,realtime_timer,sim_frame,callback_act,steering_input,throttle_input,braking_input,ballS,ballT};

    std::ofstream myfile1;
    myfile1.open(data_path+"paths/position.txt",std::ofstream::out | std::ofstream::trunc);
  //  bool new_path=0;
    while (app.GetDevice()->run()) {
      double time = my_hmmwv.GetSystem()->GetChTime();

        /*
        // Hack for acceleration-braking maneuver
        static bool braking = false;
        if (my_hmmwv.GetVehicle().GetVehicleSpeed() > target_speed)
            braking = true;
        if (braking) {
            throttle_input = 0;
            braking_input = 1;
        } else {
            throttle_input = 1;
            braking_input = 0;
        }
        */

      //     ros::Subscriber sub = n.subscribe<traj_gen_chrono::Control>("desired_ref", 1, &parameters::controlCallback, &hmmwv_params);

        // Render scene and output POV-Ray data
          if (sim_frame==-1){   //if (sim_frame % render_steps == 0) {

          app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
          app.DrawAll();
          app.EndScene();

        /*    if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
            }

            if (state_output) {
                csv << time << steering_input << throttle_input << braking_input;
                csv << my_hmmwv.GetVehicle().GetVehicleSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << std::endl;
            }*/

          render_frame++;
        }
/*
        // Debug logging
        if (debug_output && sim_frame % debug_steps == 0) {
            GetLog() << "driver acceleration:  " << acc_driver.x() << "  " << acc_driver.y() << "  " << acc_driver.z()
                     << "\n";
            GetLog() << "CG acceleration:      " << acc_CG.x() << "  " << acc_CG.y() << "  " << acc_CG.z() << "\n";
            GetLog() << "\n";
        }*/
        callback_act=0;
        ros_chrono_msgs::veh_status data_out;

        ros::Subscriber sub = n.subscribe<traj_gen_chrono::Control>("desired_ref", 10, boost::bind(controlCallback, _1, boost::ref(hmmwv_params),boost::ref(app),boost::ref(driver_gui),
        boost::ref(target_speed),boost::ref(time),boost::ref(data_out),boost::ref(vehicleinfo_pub)));
//        ros::Subscriber sub = n.subscribe<traj_gen_chrono::Control>("desired_ref", 10, boost::bind(controlCallback, _1, boost::ref(hmmwv_params),boost::ref(app)));
      //  if (sub){
        std::cout << callback_act;
      //  }

  //      app.SetPaused(1);
        path = ChBezierCurve::read(path_file);

        ChVector<> acc_CG = my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = my_hmmwv.GetVehicle().GetVehicleAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());


        // End simulation
        if (time >= t_end)
            break;


        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.

        const ChVector<> pS = driver_follower.GetSteeringController().GetSentinelLocation();
        const ChVector<> pT = driver_follower.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));
        ros::spin();
  /*      time = hmmwv_params.my_hmmwv.GetSystem()->GetChTime();
        hmmwv_params.throttle_input = selector.GetDriver()->GetThrottle();
        hmmwv_params.steering_input = selector.GetDriver()->GetSteering();
        hmmwv_params.braking_input = selector.GetDriver()->GetBraking();
        // Update modules (process inputs from other modules)
        driver_follower.Synchronize(time);
        driver_gui.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv_params.my_hmmwv.Synchronize(time, hmmwv_params.steering_input, hmmwv_params.braking_input, hmmwv_params.throttle_input, terrain);
        std::string msg = selector.UsingGUI() ? "GUI driver" : "Follower driver";
        app.Synchronize(msg, hmmwv_params.steering_input, hmmwv_params.throttle_input, hmmwv_params.braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver_follower.Advance(step);
        driver_gui.Advance(step);
        terrain.Advance(step);
        hmmwv_params.my_hmmwv.Advance(step);
        app.Advance(step);

        // Increment simulation frame number
        sim_frame++;*/

        ChVector<> global_pos = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleCOMPos();//global location of chassis reference frame origin
        ChQuaternion<> global_orntn = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
        ChVector<> rot_dt = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetWvel_loc();//global orientation as quaternion
        ChVector<> global_velCOM = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dt();
        ChVector<> global_accCOM = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
        //ChVector<> euler_ang = global_orntn.Q_to_Rotv(); //convert to euler angles
        double q0 = global_orntn[0];
        double q1 = global_orntn[1];
        double q2 = global_orntn[2];
        double q3 = global_orntn[3];
        double yaw_val=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

        if (yaw_val<0){
          yaw_val=-yaw_val+PI/2;
        }
        else if (yaw_val>=0 && yaw_val<=PI/2){
          yaw_val=PI/2-yaw_val;
        }
        else if (yaw_val>PI/2 && yaw_val<=PI){
          yaw_val=5*PI/2-yaw_val;
        }
        //ChPacejkaTire<> slip_angle = GetSlipAngle()
        double slip_angle = hmmwv_params.my_hmmwv.GetTire(0)->GetLongitudinalSlip();


        data_out.t_chrono=time; //time in chrono simulation
        data_out.x_pos= global_pos[0] ;
        data_out.y_pos=global_pos[1];
        data_out.x_v= fabs(global_velCOM[0]); //speed measured at the origin of the chassis reference frame.
        data_out.y_v= global_velCOM[1];
        data_out.x_a= global_accCOM[0];
        data_out.yaw_curr=yaw_val; //in radians
        data_out.yaw_rate=-rot_dt[2];//(yaw_val-yaw_prev_val)/(time-t_prev_val); //in radians
        data_out.sa=slip_angle;
        data_out.thrt_in=throttle_input; //throttle input in the range [0,+1]
        data_out.brk_in=braking_input; //braking input in the range [0,+1]
        data_out.str_in=steering_input; //steeering input in the range [-1,+1]

        vehicleinfo_pub.publish(data_out);
      //  loop_rate.sleep();


       myfile1 << ' ' << global_pos[0] << ' '<< global_pos[1]  <<' ' << global_pos[2]  << '\n';
    }

    if (state_output){
        csv.write_to_file(out_dir + "/state.out");
    }

//}

myfile1.close();

    return 0;
}
