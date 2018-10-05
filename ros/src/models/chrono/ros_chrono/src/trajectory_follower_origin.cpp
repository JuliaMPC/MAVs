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
#include "nloptcontrol_planner/Control.h"
#include <unistd.h>
#include <interpolation.h>
#include <PID.h>
#include <vector>
//#include "tf/tf.h"
#include <sstream>
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include <math.h>
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

#define PI 3.1415926535
//using veh_status.msg
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using namespace alglib;

using namespace rapidjson;
// =============================================================================
// Problem parameters
// Main Data Path
//std::string data_path("/home/shreyas/.julia/v0.6/MAVs/catkin_ws/data/vehicle/");
std::string data_path("../../../src/models/chrono/ros_chrono/src/data/vehicle/");
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

// Input file names for the path-follower driver model
std::string steering_controller_file(data_path+"generic/driver/SteeringController.json");
//std::string steering_controller_file("generic/driver/SteeringController.json");
std::string speed_controller_file(data_path+"generic/driver/SpeedController.json");
//std::string speed_controller_file("generic/driver/SpeedController.json");
// std::string path_file("paths/straight.txt");
std::string path_file(data_path+"paths/my_path.txt");

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 500.0;  // size in X direction
double terrainWidth = 500.0;   // size in Y direction

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 1e-2;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

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

// In src/data/vehicle/hmmwv/steering/HMMWV_PitmanArm.json revolution joint
double speed = 0.0;
double maximum_steering_angle = 50.0;
// Controller output
double target_steering_interp = 0.0;
double target_speed_interp = 0.0;
double controller_output = 0.0;
PID controller;

// =============================================================================

// Custom Irrlicht event receiver for selecting current driver model.
class ChDriverSelector : public irr::IEventReceiver {
  public:
    ChDriverSelector(const ChVehicle& vehicle, ChIrrGuiDriver* driver_gui)
        : m_vehicle(vehicle),
          m_driver_gui(driver_gui),
          m_driver(m_driver_gui),
          m_using_gui(true) {}

    ChDriver* GetDriver() { return m_driver; }
    bool UsingGUI() const { return m_using_gui; }

    virtual bool OnEvent(const irr::SEvent& event) {
        return false;
    }

  private:
    bool m_using_gui;
    const ChVehicle& m_vehicle;
    ChIrrGuiDriver* m_driver_gui;
    ChDriver* m_driver;
};

struct parameters
{
    GranularTerrain terrain;
    HMMWV_Reduced my_hmmwv;
    ChRealtimeStepTimer realtime_timer;
    int sim_frame;
    double steering_input;
    double throttle_input;
    double braking_input;
    std::vector<double> x_traj_curr;
    std::vector<double> y_traj_curr;
    std::vector<double> x_traj_prev;
    std::vector<double> y_traj_prev;
    double target_speed;
    int render_steps;
    int render_frame;
} ;

void waitForLoaded(ros::NodeHandle &n){
    bool planner_init = false;
    std::string planner_namespace;
    n.getParam("system/planner",planner_namespace);
    n.getParam("system/"+planner_namespace+"/flags/initialized",planner_init);
    while(!planner_init){
        usleep(500); // < my question
        n.getParam("system/"+planner_namespace+"/flags/initialized",planner_init);

    }
    usleep(500); //sleep another 500ms to ensure everything is loaded.
    //continue on here
}

void controlCallback(const nloptcontrol_planner::Control::ConstPtr& control_msg){
  double time = ros::Time::now().toSec();
  std::vector<double> traj_t = control_msg->t;
  std::vector<double> traj_sa = control_msg->sa;
  std::vector<double> traj_vx = control_msg->vx;
  std::cout << "Current time: "<< time << " ";

  std::cout << "Time trajectory: " << " ";
  for(int i = 0; i < traj_t.size(); i++)
    std::cout << traj_t[i] << " ";
    std::cout << std::endl;

  std::cout << "Speed trajectory: " << " ";
  for(int i = 0; i < traj_vx.size(); i++)
      std::cout << traj_vx[i] << " ";
  std::cout << std::endl;

  std::cout << "Steering trajectory: " << " ";
  for(int i = 0; i < traj_sa.size(); i++)
      std::cout << traj_sa[i] << " ";
  std::cout << std::endl;

  real_1d_array t_array;
  real_1d_array steering_array;
  real_1d_array speed_array;

  t_array.setcontent(traj_t);
  steering_array.setcontent(traj_sa);
  speed_array.setcontent(traj_vx);
  spline1dinterpolant s;
  ae_int_t natural_bound_type = 2;
  spline1dbuildcubic(t_array, speed_array, s);
  target_speed_interp = spline1dcalc(s, time);
  spline1dbuildcubic(t_array, steering_array, s);
  target_steering_interp = spline1dcalc(s, time);

  double speed_error = target_speed_interp - speed;
  controller_output = controller.control(speed_error);

  target_steering_interp = target_steering_interp / PI * 180.0 / maximum_steering_angle;
  // steering = steering * (-1);
  // std::cout << "Target_speed: " << target_speed_interp << " Actual speed: " << speed << std::endl;
  std::cout << "Target_steering: " << target_steering_interp << " Actual steering: " << target_steering_interp << std::endl;
  std::cout << std::endl;
}

// =============================================================================
int main(int argc, char* argv[]) {

  unsigned int microseconds = 10000000;
  std::cout << "Sleeping for ten sec..." << std::endl;
  usleep(microseconds);

  char cwd[1024];
  if (getcwd(cwd, sizeof(cwd)) != NULL)
      fprintf(stdout, "Current working dir: %s\n", cwd);
  else
      perror("getcwd() error");
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // read maximum_steering_angle from json
    std::string filename("PitmanArm.json");
    FILE* fp = fopen(filename.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);
    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    maximum_steering_angle = d["Revolute Joint"]["Maximum Angle"].GetDouble();

    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath("opt/chrono/chrono/data/vehicle");
    // std::cout << GetChronoDataPath() << "\n"; check path of chrono data folder
    // Initialize ROS Chrono node and set node handle to n

    ros::init(argc, argv, "Traj_follower");
    ros::NodeHandle n;
    n.setParam("system/chrono/flags/initialized",true);
    n.getParam("system/params/step_size",step_size);

    bool planner_init;
  //  bool planner_init2;

    std::string planner_namespace;
    n.getParam("system/planner",planner_namespace);
    n.getParam("system/"+planner_namespace+"/flags/initialized",planner_init);
    std::cout << "The planner state:" << planner_init << std::endl;
    ros::Subscriber sub = n.subscribe(planner_namespace+"/control", 100, controlCallback);

    //planner_init2=planner_init1;
  //    n.getParam("system/"+planner_namespace+"/flags/initialized",planner_init2);

      if(!planner_init){
        waitForLoaded(n);
    }
  //std::string planner_init;

    n.getParam("system/"+planner_namespace+"/flags/initialized",planner_init);

    // Desired vehicle speed (m/s)
    double target_speed;
    n.getParam("hmmwv_chrono/X0/v_des",target_speed);

    //Initial Position
    double x0, y0, z0, yaw0, pitch0, roll0;
    bool gui_switch;
    n.getParam("case/actual/X0/x",x0);
    n.getParam("case/actual/X0/yVal",y0);
    n.getParam("hmmwv_chrono/X0/z",z0);
    n.getParam("case/actual/X0/psi",yaw0);
    n.getParam("hmmwv_chrono/X0/theta",pitch0);
    n.getParam("hmmwv_chrono/X0/phi",roll0);
    n.getParam("system/chrono/flags/gui",gui_switch);

    bool goal_attained = false;
    bool running = true;

    ChVector<> initLoc(x0, y0, z0);
//    ChQuaternion<> initRot(q[0],q[1],q[2],q[3]);

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
    //ChQuaternion<> initRot(cos(PI/4), 0, 0, sin(PI/4)); //initial yaw of pi/2

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
    float frict_coeff, rest_coeff;
    n.getParam("vehicle/common/frict_coeff",frict_coeff);
    n.getParam("vehicle/common/rest_coeff",rest_coeff);

    GranularTerrain terrain(my_hmmwv.GetSystem());
    my_hmmwv.GetVehicle().GetWheel(0)->SetContactFrictionCoefficient(frict_coeff);
    my_hmmwv.GetVehicle().GetWheel(0)->SetContactRestitutionCoefficient(rest_coeff);

    //terrain.SetContactFrictionCoefficient(0.9f);
    //terrain.SetContactRestitutionCoefficient(0.01f);
    terrain.SetContactMaterialProperties(2e7f, 0.3f);
    terrain.SetColor(ChColor(1, 1, 1));
    //terrain.SetTexture(chrono::vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.SetTexture(data_path+"terrain/textures/tile4.jpg", 200, 200);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth);
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
    app.SetTryRealtime(true);

    // -------------------------
    // Create the driver systems
    // -------------------------

    // Create both a GUI driver and a path-follower and allow switching between them
    ChIrrGuiDriver driver_gui(app);
    driver_gui.Initialize();

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

    double throttle_input, steering_input, braking_input;
    std::vector<double> x_traj_curr, y_traj_curr,x_traj_prev,y_traj_prev; //Initialize xy trajectory vectors
    parameters hmmwv_params{terrain,my_hmmwv,realtime_timer,sim_frame,steering_input,throttle_input,braking_input,x_traj_curr,y_traj_curr,x_traj_prev,y_traj_prev,target_speed,render_steps,render_frame};
    //Load xy parameters for the first timestep
  //  std::string planner_namespace;
  //  n.getParam("system/planner",planner_namespace);

    // ----------------------
    // Create the Bezier path
    // ----------------------

    // Create and register a custom Irrlicht event receiver to allow selecting the
    // current driver model.
    ChDriverSelector selector(my_hmmwv.GetVehicle(), &driver_gui);
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

/*
    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);
*/
    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    std::ofstream myfile1;
    myfile1.open(data_path+"paths/position.txt",std::ofstream::out | std::ofstream::trunc);
    //get mass and moment of inertia about z axis
    n.setParam("vehicle/common/m",my_hmmwv.GetVehicle().GetVehicleMass());
    const ChMatrix33<> inertia_mtx= my_hmmwv.GetChassisBody()->GetInertia();
    double Izz=inertia_mtx.GetElement(2,2);
    n.setParam("vehicle/common/Izz",Izz);
    // get distance to front and rear axles
    // enum chrono::vehicle::VehicleSide LEFT;
    // enum chrono::vehicle::VehicleSide RIGHT;
    ChVector<> veh_com= my_hmmwv.GetVehicle().GetVehicleCOMPos();
    ChVector<> la_pos=my_hmmwv.GetVehicle().GetSuspension(0)->GetSpindlePos(chrono::vehicle::VehicleSide::RIGHT);
    ChVector<> lb_pos=my_hmmwv.GetVehicle().GetSuspension(0)->GetSpindlePos(chrono::vehicle::VehicleSide::LEFT);
    double la_length, lb_length;
    ChVector<> la_diff;
    la_diff.Sub(veh_com,la_pos);
    ChVector<> lb_diff;
    lb_diff.Sub(veh_com,lb_pos);
    la_length=la_diff.Length();
    lb_length=lb_diff.Length();
    n.setParam("vehicle/common/la",la_length);
    n.setParam("vehicle/common/lb",lb_length);

    // get friction and restitution coefficients
  //  float frict_coeff, rest_coeff;
    frict_coeff = my_hmmwv.GetVehicle().GetWheel(0)->GetCoefficientFriction();
    rest_coeff = my_hmmwv.GetVehicle().GetWheel(0)->GetCoefficientRestitution();
    n.setParam("vehicle/common/frict_coeff",frict_coeff);
    n.setParam("vehicle/common/rest_coeff",rest_coeff);

    // set up the controller
    controller.set_PID(0.3, 0.04, 0.001);
    controller.set_step_size(step_size);
    controller.set_output_limit(-1.0, 1.0);
    controller.initialize();

    std::cout<< "Go into the loop..." << std::endl;

    while(n.ok()){
      while (running) {
        if(gui_switch)  app.GetDevice()->run();
        hmmwv_params.target_speed = target_speed_interp;

        // Hack for acceleration-braking maneuver
        /* // on-off controller
        bool braking = false;
        if (my_hmmwv.GetVehicle().GetVehicleSpeed() > hmmwv_params.target_speed)
            braking = true;
        if (braking) {
            hmmwv_params.throttle_input = 0;
            hmmwv_params.braking_input = 1;
        }
        else {
            hmmwv_params.throttle_input = 1;
            hmmwv_params.braking_input = 0;
        }
        */
        // pid controller
        if (controller_output > 0){
          hmmwv_params.throttle_input = controller_output;
          hmmwv_params.braking_input = 0;
        }
        else{
          hmmwv_params.throttle_input = 0;
          hmmwv_params.braking_input = -controller_output;
        }

        // Hack for steering maneuver
        hmmwv_params.steering_input = target_steering_interp;

        //     ros::Subscriber sub = n.subscribe<traj_gen_chrono::Control>("desired_ref", 1, &parameters::controlCallback, &hmmwv_params);

          // Render scene and output POV-Ray data
        if (hmmwv_params.sim_frame % hmmwv_params.render_steps == 0 && gui_switch) {

            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();


            hmmwv_params.render_frame++;
          }

          ros_chrono_msgs::veh_status data_out;
  /*
          ChVector<> acc_CG = my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
          ChVector<> acc_driver = my_hmmwv.GetVehicle().GetVehicleAcceleration(driver_pos);
          double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
          double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
          double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
          double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());*/
          double time = hmmwv_params.my_hmmwv.GetSystem()->GetChTime();

          // End simulation
          if (time >= t_end)
              break;

          // hmmwv_params.throttle_input = selector_p->GetDriver()->GetThrottle();
          // hmmwv_params.steering_input = selector_p->GetDriver()->GetSteering();
          // hmmwv_params.braking_input = selector_p->GetDriver()->GetBraking();
          // Update modules (process inputs from other modules)

          // std::cout << "Chrono steering input: " << hmmwv_params.steering_input <<std::endl;

          driver_gui.Synchronize(time);
          hmmwv_params.terrain.Synchronize(time);
          hmmwv_params.my_hmmwv.Synchronize(time, hmmwv_params.steering_input, hmmwv_params.braking_input, hmmwv_params.throttle_input, hmmwv_params.terrain);
          std::string msg = selector.UsingGUI() ? "GUI driver" : "Follower driver";
          if(gui_switch) app.Synchronize(msg, hmmwv_params.steering_input, hmmwv_params.throttle_input, hmmwv_params.braking_input);

          // Advance simulation for one timestep for all modules
          double step = hmmwv_params.realtime_timer.SuggestSimulationStep(step_size);
          driver_gui.Advance(step);
          hmmwv_params.terrain.Advance(step);
          hmmwv_params.my_hmmwv.Advance(step);
          if(gui_switch) app.Advance(step);

          // Increment simulation frame number
          hmmwv_params.sim_frame++;

          ChVector<> global_pos = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleCOMPos();//global location of chassis reference frame origin
          ChQuaternion<> global_orntn = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
          ChVector<> rot_dt = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetWvel_loc();//global orientation as quaternion
          ChVector<> global_velCOM = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dt();
          ChVector<> global_accCOM = hmmwv_params.my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
          //ChVector<> euler_ang = global_orntn.Q_to_Rotv(); //convert to euler angles
          //ChPacejkaTire<> slip_angle = GetSlipAngle()
          double slip_angle = hmmwv_params.my_hmmwv.GetTire(0)->GetLongitudinalSlip();
          speed = hmmwv_params.my_hmmwv.GetVehicle().GetVehicleSpeedCOM();

          double q0 = global_orntn[0];
          double q1 = global_orntn[1];
          double q2 = global_orntn[2];
          double q3 = global_orntn[3];

          double yaw_val=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
          double theta_val=asin(2*(q0*q2-q3*q1));
          double phi_val= atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));

          n.setParam("state/chrono/state/t",time); //time in chrono simulation
          n.setParam("state/chrono/state/x", global_pos[0]) ;
          n.setParam("state/chrono/state/yVal",global_pos[1]);
          n.setParam("state/chrono/state/ux",fabs(global_velCOM[0])); //speed measured at the origin of the chassis reference frame.
          n.setParam("state/chrono/state/v", global_velCOM[1]);
          n.setParam("state/chrono/state/ax", global_accCOM[0]);
          n.setParam("state/chrono/state/psi",yaw_val); //in radians
          n.setParam("state/chrono/state/theta",theta_val); //in radians
          n.setParam("state/chrono/state/phi",phi_val); //in radians
          n.setParam("state/chrono/state/r",-rot_dt[2]);//yaw rate
          n.setParam("state/chrono/state/sa",slip_angle); //slip angle
          n.setParam("state/chrono/control/thr",hmmwv_params.throttle_input); //throttle input in the range [0,+1]
          n.setParam("state/chrono/control/brk",hmmwv_params.braking_input); //braking input in the range [0,+1]
          n.setParam("state/chrono/control/str",hmmwv_params.steering_input); //steeering input in the range [-1,+1]

          data_out.t_chrono=time; //time in chrono simulation
          data_out.x_pos= global_pos[0];
          data_out.y_pos=global_pos[1];
          data_out.x_v= fabs(global_velCOM[0]); //speed measured at the origin of the chassis reference frame.
          data_out.y_v= global_velCOM[1];
          data_out.x_a= global_accCOM[0];
          data_out.yaw_curr=yaw_val; //in radians
          data_out.yaw_rate=-rot_dt[2];//yaw rate
          data_out.sa=slip_angle; //slip angle
          data_out.thrt_in=hmmwv_params.throttle_input; //throttle input in the range [0,+1]
          data_out.brk_in=hmmwv_params.braking_input; //braking input in the range [0,+1]
          data_out.str_in=hmmwv_params.steering_input; //steeering input in the range [-1,+1]

          vehicleinfo_pub.publish(data_out);
        //  loop_rate.sleep();

         myfile1 << ' ' << global_pos[0] << ' '<< global_pos[1]  <<' ' << global_pos[2]  << '\n';
         ros::spinOnce();
         n.getParam("system/" + planner_namespace + "/flags/goal_attained",goal_attained);
         if(goal_attained){
           n.setParam("system/goal_attained","true");
           n.setParam("system/chrono/flags/running","false");
           running = false;
         }
      }
    }
/*
    if (state_output){
        csv.write_to_file(out_dir + "/state.out");
    }*/
myfile1.close();

    return 0;
}
