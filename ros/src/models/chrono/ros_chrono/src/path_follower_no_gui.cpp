// C/C++ library
#include <iostream>
#include <cstdio>
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
#include "chrono/utils/ChFilters.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace alglib;
using namespace std;

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
// Contact method type (SMC, NSC) http://api.projectchrono.org/classchrono_1_1_ch_material_surface.html#ac59231413e1592095475c507cd7cfb95
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;

// Type of tire model (RIGID, LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

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
void waitForLoaded(ros::NodeHandle &node) {
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

	return ChVector<>(veh_x, veh_y, veh_z);
}

void plannerCallback(const nloptcontrol_planner::Trajectory::ConstPtr& control_msgs) {
	traj_t = control_msgs->t;
	traj_x = control_msgs->x;
	traj_y = control_msgs->y;
	traj_psi = control_msgs->psi;
	traj_sa = control_msgs->sa;
	traj_ux = control_msgs->ux;

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
	double min_dist_sqr = pow(pos_global[0] - traj_x[0], 2) + pow(pos_global[1] - traj_y[0], 2);

	for (int i = 1; i < traj_x.size() - 1; i++) {
		double dist_sqr = pow(pos_global[0] - traj_x[i], 2) + pow(pos_global[1] - traj_y[i], 2);
		if (dist_sqr < min_dist_sqr) {
			min_dist_sqr = dist_sqr;
			index = i;
		}
	}
	return index;
}

unsigned int getTargetPos(const ChVector<>& pos_global, const std::vector<double>& traj_x, const std::vector<double>& traj_y, const std::vector<double>& traj_ux,
	double& target_speed, double cur_speed, double& x_target, double& y_target, double& Kpp, double& Rmin, double& Rmax) {
	unsigned int index_closest = get_nearest_index(pos_global, traj_x, traj_y);
	double dist_target = std::min(Rmax, std::max(Rmin, Kpp*cur_speed)); //double dist_target = std::min(40.0, std::max(30.0, 1.5*cur_speed));
	double index_target = index_closest;
	double dist_last;
	double dist_cur = sqrt(pow(pos_global[0] - traj_x[index_closest], 2) + pow(pos_global[1] - traj_y[index_closest], 2));
	double div_min = DBL_MAX;
	for (int i = index_closest + 1; i < traj_x.size(); i++) {
		dist_last = dist_cur;
		dist_cur = sqrt(pow(pos_global[0] - traj_x[i], 2) + pow(pos_global[1] - traj_y[i], 2));
		if ((dist_last < dist_target) && (dist_cur > dist_target)) {
			// linear interpolation to find the target position
			index_target = i;
			double xa = traj_x[i - 1] - pos_global[0];
			double ya = traj_y[i - 1] - pos_global[1];
			double xb = traj_x[i] - pos_global[0];
			double yb = traj_y[i] - pos_global[1];
			double ta = (xa - xb)*(xa - xb) + (ya - yb)*(ya - yb);
			double tb = -2 * xa*xa + 2 * xa*xb - 2 * ya*ya + 2 * ya*yb;
			double tc = xa * xa + ya * ya;
			double insqrt = tb * tb - 4 * ta*(tc - dist_target * dist_target);
			if (insqrt < 0) {
				std::cout << "no solution" << std::endl;
				x_target = xb + pos_global[0]; //x_target = xb + pos_global[0];
				y_target = yb + pos_global[1]; // y_target = yb + pos_global[1];
				target_speed = traj_ux[i];
			}
			else {
				insqrt = sqrt(insqrt);
				double t1 = (-tb + insqrt) / (2.0*ta);
				double t2 = (-tb - insqrt) / (2.0*ta);
				if (t1 >= 0.0 && t1 <= 1.0) {
					x_target = (1 - t1)*xa + t1 * xb + pos_global[0];
					y_target = (1 - t1)*ya + t1 * yb + pos_global[1];
					target_speed = traj_ux[i - 1] * (1 - t1) + traj_ux[i] * t1;
				}
				else if (t2 >= 0.0 && t2 <= 1.0) {
					x_target = (1 - t2)*xa + t2 * xb + pos_global[0];
					y_target = (1 - t2)*ya + t2 * yb + pos_global[1];
					target_speed = traj_ux[i - 1] * (1 - t2) + traj_ux[i] * t2;
				}
				else {
					std::cout << "no right solution" << std::endl;
					x_target = xb + pos_global[0];
					y_target = yb + pos_global[1];
					target_speed = traj_ux[i];
				}
				double temp = sqrt((x_target - pos_global[0])*(x_target - pos_global[0]) + (y_target - pos_global[1])*(y_target - pos_global[1]));
			}
			break;
		}
	}
	return index_target;
}

double getTargetAngle(const ChVector<>& pos_global, const std::vector<double>& traj_x, const std::vector<double>& traj_y,
	const std::vector<double>& traj_ux, double& target_speed, double cur_speed, double l, double yaw, double& Kpp, double& Rmin, double& Rmax) {
	double x_target;
	double y_target;
	unsigned int index_target = getTargetPos(pos_global, traj_x, traj_y, traj_ux, target_speed, cur_speed, x_target, y_target, Kpp, Rmin, Rmax);
	double x_target_COM = cos(yaw)*(x_target - pos_global[0]) + sin(yaw)*(y_target - pos_global[1]);
	double y_target_COM = -sin(yaw)*(x_target - pos_global[0]) + cos(yaw)*(y_target - pos_global[1]);
	double angle_target = atan(2 * y_target_COM*l / (x_target_COM*x_target_COM + y_target_COM * y_target_COM));
	return angle_target;
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
	std::string chrono_namespace;

	node.getParam("system/planner", planner_namespace);
	node.getParam("system/chrono/namespace", chrono_namespace);

	ros::Subscriber planner_sub = node.subscribe(planner_namespace + "/control", 100, plannerCallback);
	ros::Publisher state_pub = node.advertise<mavs_msgs::state>("/state", 5);
	ros::Publisher control_pub = node.advertise<mavs_msgs::control>("/control", 5);

	mavs_msgs::state state_data;
	mavs_msgs::control control_data;

	bool debug;
	node.getParam("system/chrono/flags/debug", debug);

	// Define variables for ROS parameters server
	double step_size;
	double tire_step_size;
	double x0, y0, z0; // Initial global position
	double roll0, pitch0, yaw0, ux0, ax0, sa0; // Initial global orientation

	// Get parameters from ROS Parameter Server
	node.getParam("system/params/step_size", step_size); // ROS loop rate and Chrono step size
	node.getParam("system/params/tire_step_size", tire_step_size);

	node.getParam("case/actual/X0/x", x0); // initial x
	node.getParam("case/actual/X0/yVal", y0); // initial y
	node.getParam("case/actual/X0/z", z0); // initial z
	//node.getParam("case/actual/X0/v", v); // lateral velocity
	//node.getParam("case/actual/X0/r", r); // yaw rate
	node.getParam("case/actual/X0/theta", pitch0); // initial pitch
	node.getParam("case/actual/X0/phi", roll0); // initial roll
	node.getParam("case/actual/X0/psi", yaw0); // initial yaw angle
	node.getParam("case/actual/X0/sa", sa0);
	node.getParam("case/actual/X0/ux", ux0);
	node.getParam("case/actual/X0/ax", ax0);


	// Load chrono vehicle_params
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
	node.getParam("vehicle/chrono/controller/time_shift", time_shift);

	// ---------------------
	// controllers
	// ---------------------
	// velocity PID controller
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

  // pure pursuit controller
	double Kpp, Rmin, Rmax;
	node.getParam("vehicle/chrono/controller/Kpp", Kpp);
	node.getParam("vehicle/chrono/controller/Rmin", Rmin);
	node.getParam("vehicle/chrono/controller/Rmax", Rmax);

	// loop rate
	ros::Rate loop_rate(1.0 / step_size);

	// initial vehicle location and orientation
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
	ChQuaternion<> initRot(q0_0, q0_1, q0_2, q0_3);

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
																 ChVector<>(terrainLength, terrainWidth, 10),
																 true, 20, true);
	patch->SetContactFrictionCoefficient(frictCoeff);
	patch->SetContactRestitutionCoefficient(restCoeff);
	patch->SetContactMaterialProperties(2e7f, 0.3f);
	patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
	patch->SetTexture(data_path + "terrain/textures/tile4.jpg", 200, 200);
	terrain.Initialize();

	// ---------------
	// Simulation loop
	// ---------------
	// Vehicle steering angle
	double long_velocity = 0.0;
	ChVector<> VehicleCOMPos = my_hmmwv.GetVehicle().GetVehicleCOMPos();
	ChQuaternion<> VehicleRot = my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
	// long_velocity = my_hmmwv.GetVehicle().GetVehicleSpeedCOM();
	double yaw_angle = VehicleRot.Q_to_Euler123()[2];

	// collect controller output data from modules (for inter-module communication)
	double throttle_input = 0;
	double steering_input = 0;
	double braking_input = 0;

	// calculate control error
	std::vector<double> x_cal(3, VehicleCOMPos[0]);
	std::vector<double> y_cal(3, VehicleCOMPos[1]);
	double steering_angle;

	// find lr and lf
	ChVector<> front_pos1 = my_hmmwv.GetVehicle().GetWheelPos(0);
	ChVector<> front_pos2 = my_hmmwv.GetVehicle().GetWheelPos(1);
	ChVector<> rear_pos1 = my_hmmwv.GetVehicle().GetWheelPos(2);
	ChVector<> rear_pos2 = my_hmmwv.GetVehicle().GetWheelPos(3);
	double front_x = (front_pos1[0] + front_pos2[0]) / 2.;
	double front_y = (front_pos1[1] + front_pos2[1]) / 2.;
	double rear_x = (rear_pos1[0] + rear_pos2[0]) / 2.;
	double rear_y = (rear_pos1[1] + rear_pos2[1]) / 2.;
	double front2rear = sqrt(pow(rear_x - front_x, 2) + pow(rear_y - front_y, 2));

	// wait system loaded
	waitForLoaded(node);
	while (ros::ok()) {
		// get chrono time
		double chrono_time = my_hmmwv.GetSystem()->GetChTime();
		// update time shift based off of current solve time
		node.getParam("vehicle/chrono/controller/time_shift", time_shift);

		// --------------------------
		// interpolation using ALGLIB
		// --------------------------
		if (traj_t.size() > 1) {
				real_1d_array t_arr, sa_arr, ux_arr;
				t_arr.setcontent(traj_t.size(), &(traj_t[0]));

				// speed
				ux_arr.setcontent(traj_ux.size(), &(traj_ux[0]));
				spline1dinterpolant s_ux;
				spline1dbuildlinear(t_arr, ux_arr, s_ux);
				traj_ux_interp = spline1dcalc(s_ux, chrono_time + time_shift);
				double ux_err = traj_ux_interp - long_velocity;
				double vel_controller_output = vel_controller.control(ux_err);

				if (debug) {
					printf("ux_err: %lf\ntraj_ux_interp: %lf\nlong_velocity: %lf\nvel_controller_output: %lf\n", ux_err, traj_ux_interp,long_velocity,vel_controller_output);
				}

				if (vel_controller_output > 0) {
					throttle_input = vel_controller_output;
					braking_input = 0;
				}
				else {
					throttle_input = 0;
					braking_input = -vel_controller_output;
				}

				// steering
				/*
				sa_arr.setcontent(traj_sa.size(), &(traj_sa[0]));
				spline1dinterpolant s_sa;
				spline1dbuildcubic(t_arr, sa_arr, s_sa);
				traj_sa_interp = spline1dcalc(s_sa, chrono_time + time_shift);
				*/
				front_pos1 = my_hmmwv.GetVehicle().GetWheelPos(0);
				front_pos2 = my_hmmwv.GetVehicle().GetWheelPos(1);
				rear_pos1 = my_hmmwv.GetVehicle().GetWheelPos(2);
				rear_pos2 = my_hmmwv.GetVehicle().GetWheelPos(3);
				front_x = (front_pos1[0] + front_pos2[0]) / 2.;
				front_y = (front_pos1[1] + front_pos2[1]) / 2.;
				rear_x = (rear_pos1[0] + rear_pos2[0]) / 2.;
				rear_y = (rear_pos1[1] + rear_pos2[1]) / 2.;
				front2rear = sqrt(pow(rear_x - front_x, 2) + pow(rear_y - front_y, 2));
				traj_sa_interp = 1.0*getTargetAngle(VehicleCOMPos, traj_x, traj_y, traj_ux, traj_ux_interp, long_velocity, front2rear, yaw_angle, Kpp, Rmin, Rmax);

				double alpha = 1.0; 				// low pass filter on steering
				steering_input = (1.0 - alpha) * steering_input + alpha * traj_sa_interp / maximum_steering_angle;
				steering_input = std::max(-1.0, std::min(1.0, steering_input));
				steering_angle = steering_input * maximum_steering_angle; // steering angle (rad)
		}
		else if (traj_x.empty() || traj_x.size() == 1) {
				braking_input = 0;
				throttle_input = 0;
				steering_angle = 0;
				steering_input = 0;
		}

		// Update modules (process inputs from other modules)
		terrain.Synchronize(chrono_time);
		my_hmmwv.Synchronize(chrono_time, steering_input, braking_input, throttle_input, terrain);

		// Advance simulation for one timestep for all modules
		terrain.Advance(step_size);
		my_hmmwv.Advance(step_size);

		VehicleCOMPos = my_hmmwv.GetVehicle().GetVehicleCOMPos();
		VehicleRot = my_hmmwv.GetVehicle().GetVehicleRot();//global orientation as quaternion
		ChVector<> rot_dt = my_hmmwv.GetChassisBody()->GetWvel_loc(); // actual angular speed (expressed in local coords)
	  ChVector<> VehiclePos = my_hmmwv.GetVehicle().GetVehiclePos(); // gloabal vehicle frame origin location

		// Compute yaw angle
		yaw_angle = VehicleRot.Q_to_Euler123()[2];
		ChVector<> ORI2COM = global2veh(yaw_angle, VehicleCOMPos - VehiclePos);
		ChVector<> VehicleRot_dt = my_hmmwv.GetChassisBody()->GetWvel_loc(); // actual angular speed (expressed in local coords)
		ChVector<> VehicleCOMVel_global = my_hmmwv.GetVehicle().GetVehiclePointVelocity(ORI2COM); // vehicle COM velocity (m/s)
		ChVector<> VehicleCOMAcc = my_hmmwv.GetVehicle().GetVehicleAcceleration(ORI2COM); // vehicle COM acceleration (m/s^2)
		VehicleCOMAcc[0] = std::max(-1.5, std::min(1.5, VehicleCOMAcc[0])); // let vehicle acceleration bounded in [-1.5, 1.5] (temporary solution)
		ChVector<> VehicleCOMVel = global2veh(yaw_angle, VehicleCOMVel_global);

		if (debug) {
			std::cout << " VehiclePos[0] = " <<  VehiclePos[0] << std::endl;
			std::cout << " VehiclePos[1] = " <<  VehiclePos[1] << std::endl;
			std::cout << " VehiclePos[2] = " <<  VehiclePos[2] << std::endl;
			std::cout << " VehicleCOMPos[0] = " <<  VehicleCOMPos[0] << std::endl;
			std::cout << " VehicleCOMPos[1] = " <<  VehicleCOMPos[1] << std::endl;
			std::cout << " VehicleCOMPos[2] = " <<  VehicleCOMPos[2] << std::endl;
		}

		// Get vertical tire force
		std::vector<double> TireForceVertical;
		for (int i = 0; i < 4; i++) {
			ChVector<> TireForce = my_hmmwv.GetTire(i)->ReportTireForce(&terrain).force;
			TireForceVertical.push_back(TireForce[2]);
		}

		// Update vehicle state and control parameters
		node.setParam("/state/t", chrono_time);
		node.setParam("/state/x", VehicleCOMPos[0]);       // global x position (m)
		node.setParam("/state/y", VehicleCOMPos[1]);       // global y position (m)
		node.setParam("/state/z", VehicleCOMPos[2]);       // global z position (m)
		node.setParam("/state/v", VehicleCOMVel[1]);       // lateral velocity (m/s)
		node.setParam("/state/r", VehicleRot_dt[2]);       // yaw rate (rad/s)
		node.setParam("/state/psi", yaw_angle);            // global heading angle (yaw angle) (rad)
		node.setParam("/state/sa", steering_angle);        // steering angle at the tire (rad)
		node.setParam("/state/ux", VehicleCOMVel[0]);      // longitudinal velocity  (vehicle frame) (m/s)
		node.setParam("/state/ax", VehicleCOMAcc[0]);      // longitudinal acceleration (vehicle frame) (m/s^2)
		node.setParam("/state/vtfl", TireForceVertical[0]);
		node.setParam("/state/vtfr", TireForceVertical[1]);
		node.setParam("/state/vtrl", TireForceVertical[2]);
		node.setParam("/state/vtrr", TireForceVertical[3]);
		node.setParam("/control/thr", throttle_input);
		node.setParam("/control/brk", braking_input);
		node.setParam("/control/str", steering_input);

    long_velocity = VehicleCOMVel[0];

		// Update state and control messages
		state_data.t = chrono_time; // time in chrono simulation
		state_data.x = VehicleCOMPos[0];
		state_data.y = VehicleCOMPos[1];
		state_data.z = VehicleCOMPos[2];
		state_data.ux = VehicleCOMVel[0];
		state_data.v = VehicleCOMVel[1];
		state_data.ax = VehicleCOMAcc[0];
		state_data.psi = yaw_angle;      // yaw angle (rad)
		state_data.r = VehicleRot_dt[2]; // yaw rate (rad/s)
		state_data.sa = steering_angle;  // steering angle at the tire (rad)
		state_data.tireF_fl = TireForceVertical[0];
		state_data.tireF_fr = TireForceVertical[1];
		state_data.tireF_rl = TireForceVertical[2];
		state_data.tireF_rr = TireForceVertical[3];
		control_data.t = chrono_time;
		control_data.thrt_in = throttle_input;  // throttle input in the range [0,+1]
		control_data.brk_in = braking_input;    // braking input in the range [0,+1]
		control_data.str_in = steering_input;   // steeering input in the range [-1,+1]
		state_pub.publish(state_data);
		control_pub.publish(control_data);

		ros::spinOnce();
	}

	return 0;
}
