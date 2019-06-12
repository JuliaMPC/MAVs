# Options
tire_model = "nonlinear"; # "none" || "linear" || "nonlinear"
nonlinear_acceleration_constraints = "on"
vertical_tire_force_constraints = "on"
#integration_scheme =  :backward_euler
integration_scheme =  :trapazoidal
obstacle_exists = "true"
plotting = "false"
steps = 50; # size of t_solve
execution_horizon =  0.5; # was 0.5
if (typeof(execution_horizon) != Float64) error("execution_horizon must be a Float!") end
N = 60;   # preview horizo
x_ref   = 200;
y_ref   = 100;
psi_ref = pi/2; # define goal
if obstacle_exists == "true"
  Q = 3; # number of obstacles
  A = [5,10,2]
  B = [5,10,2]
  s_x = [-2,0,0]
  s_y = [0,0,4]
  X0 = [200,180,200]
  Y0 = [34,75,30]
end
