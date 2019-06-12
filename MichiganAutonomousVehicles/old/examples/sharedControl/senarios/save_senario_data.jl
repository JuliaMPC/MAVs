# save senario data
tire_model_data = DataFrame(ID = 1,tire_model=tire_model);
nonlinear_acceleration_constraints_data = DataFrame(ID = 1,nonlinear_acceleration_constraints=nonlinear_acceleration_constraints);
vertical_tire_force_constraints_data = DataFrame(ID = 1,vertical_tire_force_constraints=vertical_tire_force_constraints);
N_data = DataFrame(ID = 1,N =N );
execution_horizon_data = DataFrame(ID = 1,execution_horizon=execution_horizon);
solver_def_data = DataFrame(ID = 1, solver_def=solver_def[RR]);
integration_scheme_data = DataFrame(ID=1,integration_scheme=integration_scheme)
x_ref_data = DataFrame(ID=1, x_ref = x_ref)
y_ref_data = DataFrame(ID=1, y_ref = y_ref)
psi_ref_data = DataFrame(ID=1, psi_ref = psi_ref)

s_data = join(tire_model_data,nonlinear_acceleration_constraints_data,on=:ID);
s_data = join(s_data,vertical_tire_force_constraints_data,on=:ID);
s_data = join(s_data,N_data,on=:ID);
s_data = join(s_data,execution_horizon_data,on=:ID);
s_data = join(s_data,solver_def_data,on=:ID)
s_data = join(s_data,integration_scheme_data,on=:ID)
s_data = join(s_data,x_ref_data,on=:ID)
s_data = join(s_data,y_ref_data,on=:ID)
s_data = join(s_data,psi_ref_data,on=:ID)

if obstacle_exists == "true";
  A_data = DataFrame(ID = 1:Q, A = A);
  B_data = DataFrame(ID = 1:Q, B = B);
  s_x_data = DataFrame(ID = 1:Q, s_x = s_x);
  s_y_data = DataFrame(ID = 1:Q, s_y = s_y);
  X0_data = DataFrame(ID = 1:Q, X0 = X0);
  Y0_data = DataFrame(ID = 1:Q, Y0 = Y0);

  # build a single table
  obs_data = join(A_data,B_data, on = :ID);
  obs_data = join(obs_data,s_x_data,on =:ID);
  obs_data = join(obs_data,s_y_data,on =:ID);
  obs_data = join(obs_data,X0_data,on =:ID);
  obs_data = join(obs_data,Y0_data,on=:ID);
end
