using RobotOS, YAML, VehicleModels, NLOptControl, VehicleModels, MAVs


# To find fieldnames
r = ()
for i in 1:length(fieldnames(p))
      r = (r...,fieldnames(p)[i])
end
#la,lb,x_min,x_max,y_min,y_max,psi_min,psi_max,u_min,u_max,ax_min,ax_max,sa_min,sa_max,x0_,y0_,psi0_,u0_,ax0_
# KB
@unpack la,lb,x_min,x_max,y_min,y_max,psi_min,psi_max,u_min,u_max,ax_min,ax_max,sa_min,sa_max = d
