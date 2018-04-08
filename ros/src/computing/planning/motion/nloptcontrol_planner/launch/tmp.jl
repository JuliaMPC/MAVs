using RobotOS, YAML, VehicleModels, NLOptControl, VehicleModels, MAVs

p = Vpara()
#p = VparaKB()
# To find fieldnames
r = ()
for i in 1:length(fieldnames(p))
      r = (r...,fieldnames(p)[i])
end
# KB
#la,lb,x_min,x_max,y_min,y_max,psi_min,psi_max,u_min,u_max,ax_min,ax_max,sa_min,sa_max,x0_,y0_,psi0_,u0_,ax0_
# pa = VparaKB(la=copy(c["vehicle"][:la]),lb=copy(c["vehicle"][:lb]),x_min=copy(c["misc"]["Xlims"][1]),x_max=copy(c["misc"]["Xlims"][2]),y_min=copy(c["misc"]["Ylims"][1]),y_max=copy(c["misc"]["Ylims"][2]),psi_min=copy(c["vehicle"][:psi_min]),psi_max=copy(c["vehicle"][:psi_max]),u_min=copy(c["vehicle"][:u_min]),u_max=copy(c["vehicle"][:u_max]),ax_min=copy(c["vehicle"][:ax_min]),ax_max=copy(c["vehicle"][:ax_max]),sa_min=copy(c["vehicle"][:sa_min]),sa_max=copy(c["vehicle"][:sa_max]))

# 3DOF missing
# not needed
# x0_,y0_,psi0_,v0_,u0_,sa0_,sr0_,ax0_,jx0_,r0_
# calculated in VehicleModels.jl
# PC1=copy(c["vehicle"][:PC1]),PD1=copy(c["vehicle"][:PD1]),PD2=copy(c["vehicle"][:PD2]),PE1=copy(c["vehicle"][:PE1]),PE2=copy(c["vehicle"][:PE2]),PE3=copy(c["vehicle"][:PE3]),PK1=copy(c["vehicle"][:PK1]),PK2=copy(c["vehicle"][:PK2]),PH1=copy(c["vehicle"][:PH1]),PH2=copy(c["vehicle"][:PH2]),PV1=copy(c["vehicle"][:PV1]),PV2=copy(c["vehicle"][:PV2])
# a_t, b_t
#(:m, :Izz, :la, :lb, :FzF0, :FzR0, :dFzx_coeff, :KZX, :KZYR, :AXC, :x_min, :x_max, :y_min, :y_max, :sa_min, :sa_max, :psi_min, :psi_max, :u_min, :u_max, :sr_min, :sr_max, :jx_min, :jx_max, :FZ0, :PCY1, :PDY1, :PDY2, :PEY1, :PEY2, :PEY3, :PKY1, :PKY2, :PHY1, :PHY2, :PVY1, :PVY2, :PC1, :PD1, :PD2, :PE1, :PE2, :PE3, :PK1, :PK2, :PH1, :PH2, :PV1, :PV2, :Caf, :Car, :Fy_min, :Fy_max, :x0_, :y0_, :psi0_, :v0_, :u0_, :sa0_, :sr0_, :ax0_, :jx0_, :r0_, :Fz_min, :Fz_off, :a_t, :b_t, :EP)
pa = Vpara(m=copy(c["vehicle"][:m]),Izz=copy(c["vehicle"][:Izz]), la=copy(c["vehicle"][:la]), lb=copy(c["vehicle"][:lb]), FzF0=copy(c["vehicle"][:FzF0]), FzR0=copy(c["vehicle"][:FzR0]), dFzx_coeff=copy(c["vehicle"][:dFzx_coeff]), KZX=copy(c["vehicle"][:KZX]), KZYR=copy(c["vehicle"][:KZYR]), AXC=copy(c["vehicle"][:AXC]),x_min=copy(c["misc"]["Xlims"][1]),x_max=copy(c["misc"]["Xlims"][2]),y_min=copy(c["misc"]["Ylims"][1]),y_max=copy(c["misc"]["Ylims"][2]),sa_min=copy(c["vehicle"][:sa_min]),sa_max=copy(c["vehicle"][:sa_max]),psi_min=copy(c["vehicle"][:psi_min]),psi_max=copy(c["vehicle"][:psi_max]),u_min=copy(c["vehicle"][:u_min]),u_max=copy(c["vehicle"][:u_max]),sr_min=copy(c["vehicle"][:sr_min]),sr_max=copy(c["vehicle"][:sr_max]),jx_min=copy(c["vehicle"][:jx_min]),jx_max=copy(c["vehicle"][:jx_max]),FZ0=copy(c["vehicle"][:FZ0]),PCY1=copy(c["vehicle"][:PCY1]),PDY1=copy(c["vehicle"][:PDY1]),PDY2=copy(c["vehicle"][:PDY2]),PEY1=copy(c["vehicle"][:PEY1]),PEY2=copy(c["vehicle"][:PEY2]),PEY3=copy(c["vehicle"][:PEY3]),PKY1=copy(c["vehicle"][:PKY1]),PKY2=copy(c["vehicle"][:PKY2]),PHY1=copy(c["vehicle"][:PHY1]),PHY2=copy(c["vehicle"][:PHY2]),PVY1=copy(c["vehicle"][:PVY1]),PVY2=copy(c["vehicle"][:PVY2]),Caf=copy(c["vehicle"][:Caf]),Car=copy(c["vehicle"][:Car]),Fy_min=copy(c["vehicle"][:Fy_min]),Fy_max=copy(c["vehicle"][:Fy_max]),Fz_min=copy(c["vehicle"][:Fz_min]),Fz_off=copy(c["vehicle"][:Fz_off]),EP=copy(c["misc"]["EP"]))

#pa = VparaKB(x_min=copy(c["misc"]["Xlims"][1]),x_max=copy(c["misc"]["Xlims"][2]),y_min=copy(c["misc"]["Ylims"][1]),y_max=copy(c["misc"]["Ylims"][2]));


  # message for solution to optimal control problem
  plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")
  pub = Publisher{Control}(string(plannerNamespace,"/control"), queue_size=10)
  #pub_path = Publisher{Path}(string(plannerNamespace,"/path"), queue_size=10)
  pub_path = Publisher{Path}("/path", queue_size=10)

  sub = Subscriber{Control}(string(plannerNamespace, "/control"), setTrajParams, queue_size = 10)

  # get the parameters
  #if !RobotOS.has_param("planner/nloptcontrol_planner/misc")
  #    error("Please set rosparam:planner/nloptcontrol_planner/misc")
  #elseif !RobotOS.has_param("case")
  #    error("Please set rosparam: case")
  #elseif !RobotOS.has_param("planner/nloptcontrol_planner")
  #    error("Please set rosparam: planner/nloptcontrol_planner")
  #end

  # using the filenames set as rosparams, the datatypes of the parameters get messed up if they are put on the ROS server
  # and then loaded into julia through RobotOS.jl; but less is messed up by loading using YAML.jl
  case = YAML.load(open(RobotOS.get_param("case_params_path")))["case"]
  planner = YAML.load(open(RobotOS.get_param("planner_params_path")))["planner"]["nloptcontrol_planner"]
  vehicle = YAML.load(open(RobotOS.get_param("vehicle_params_path")))["vehicle"]["nloptcontrol_planner"]

  c = YAML.load(open(string(Pkg.dir("MAVs"),"/config/empty.yaml")))
  c["vehicle"] = convert(Dict{Symbol,Any}, vehicle)  # need symbols to work with VehicleModels.jl
  c["weights"] = planner["weights"]
  c["misc"] = planner["misc"]
  c["solver"] = planner["solver"]
  c["tolerances"] = planner["tolerances"]
  c["X0"] = case["actual"]["X0"]
  c["goal"] = case["goal"]

  if RobotOS.get_param("system/nloptcontrol_planner/flags/known_environment")
    c["obstacle"] = case["actual"]["obstacle"]
  else  # NOTE currently the the python parcer does not like assumed/obstacle format!, this will fail!
    c["obstacle"] = case["assumed"]["obstacle"]
  end
  # fix messed up data types
  for keyA in ["weights", "misc", "X0", "solver", "tolerances"]
    for (key,value) in c[keyA]
      if isequal(value, "NaN"); c[keyA][key] = NaN; end
    end
  end
  for keyA in ["misc"]
    for (key,value) in c[keyA]
      if isequal(typeof(c[keyA][key]),String); c[keyA][key] = Symbol(c[keyA][key]); end
    end
  end


  # NOTE these are not needed
#  for keyA in ["vehicle"]
#    for (key,value) in c[keyA]
#      if isequal(typeof(c[keyA][key]),String); c[keyA][key] = Float64(c[keyA][key]); end
#    end
#  end
#  if c["solver"]["warm_start_init_point"]
#    c["solver"]["warm_start_init_point"] = "yes"
#  else
#    c["solver"]["warm_start_init_point"] = "no"
#  end
