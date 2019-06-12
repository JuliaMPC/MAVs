"""
# to visualize the current obstacle in the field
obstaclePlot(n,c)

# to run it after a single optimization
pp = obstaclePlot(n,c,1)

# to create a new plot
pp = obstaclePlot(n,c,idx)

# to add to an exsting position plot
pp = obstaclePlot(n,c,idx,pp;(:append=>true))

# posterPlot
pp = obstaclePlot(n,c,ii,pp;(:append=>true),(:posterPlot=>true)) # add obstacles
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 7/5/2017 \n
--------------------------------------------------------------------------------------\n
"""
function obstaclePlot(n,c,idx,args...;kwargs...)
  r = n.r
  kw = Dict(kwargs)

  # check to see if is a basic plot
  if !haskey(kw,:basic); basic=false;
  else; basic=get(kw,:basic,0);
  end

  # check to see if is a poster plot
  if !haskey(kw,:posterPlot);posterPlot=false;
  else; posterPlot=get(kw,:posterPlot,0);
  end

  # check to see if user wants to reduce the size of the markers TODO get ride of this eventually
  if !haskey(kw,:smallMarkers);smallMarkers=false;
  else;smallMarkers=get(kw,:smallMarkers,0);
  end

  # check to see if user wants to crash
  if !haskey(kw,:obstacleMiss);obstacleMiss=false;
  else;obstacleMiss=get(kw,:obstacleMiss,0);
  end

  if basic
    pp = plot(0,leg=:false)

    if typeof(c["goal"]["x"])==Float64  # TODO remove redundant code
      if isnan(c["goal"]["tol"]); rg = 1; else rg = c["goal"]["tol"]; end
      if !posterPlot || idx == r.ocp.evalNum
        pts = Plots.partialcircle(0,2π,100,rg)
        x, y = Plots.unzip(pts)
        x += c["goal"]["x"];  y += c["goal"]["yVal"]
        pts = collect(zip(x, y))
        if !smallMarkers  #TODO get ride of this-> will not be a legend for this case
          scatter!((c["goal"]["x"],c["goal"]["yVal"]),marker=_pretty_defaults[:goal_marker][1],label="Goal Area")
        end
        plot!(pts,line=_pretty_defaults[:goal_line],fill=_pretty_defaults[:goal_fill][1],leg=true,label="")

        if isnan(c["tolerances"]["fx"]); rg = 1; else rg = c["tolerances"]["fx"]; end
        pts = Plots.partialcircle(0,2π,100,rg)
        x, y = Plots.unzip(pts)
        x += c["goal"]["x"];  y += c["goal"]["yVal"]
        pts = collect(zip(x, y))
        if !smallMarkers  #TODO get ride of this-> will not be a legend for this case
          scatter!((c["goal"]["x"],c["goal"]["yVal"]),marker=_pretty_defaults[:goal_marker][2],label="Goal")
        end
        plot!(pts,line=_pretty_defaults[:goal_line],fill=_pretty_defaults[:goal_fill][2],leg=true,label="")
      end
    end

    if !isempty(c["obstacle"]["radius"])
      for i in 1:length(c["obstacle"]["radius"])
          # create an ellipse
          pts = Plots.partialcircle(0,2π,100,c["obstacle"]["radius"][i])
          x, y = Plots.unzip(pts)
          tc = 0
          x += c["obstacle"]["x0"][i] + c["obstacle"]["vx"][i]*tc
          y = c["obstacle"]["radius"][i]/c["obstacle"]["radius"][i]*y + c["obstacle"]["y0"][i] + c["obstacle"]["vy"][i]*tc
          pts = collect(zip(x, y))
          if i==1
            scatter!((c["obstacle"]["x0"][i],c["obstacle"]["y0"][i]),marker=_pretty_defaults[:obstacle_marker],label="Obstacles")
          end
          plot!(pts,line=_pretty_defaults[:obstacle_line],fill=_pretty_defaults[:obstacle_fill],leg=true,label="") #line=(3.0,0.0,:solid,:red)
        #  plot!(pts,line=(4.0,0.0,:solid,:red),fill=(0, 1, :red),leg=true,label="")
      end
    end
  else
    # check to see if user would like to add to an existing plot
    if !haskey(kw,:append); kw_ = Dict(:append => false); append = get(kw_,:append,0);
    else; append = get(kw,:append,0);
    end
    if !append; pp=plot(0,leg=:false); else pp=args[1]; end

    # plot the goal; assuming same in x and y
    if typeof(c["goal"]["x"])==Float64 # TODO remove redundant code
      if isnan(c["goal"]["tol"]); rg = 1; else rg = c["goal"]["tol"]; end
      if !posterPlot || idx ==r.ocp.evalNum
        pts = Plots.partialcircle(0,2π,100,rg)
        x, y = Plots.unzip(pts)
        x += c["goal"]["x"];  y += c["goal"]["yVal"]
        pts = collect(zip(x, y))
        if !smallMarkers  #TODO get ride of this-> will not be a legend for this case
          scatter!((c["goal"]["x"],c["goal"]["yVal"]),marker=_pretty_defaults[:goal_marker][1],label="Goal Area")
        end
        plot!(pts,line=_pretty_defaults[:goal_line],fill=_pretty_defaults[:goal_fill][1],leg=true,label="")

        if isnan(c["tolerances"]["fx"]); rg=1; else rg = c["tolerances"]["fx"]; end
        pts = Plots.partialcircle(0,2π,100,rg)
        x, y = Plots.unzip(pts)
        x += c["goal"]["x"];  y += c["goal"]["yVal"]
        pts = collect(zip(x, y))
        if !smallMarkers  #TODO get ride of this-> will not be a legend for this case
          scatter!((c["goal"]["x"],c["goal"]["yVal"]),marker=_pretty_defaults[:goal_marker][2],label="Goal")
        end
        plot!(pts,line=_pretty_defaults[:goal_line],fill=_pretty_defaults[:goal_fill][2],leg=true,label="")
      end
    end

    if typeof(c["obstacle"]["radius"][1])==Float64
      for i in 1:length(c["obstacle"]["radius"])
        # create an ellipse
        pts = Plots.partialcircle(0,2π,100,c["obstacle"]["radius"][i])
        x, y = Plots.unzip(pts)
        if _pretty_defaults[:plant]
          x += c["obstacle"]["x0"][i] + c["obstacle"]["vx"][i]*r.ip.dfsplant[idx][:t][end];
          y = c["obstacle"]["radius"][i]/c["obstacle"]["radius"][i]*y + c["obstacle"]["y0"][i] + c["obstacle"]["vy"][i]*r.ip.dfsplant[idx][:t][end];
        else
          if r.ocp.dfs[idx]!=nothing
            tc=r.ocp.dfs[idx][:t][end];
          else
            tc=0;
            if idx!=1; warn("\n Obstacles set to inital condition for current frame. \n") end
          end
          x += c["obstacle"]["x0"][i] + c["obstacle"]["vx"][i]*tc;
          y = c["obstacle"]["radius"][i]/c["obstacle"]["radius"][i]*y + c["obstacle"]["y0"][i] + c["obstacle"]["vy"][i]*tc;
        end
        pts = collect(zip(x, y))
        X = c["obstacle"]["x0"][i] + c["obstacle"]["vx"][i]*r.ip.dfsplant[idx][:t][end]
        Y = c["obstacle"]["y0"][i] + c["obstacle"]["vy"][i]*r.ip.dfsplant[idx][:t][end]
        if posterPlot
          shade=idx/r.ocp.evalNum;
          if idx==r.ocp.evalNum && i==1
            scatter!((X,Y),marker=_pretty_defaults[:obstacle_marker],label="Obstacles")
          end
          plot!(pts,line=_pretty_defaults[:obstacle_line],fill=(0,shade,:red),leg=true,label="")
          annotate!(X,Y,text(string(idx*c["misc"]["tex"],"s"),10,:black,:center))
        else
          if i==1 && !smallMarkers
            scatter!((X,Y),marker=_pretty_defaults[:obstacle_marker],label="Obstacles")
          end
          plot!(pts,line=_pretty_defaults[:obstacle_line],fill=_pretty_defaults[:obstacle_fill],leg=true,label="") #line=(3.0,0.0,:solid,:red)
          if obstacleMiss && i>8
            annotate!(X,Y+5,text("obstacle not seen!"),14,:red,:center)
          end
        end
      end
    end

    xaxis!(n.ocp.state.description[1])
    yaxis!(n.ocp.state.description[2])
    if !_pretty_defaults[:simulate] savefig(string(r.resultsDir,"/",n.ocp.state.name[1],"_vs_",n.ocp.state.name[2],".",_pretty_defaults[:format])); end
  end
  return pp
end

obstaclePlot(n,c)=obstaclePlot(n,c,1;(:basic=>true))

#=
using Plots
import Images
#ENV["PYTHONPATH"]="/home/febbo/.julia/v0.5/Conda/deps/usr/bin/python"
img=Images.load(Pkg.dir("PrettyPlots/src/humvee.png"));
plot(img)
=#
"""
pp=vehiclePlot(n,c,idx);
pp=vehiclePlot(n,c,idx,pp;(:append=>true));
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 4/4/2017 \n
--------------------------------------------------------------------------------------\n
"""
function vehiclePlot(n,c,idx,args...;kwargs...)
  r = n.r

  kw = Dict(kwargs)

  # check to see if user wants to zoom
  if !haskey(kw,:zoom); zoom = false;
  else; zoom=get(kw,:zoom,0);
  end

  # check to see if user would like to add to an existing plot
  if !haskey(kw,:append); append = false
  else; append = get(kw,:append,0);
  end
  if !append; pp=plot(0,leg=:false); else pp=args[1]; end

  # check to see if is a poster plot
  if !haskey(kw,:posterPlot); posterPlot=false;
  else; posterPlot=get(kw,:posterPlot,0);
  end

  # check to see if we want to set the limits to the position constraints
  if !haskey(kw,:setLims);setLims=false;
  else;setLims=get(kw,:setLims,0);
  end

  # check to see if user wants to reduce the size of the markers TODO get ride of this eventually
  if !haskey(kw,:smallMarkers);smallMarkers=false;
  else;smallMarkers=get(kw,:smallMarkers,0);
  end

  w = _pretty_defaults[:vehicle_width]; h=_pretty_defaults[:vehicle_length]
  XQ = [-w/2 w/2 w/2 -w/2 -w/2]
  YQ = [h/2 h/2 -h/2 -h/2 h/2]

  # plot the vehicle
  if _pretty_defaults[:plant]
    X_v = r.ip.dfsplant[idx][:x][end]  # using the end of the simulated data from the vehicle model
    Y_v = r.ip.dfsplant[idx][:y][end]
    PSI_v = r.ip.dfsplant[idx][:psi][end]-pi/2
  else
   #TODO condider eliminating option to plot mpc only
    X_v = r.ocp.dfs[idx][:x][1] # start at begining
    Y_v = r.ocp.dfs[idx][:y][1]
    PSI_v = r.ocp.dfs[idx][:psi][1]-pi/2
  end

  P = [XQ;YQ]
  ct = cos(PSI_v)
  st = sin(PSI_v)
  R = [ct -st;st ct]
  P2 = R*P
  if !posterPlot || idx==r.ocp.evalNum
    if !smallMarkers # for legend
      scatter!((X_v,Y_v),marker=_pretty_defaults[:vehicle_marker], grid=true,label="Vehicle")
    end
  end
  scatter!((P2[1,:]+X_v,P2[2,:]+Y_v),ms=0,fill=_pretty_defaults[:vehicle_fill],leg=true,grid=true,label="")

  if !zoom && !setLims
    if _pretty_defaults[:plant]  # TODO push this to a higher level
      xL = minDF(r.ip.dfsplant,:x);xU=maxDF(r.ip.dfsplant,:x);
      yL = minDF(r.ip.dfsplant,:y);yU=maxDF(r.ip.dfsplant,:y);
    else
      xL = minDF(r.ocp.dfs,:x);xU=maxDF(r.ocp.dfs,:x);
      yL = minDF(r.ocp.dfs,:y);yU=maxDF(r.ocp.dfs,:y);
    end
      dx = xU - xL;dy = yU - yL; # axis equal
      if dx>dy; yU=yL+dx; else xU=xL+dy; end
      xlims!(xL,xU)
      ylims!(yL,yU)
  else
    xlims!(X_v-20.,X_v+80.)
    ylims!(Y_v-50.,Y_v+50.)
  end

  if posterPlot
    t = idx*c["misc"]["tex"]
    annotate!(X_v,Y_v-4,text(string("t=",t," s"),10,:black,:center))
  end

  if setLims || posterPlot
    xlims!(c["misc"]["Xlims"][1],c["misc"]["Xlims"][2])
    ylims!(c["misc"]["Ylims"][1],c["misc"]["Ylims"][2])
  end

  if !_pretty_defaults[:simulate]; savefig(string(r.resultsDir,"x_vs_y",".",_pretty_defaults[:format])); end
  return pp
end
"""
vt = vtPlot(n,idx)
-------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 3/11/2017 \n
--------------------------------------------------------------------------------------\n
"""
function vtPlot(n,idx::Int64)
  r = n.r
  pa = n.ocp.params[1]
  c = n.ocp.params[5]

	@unpack_Vpara pa

  # check length of n.r.ocp.dfs to see if there is data for the provided idx
  if length(n.r.ocp.dfs) >= idx
    ocpPlot = true
  else
    ocpPlot = false
  end

#  if idx > length(n.r.ocp.dfs)
#    warn("Cannot plot idx = ", idx, " because length(n.r.ocp.dfs) = ", length(n.r.ocp.dfs), ". \n
#          Skipping idx in vtPlot().")
#    vt = plot(0,leg=:false)
#    return vt
#  end

  if ocpPlot
    t_vec=linspace(0.0,max(5,ceil(r.ocp.dfs[end][:t][end]/1)*1),_pretty_defaults[:L]);
  else
    t_vec=linspace(0.0,max(5,ceil(r.ip.dfsplant[end][:t][end]/1)*1),_pretty_defaults[:L]);
  end

	vt=plot(t_vec,c["vehicle"][:Fz_off]*ones(_pretty_defaults[:L],1),line=_pretty_defaults[:limit_lines][2],label="min")

  if !_pretty_defaults[:plantOnly] && ocpPlot

    if  isequal(c["misc"]["model"],:ThreeDOFv2)
      V=r.ocp.dfs[idx][:v];R=r.ocp.dfs[idx][:r];SA=r.ocp.dfs[idx][:sa];
      Ax=r.ocp.dfs[idx][:ax]; U=r.ocp.dfs[idx][:ux];
    elseif isequal(c["misc"]["model"],:ThreeDOFv1) # constain speed (the model is not optimizing speed)
      V=r.ocp.dfs[idx][:v];R=r.ocp.dfs[idx][:r];SA=r.ocp.dfs[idx][:sa];
      U=c["misc"]["ux"]*ones(length(V)); Ax=zeros(length(V));
    elseif isequal(c["misc"]["model"],:KinematicBicycle2)
      Vtotal = r.ocp.dfs[idx][:ux]
      Atotal = r.ocp.dfs[idx][:ax]
      SA = r.ocp.dfs[idx][:sa]
      Beta = atan.(la/(la+lb)*tan.(SA))
      V = Vtotal.*sin.(Beta)
      U = Vtotal.*cos.(Beta)
      Ax = Atotal.*cos.(Beta)
      Rturn = (la + lb)./SA  # turning radius, Ackerman angle (small angle assumption)
      R = V./Rturn
    end
    plot!(r.ocp.dfs[idx][:t],@FZ_RL(),line=_pretty_defaults[:mpc_lines][1],label="RL-mpc");
    plot!(r.ocp.dfs[idx][:t],@FZ_RR(),line=_pretty_defaults[:mpc_lines][2],label="RR-mpc");
    plot!(r.ocp.dfs[idx][:t],@FZ_FL(),line=_pretty_defaults[:mpc_lines][7],label="FL-mpc");
    plot!(r.ocp.dfs[idx][:t],@FZ_FR(),line=_pretty_defaults[:mpc_lines][4],label="FR-mpc");
  end
  if _pretty_defaults[:plant]

    if isequal(c["misc"]["model"],:ThreeDOFv2)
      temp = [r.ip.dfsplant[jj][:v] for jj in 1:idx] # V
      V = [idx for tempM in temp for idx=tempM]
      temp = [r.ip.dfsplant[jj][:ux] for jj in 1:idx]; # ux
      U=[idx for tempM in temp for idx=tempM];

      temp = [r.ip.dfsplant[jj][:ax] for jj in 1:idx]; # ax
      Ax=[idx for tempM in temp for idx=tempM];

      temp = [r.ip.dfsplant[jj][:r] for jj in 1:idx]; # r
      R=[idx for tempM in temp for idx=tempM];

      temp = [r.ip.dfsplant[jj][:sa] for jj in 1:idx]; # sa
      SA=[idx for tempM in temp for idx=tempM];
    elseif  isequal(c["misc"]["model"],:ThreeDOFv1) # constain speed ( the model is not optimizing speed)
      temp = [r.ip.dfsplant[jj][:v] for jj in 1:idx] # V
      V = [idx for tempM in temp for idx=tempM]
      U=c["misc"]["ux"]*ones(length(V)); Ax=zeros(length(V));

      temp = [r.ip.dfsplant[jj][:r] for jj in 1:idx]; # r
      R=[idx for tempM in temp for idx=tempM];

      temp = [r.ip.dfsplant[jj][:sa] for jj in 1:idx]; # sa
      SA=[idx for tempM in temp for idx=tempM];
    elseif isequal(c["misc"]["model"],:KinematicBicycle2)
      temp = [r.ip.dfsplant[jj][:ux] for jj in 1:idx]; # u
      Vtotal=[idx for tempM in temp for idx=tempM];

      temp = [r.ip.dfsplant[jj][:ax] for jj in 1:idx]; # a
      Atotal=[idx for tempM in temp for idx=tempM];

      temp = [r.ip.dfsplant[jj][:sa] for jj in 1:idx]; # sa
      SA = [idx for tempM in temp for idx=tempM];

      Beta = atan.(la/(la+lb)*tan.(SA))
      V = Vtotal.*sin.(Beta)
      U = Vtotal.*cos.(Beta)
      Ax = Atotal.*cos.(Beta)
      Rturn = (la + lb)./SA  # turning radius, Ackerman angle (small angle assumption)
      R = V./Rturn
    end

    # time
    temp = [r.ip.dfsplant[jj][:t] for jj in 1:idx]
    time = [idx for tempM in temp for idx=tempM]

    plot!(time,@FZ_RL(),line=_pretty_defaults[:plant_lines][1],label="RL-plant")
    plot!(time,@FZ_RR(),line=_pretty_defaults[:plant_lines][2],label="RR-plant")
    plot!(time,@FZ_FL(),line=_pretty_defaults[:plant_lines][3],label="FL-plant")
    plot!(time,@FZ_FR(),line=_pretty_defaults[:plant_lines][4],label="FR-plant")
  end
  plot!(size=_pretty_defaults[:size])
	adjust_axis(xlims(),ylims())
  xlims!(t_vec[1],t_vec[end])
  ylims!(_pretty_defaults[:tire_force_lims])
	title!("Vertical Tire Forces"); yaxis!("Force (N)"); xaxis!("time (s)")
	if !_pretty_defaults[:simulate] savefig(string(r.resultsDir,"vt.",_pretty_defaults[:format])) end
  return vt
end

"""
axp=axLimsPlot(n,pa,idx)
axp=axLimsPlot(n,pa,idx,axp;(:append=>true))
# this plot adds the nonlinear limits on acceleration to the plot
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 3/11/2017 \n
--------------------------------------------------------------------------------------\n
"""
function axLimsPlot(n,pa,idx::Int64,args...;kwargs...)
  r = n.r

  kw = Dict(kwargs)
  if !haskey(kw,:append); kw_ = Dict(:append => false); append = get(kw_,:append,0);
  else; append = get(kw,:append,0);
  end
  if !append; axp=plot(0,leg=:false); else axp=args[1]; end

  # check length of n.r.ocp.dfs to see if there is data for the provided idx
  if length(n.r.ocp.dfs) >= idx
    ocpPlot = true
  else
    ocpPlot = false
  end

  @unpack_Vpara pa

  if !_pretty_defaults[:plant] &&  !_pretty_defaults[:plantOnly] && ocpPlot
    t_vec = linspace(0.0,max(5,ceil(r.ocp.dfs[end][:t][end]/1)*1),_pretty_defaults[:L])
  else
    t_vec = linspace(0,max(5,ceil(r.ip.dfsplant[end][:t][end]/1)*1),_pretty_defaults[:L])
  end

  if !_pretty_defaults[:plantOnly] && ocpPlot
    U = r.ocp.dfs[idx][:ux]
    plot!(r.ocp.dfs[idx][:t],@Ax_max(),line=_pretty_defaults[:limit_lines][2],label="max-mpc")
    plot!(r.ocp.dfs[idx][:t],@Ax_min(),line=_pretty_defaults[:limit_lines][1],label="min-mpc")
  end
  if _pretty_defaults[:plant]
    temp = [r.ip.dfsplant[jj][:ux] for jj in 1:idx] # ux
    U = [idx for tempM in temp for idx=tempM]

    # time
    temp = [r.ip.dfsplant[jj][:t] for jj in 1:idx]
    time = [idx for tempM in temp for idx=tempM]

    plot!(time,@Ax_max(),line=_pretty_defaults[:limit_lines][4],label="max-plant")
    plot!(time,@Ax_min(),line=_pretty_defaults[:limit_lines][3],label="min-plant")
  end
  ylims!(_pretty_defaults[:ax_lims])
  plot!(size=_pretty_defaults[:size])
  if !_pretty_defaults[:simulate] savefig(string(r.resultsDir,"axp.",_pretty_defaults[:format])) end
  return axp
end


"""
# to visualize the current track in the field
trackPlot(c)

pp=trackPlot(c,pp;(:append=>true));
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 5/1/2017 \n
--------------------------------------------------------------------------------------\n
"""
function trackPlot(c,args...;kwargs...)
  kw = Dict(kwargs);
  # s=Settings(); NOTE commented out

  # check to see if user would like to add to an existing plot
  if !haskey(kw,:append); append=false;
  else; append = get(kw,:append,0);
  end

  # check to see if user wants to reduce the size of the markers TODO get ride of this eventually
  if !haskey(kw,:smallMarkers);smallMarkers=false;
  else;smallMarkers=get(kw,:smallMarkers,0);
  end

  if !append; pp=plot(0,leg=:false); else pp=args[1]; end

  if c["track"]["func"]==:poly
    f(y)=c["track"]["a"][1] + c["track"]["a"][2]*y + c["track"]["a"][3]*y^2 + c["track"]["a"][4]*y^3 + c["track"]["a"][5]*y^4;
    Y=c["track"]["Y"];
    X=f.(Y);
  elseif c["track"]["func"]==:fourier
    ff(x)=c["track"]["a"][1]*sin(c["track"]["b"][1]*x+c["track"]["c"][1]) + c["track"]["a"][2]*sin(c["track"]["b"][2]*x+c["track"]["c"][2]) + c["track"]["a"][3]*sin(c["track"]["b"][3]*x+c["track"]["c"][3]) + c["track"]["a"][4]*sin(c["track"]["b"][4]*x+c["track"]["c"][4]) + c["track"]["a"][5]*sin(c["track"]["b"][5]*x+c["track"]["c"][5]) + c["track"]["a"][6]*sin(c["track"]["b"][6]*x+c["track"]["c"][6]) + c["track"]["a"][7]*sin(c["track"]["b"][7]*x+c["track"]["c"][7]) + c["track"]["a"][8]*sin(c["track"]["b"][8]*x+c["track"]["c"][8])+c["track"]["y0"];
    X=c["track"]["X"];
    Y=ff.(X);
  end

  if !smallMarkers; L=40; else L=4; end

  plot!(X,Y,label="Road",line=(L,0.3,:solid,:black))
  return pp
end

"""

pp=lidarPlot(r,c,idx);
pp=lidarPlot(r,c,idx,pp;(:append=>true));
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/3/2017, Last Modified: 4/3/2017 \n
--------------------------------------------------------------------------------------\n
"""
function lidarPlot(r,c,idx,args...;kwargs...)
  kw = Dict(kwargs);

  # check to see if user would like to add to an existing plot
  if !haskey(kw,:append); kw_ = Dict(:append => false); append = get(kw_,:append,0);
  else; append = get(kw,:append,0);
  end
  if !append; pp=plot(0,leg=:false); else pp=args[1]; end

  if idx > length(r.ocp.dfs)
    warn("Cannot plot idx = ", idx, " because length(r.ocp.dfs) = ", length(r.ocp.dfs), ". \n
          Skipping idx in lidarPlot().")
    return pp
  end

  # plot the LiDAR
  #if _pretty_defaults[:plant]
  #  X_v = r.ip.dfsplant[idx][:x][1]  # using the begining of the simulated data from the vehicle model
  #  Y_v = r.ip.dfsplant[idx][:y][1]
  #  PSI_v = r.ip.dfsplant[idx][:psi][1]-pi/2
  #else # NOTE plotting w.r.t. the OCP, that is how the constraints are set up
  X_v = r.ocp.dfs[idx][:x][1]
  Y_v = r.ocp.dfs[idx][:y][1]
  PSI_v = r.ocp.dfs[idx][:psi][1]-pi/2
  #end

  pts = Plots.partialcircle(PSI_v-pi,PSI_v+pi,50,c["misc"]["Lr"]);
  x, y = Plots.unzip(pts);
  x += X_v;  y += Y_v;
  pts = collect(zip(x, y));
  plot!(pts,line=_pretty_defaults[:lidar_line],fill=_pretty_defaults[:lidar_fill],leg=true,label="LiDAR Range")
  return pp
end
"""
# to plot the second solution
pp = posPlot(n,2)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 5/1/2017 \n
--------------------------------------------------------------------------------------\n
"""
function posPlot(n,idx;kwargs...)
  r = n.r
  c = n.ocp.params[5]

  kw = Dict(kwargs)
  if !haskey(kw,:zoom); zoom=false;
  else; zoom=get(kw,:zoom,0);
  end
  # check to see if we want to set the limits to the position constraints
  if !haskey(kw,:setLims)
    setLims = false
  else
    setLims = get(kw,:setLims,0)
  end

  # check to see if user wants to reduce the size of the markers TODO get ride of this eventually
  if !haskey(kw,:smallMarkers);smallMarkers=false;
  else;smallMarkers=get(kw,:smallMarkers,0);
  end

  # check to see if user wants to crash
  if !haskey(kw,:obstacleMiss);obstacleMiss=false;
  else;obstacleMiss=get(kw,:obstacleMiss,0);
  end

  if haskey(c,"track"); pp=trackPlot(c;(:smallMarkers=>smallMarkers)); else pp=plot(0,leg=:false); end  # track
  if haskey(c["misc"],"Lr"); pp=lidarPlot(r,c,idx,pp;(:append=>true)); end  # lidar

  pp = obstaclePlot(n,c,idx,pp;(:append=>true),(:smallMarkers=>smallMarkers),(:obstacleMiss=>obstacleMiss))   # obstacles
  pp = statePlot(n,idx,1,2,pp;(:lims=>false),(:append=>true)) # vehicle trajectory
  pp = vehiclePlot(n,c,idx,pp;(:append=>true),(:zoom=>zoom),(:setLims=>setLims),(:smallMarkers=>smallMarkers))# vehicle

  if !setLims; plot!(aspect_ratio=:equal); end

  if !_pretty_defaults[:simulate] savefig(string(r.resultsDir,"pp.",_pretty_defaults[:format])) end
  return pp
end

"""
main=mainPlot(n,idx;kwargs...)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 2/12/2018 \n
--------------------------------------------------------------------------------------\n
"""
function mainPlot(n,idx;kwargs...)
  r = n.r
  pa = n.ocp.params[1]
  c = n.ocp.params[5]

  kw = Dict(kwargs)
  if !haskey(kw,:mode);error("select a mode for the simulation \n")
  else; mode=get(kw,:mode,0);
  end

  if mode==:path1
    sap=statePlot(n,idx,6)
    vp=statePlot(n,idx,3)
    rp=statePlot(n,idx,4)
    vt=vtPlot(n,idx)
    pp=posPlot(n,idx)
    pz=posPlot(n,idx;(:zoom=>true))
    if _pretty_defaults[:plant]; tp=tPlot(n,idx); else; tp=plot(0,leg=:false); end
    l = @layout [a{0.3w} [grid(2,2)
                          b{0.2h}]]
    mainS=plot(pp,sap,vt,pz,rp,tp,layout=l,size=_pretty_defaults[:size]);
  elseif mode==:path2
    sap=statePlot(n,idx,6);plot!(leg=:topleft)
    vp=statePlot(n,idx,3);plot!(leg=:topleft)
    vt=vtPlot(n,idx);plot!(leg=:bottomleft)
    pz=posPlot(n,idx;(:zoom=>true));plot!(leg=:topleft)
    if _pretty_defaults[:plant]; tp=tPlot(n,idx);plot!(leg=:topright) else; tp=plot(0,leg=:false);plot!(leg=:topright) end
    l=@layout([a{0.6w} [b;c]; d{0.17h}])
    mainS=plot(pz,vt,sap,tp,layout=l,size=_pretty_defaults[:size]);
  elseif mode==:path3
    sap=statePlot(n,idx,6);plot!(leg=:topleft)
    vp=statePlot(n,idx,3);plot!(leg=:topleft)
    vt=vtPlot(n,idx);plot!(leg=:bottomleft)
    pp=posPlot(n,idx;(:setLims=>true),(:smallMarkers=>true),(:obstacleMiss=>false));plot!(leg=false);
    pz=posPlot(n,idx;(:zoom=>true),(:obstacleMiss=>false));plot!(leg=:topleft)
    if _pretty_defaults[:plant]; tp=tPlot(n,idx);plot!(leg=:topright) else; tp=plot(0,leg=:false);plot!(leg=:topright) end
    l=@layout([[a;
                b{0.2h}] [c;d;e]])
    mainS=plot(pz,pp,vt,sap,tp,layout=l,size=_pretty_defaults[:size]);
  elseif mode==:zoom1
    if isequal(c["misc"]["model"],:ThreeDOFv2)
    sap = statePlot(n,idx,6);plot!(leg=:topleft)
    longv = statePlot(n,idx,7);plot!(leg=:topleft)
    axp=axLimsPlot(n,pa,idx);# add nonlinear acceleration limits
    axp=statePlot(n,idx,8,axp;(:lims=>false),(:append=>true));plot!(leg=:bottomright);
    vt = vtPlot(n,idx);plot!(leg=:bottomleft)
    pp = posPlot(n,idx;(:setLims=>true),(:smallMarkers=>true),(:obstacleMiss=>false));plot!(leg=false);
    pz = posPlot(n,idx;(:zoom=>true),(:obstacleMiss=>false));plot!(leg=:topleft)
    if _pretty_defaults[:plant]; tp=tPlot(n,idx);plot!(leg=:topright) else; tp=plot(0,leg=:false);plot!(leg=:topright) end
    l = @layout [[a;b{0.2h}] c{0.1w} [d;e;f;g]]
    mainS = plot(pz,tp,pp,vt,sap,longv,axp,layout=l,size=_pretty_defaults[:size])
    else
      error("TODO")
    end
  elseif mode==:open1
    if isequal(c["misc"]["model"],:ThreeDOFv2)
      sap=statePlot(n,idx,6);plot!(leg=:topleft)
      longv=statePlot(n,idx,7);plot!(leg=:topleft)
      axp=axLimsPlot(n,pa,idx);# add nonlinear acceleration limits
      axp=statePlot(n,idx,8,axp;(:lims=>false),(:append=>true));plot!(leg=:bottomright);
      pp=posPlot(n,idx;(:setLims=>false));plot!(leg=:topright);
      if _pretty_defaults[:plant]; tp=tPlot(n,idx); else; tp=plot(0,leg=:false); end
      vt=vtPlot(n,idx)
      l = @layout [a{0.5w} [grid(2,2)
                            b{0.2h}]]
      mainS=plot(pp,sap,vt,longv,axp,tp,layout=l,size=_pretty_defaults[:size]);
    elseif isequal(c["misc"]["model"],:KinematicBicycle2)
      sap=controlPlot(n,idx,1);plot!(leg=:topleft)
      longv=statePlot(n,idx,4);plot!(leg=:topleft)
      axp=controlPlot(n,idx,2);plot!(leg=:bottomright);
      pp=posPlot(n,idx;(:setLims=>true));plot!(leg=:topright);
      if _pretty_defaults[:plant]; tp=tPlot(n,idx); else; tp=plot(0,leg=:false); end
      vt=vtPlot(n,idx)
      l = @layout [a{0.5w} [grid(2,2)
                            b{0.2h}]]
      mainS=plot(pp,sap,vt,longv,axp,tp,layout=l,size=_pretty_defaults[:size]);
    end

  end

  return mainS
end


"""
mainSim(n;(:mode=>:open1))
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/13/2017, Last Modified: 7/6/2017 \n
--------------------------------------------------------------------------------------\n
"""

function mainSim(n;kwargs...)
  r = n.r
  kw = Dict(kwargs)
  if !haskey(kw,:mode);error("select a mode for the simulation \n")
  else; mode=get(kw,:mode,0);
  end

  if n.r.ocp.evalNum>2
    if typeof(n.r.ocp.dfs[end]) == nothing || typeof(n.r.ocp.dfs[end]) == Void
      pop!(n.r.ocp.dfs)
    end # assuming there is only one nothing
    if abs(n.r.ocp.dfs[end][:t][end]-n.r.ocp.dfs[end][:t][1]) < 0.2
      warn("\n The time scale for the final optimization is too small to plot.\n
                Deleting the final element in the results! \n ")
      pop!(n.r.ocp.dfs)
      num = n.r.ocp.evalNum - 1
    else
      num = n.r.ocp.evalNum
    end
      if n.r.ocp.evalNum > length(n.r.ip.dfsplant)
        warn("Reducing the number of frames. n.r.ocp.evalNum > length(n.r.ip.dfsplant) ")
        num = length(n.r.ip.dfsplant)
      end
     anim = @animate for idx in 1:num
       mainPlot(n,idx;(:mode=>mode))
    end
    cd(n.r.resultsDir)
      gif(anim,"mainSim.gif",fps=Int(ceil(1/n.mpc.v.tex)));
      run(`ffmpeg -f gif -i mainSim.gif RESULT.mp4`)
    cd(n.r.mainDir)
  else
    warn("the evaluation number was not greater than 2. Cannot make animation. Plotting a static plot.")
    warn("\n Modifying current plot settings! \n")
    plotSettings(;(:simulate=>false),(:plant=>false));
    mainPlot(n,1;(:mode=>mode))
  end
  return nothing
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/13/2017, Last Modified: 5/1/2017 \n
--------------------------------------------------------------------------------------\n
"""

function pSim(n,c)
  r = n.r

  anim = @animate for ii in 1:length(r.ocp.dfs)
    posPlot(n,ii);
  end
  gif(anim, string(r.resultsDir,"posSim.gif"), fps=Int(ceil(1/n.mpc.v.tex)) );
  return nothing
end


"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/13/2017, Last Modified: 4/13/2017 \n
--------------------------------------------------------------------------------------\n
"""

function pSimGR(n)

  ENV["GKS_WSTYPE"]="mov"
  gr(show=true)
  for ii in 1:length(n.r.ocp.dfs)
    posPlot(n,ii);
  end
end

"""
default(guidefont = font(17), tickfont = font(15), legendfont = font(12), titlefont = font(20))
s=Settings(;save=true,MPC=true,simulate=false,format=:png,plantOnly=true);
posterP(n,c)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/13/2017, Last Modified: 6/22/2017 \n
--------------------------------------------------------------------------------------\n
"""

function posterP(n)
  r = n.r
  pa = n.ocp.params[1]
  c = n.ocp.params[5]

  if n.r.ocp.status==:Infeasible
    warn("\n Current solution is infeasible! Will try to plot, but it may fail... \n")
  end
  warn("\n Modifying current plot settings! \n")
  plotSettings(;(:simulate=>false),(:plant=>true),(:plantOnly=>true));

  # static plots for each frame
  idx = length(n.r.ocp.dfs)
  idxT = length(n.r.ocp.dfsOpt[:tSolve])
  if isequal(c["misc"]["model"],:ThreeDOFv2)
    sap = statePlot(n,idx,6)
    longv = statePlot(n,idx,7)
    axp = axLimsPlot(n,pa,idx); # add nonlinear acceleration limits
    axp = statePlot(n,idx,8,axp;(:lims=>false),(:append=>true))
  elseif isequal(c["misc"]["model"],:KinematicBicycle2)
    sap = controlPlot(n,idx,1);plot!(leg=:topleft)
    longv = statePlot(n,idx,4);plot!(leg=:topleft)
    axp = controlPlot(n,idx,2);plot!(leg=:bottomright)
  end
  pp = statePlot(n,idx,1,2;(:lims=>false))
  if _pretty_defaults[:plant]; tp=tPlot(n,idx); else; tp=plot(0,leg=:false); end
  vt = vtPlot(n,idx)

  # dynamic plots ( maybe only update every 5 frames or so)
  L = length(n.r.ip.dfsplant)
  v = Vector(1:5:L); if v[end]!=L; append!(v,L); end
  for ii in v
    if ii==1
      st1 = 1;st2 = 2;
      # values
  		temp = [r.ip.dfsplant[jj][n.ocp.state.name[st1]] for jj in 1:L]
  		vals1 = [idx for tempM in temp for idx=tempM]

  		# values
  		temp = [r.ip.dfsplant[jj][n.ocp.state.name[st2]] for jj in 1:L]
  		vals2 = [idx for tempM in temp for idx=tempM]

      pp = obstaclePlot(n,c,ii;(:append=>false),(:posterPlot=>true)) # add obstacles
  		plot!(vals1,vals2,line=_pretty_defaults[:plant_lines][1],label="Vehicle Trajectory")
      #pp = plot(vals1,vals2,line=_pretty_defaults[:plant_lines][1],label="Vehicle Trajectory")
      #pp = obstaclePlot(n,c,ii,pp;(:append=>true),(:posterPlot=>true)) # add obstacles
      pp = vehiclePlot(n,c,ii,pp;(:append=>true),(:posterPlot=>true))  # add the vehicle
    else
      pp = obstaclePlot(n,c,ii,pp;(:append=>true),(:posterPlot=>true))  # add obstacles
      pp = vehiclePlot(n,c,ii,pp;(:append=>true),(:posterPlot=>true))  # add the vehicle
    end
  end
  l = @layout [a{0.5w} [grid(2,2)
                        b{0.2h}]]
  poster = plot(pp,sap,vt,longv,axp,tp,layout=l,size=_pretty_defaults[:size])
  savefig(string(r.resultsDir,"poster",".",_pretty_defaults[:format]))
  return nothing
end
