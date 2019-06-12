"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2017, Last Modified: 2/06/2018 \n
--------------------------------------------------------------------------------------\n
"""
function checkCrash(n; kwargs...)
    kw = Dict(kwargs)  # NOTE kwargs are depreciated

    c = n.ocp.params[5]
    sm = c["misc"]["sm"]
    if !haskey(kw,:plant); plant=true;
    else; plant = get(kw,:plant,0);
    end
    @unpack_Vpara n.ocp.params[1]

    if isequal(c["misc"]["model"],:ThreeDOFv2) || isequal(c["misc"]["model"],:ThreeDOFv3)
        # check to see if the minimum vertical tire load was exceeded
        Fz_off = c["vehicle"][:Fz_off]  # should be checked with mode
        if isequal(c["misc"]["model"],:ThreeDOFv2)
            V = n.r.ip.dfsplant[end][:v]
            U = n.r.ip.dfsplant[end][:ux]
            Ax = n.r.ip.dfsplant[end][:ax]
            R = n.r.ip.dfsplant[end][:r]
            SA = n.r.ip.dfsplant[end][:sa]
        elseif isequal(c["misc"]["model"],:ThreeDOFv3)
            V = n.r.ip.dfsplant[end][:v]
            U = u0_*ones(length(V),1)
            Ax = zeros(length(V),1)
            R = n.r.ip.dfsplant[end][:r]
            SA = n.r.ip.dfsplant[end][:sa]
        end

        if any(@FZ_RL() .< Fz_off) || any(@FZ_RR() .< Fz_off) || any(@FZ_FR() .< Fz_off) || any(@FZ_RL() .< Fz_off)
            println("The vertical tire force went below Fz_off")
            return true, :tireOff
        end
    end
    # check to see if the vehicle crashed into and obstacle
    if plant
        t = n.r.ip.plant[:t]
        X = n.r.ip.plant[:x]
        Y = n.r.ip.plant[:y]
        crash_tmp = zeros(length(c["obstacle"]["radius"]),1)
        for obs in 1:length(c["obstacle"]["radius"])
            # obstacle postions after the initial postion
            X_obs= c["obstacle"]["x0"][obs] .+ c["obstacle"]["vx"][obs].*t
            Y_obs= c["obstacle"]["y0"][obs] .+ c["obstacle"]["vy"][obs].*t
            if minimum((X-X_obs).^2./(c["obstacle"]["radius"][obs]+sm).^2 + (Y-Y_obs).^2./(c["obstacle"]["radius"][obs]+sm).^2) < 1
                crash_tmp[obs] = 1
                println("the vehicle crashed! \n")
            end
        end
        if maximum(crash_tmp)>0
            return true, :crash
        else
            return false, NaN
        end
    else # in the NLOptControl.jl paper, there was no plant and the following code was used
        # also this will break, need to remove pts from function handle
       pts = 300
       # redo interpolation with desired number of points
      if n.s.integrationMethod == :ps
          interpolateLagrange!(n;numPts=Int64(pts/n.Ni))
      else
          interpolateLinear!(n;numPts=pts)
      end
      X = n.r.X_pts[:,1]; Y = n.r.X_pts[:,2];

      crash_tmp = zeros(pts,1);
      for i = 1:pts
        temp = (X[i]-c.o.X0[1])^2/(c.o.B[1]+sm)^2 + (Y[i]-c.o.Y0[1])^2/(c.o.A[1]+sm)^2;
       if temp < 1
          crash_tmp[i] = 1;
       else
          crash_tmp[i] = 0;
       end
      end

      if maximum(crash_tmp)>0
          crash = 1;
          print("the vehicle crashed! \n")
      else
          crash = 0;
      end
     return crash
  end
end

"""
# this funtion can be used to get the fieldnames in a parameter set
# example
FNames(Vpara())
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/08/2018, Last Modified: 4/08/2018 \n
--------------------------------------------------------------------------------------\n
"""
function FNames(pa)
# To find fieldnames
r = ()
for i in 1:length(fieldnames(p))
      r = (r...,fieldnames(p)[i])
end

return r
end
