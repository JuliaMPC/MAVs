# Three DOF Vehicle Model
include("F_YR.jl")
include("F_YF.jl")
include("FZ_RL.jl")
include("FZ_RR.jl")
include("FZ_FL.jl")
include("FZ_FR.jl")
include("Ax_max.jl")
include("Ax_min.jl")

"""
ThreeDOFv2_expr(n) & sol, U = ThreeDOFv1(n,x,u)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 10/20/2017, Last Modified: 5/31/2017 \n
--------------------------------------------------------------------------------------\n
# this vehicle model is controlled using steering angle and speed
"""
function ThreeDOFv1{T<:Any}(n,x::Array{T,2},u::Array{T,2})
  if n.s.integrationMethod==:tm; L=size(x)[1]; else; L=size(x)[1]-1; end
  dx = Array{Any}(L,n.numStates)
  v = x[:,3]; r = x[:,4]; psi = x[:,5];

  # parameters
  ax = zeros(length(psi)); sa = u[:,1];
  pa=params[1]; UX=params[2];
  ux=getvalue(UX)*ones(L,1);           # assume UX is a constant
  @unpack_Vpara n.ocp.params[1]            # vehicle parameters

  # nonlinear tire model
  FYF=@NLexpression(n.ocp.mdl, [i = 1:L], (PD2*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*(FzF0 - (ax[i] - v[i]*r[i])*KZX))*sin(PC1*atan((((PK1*sin(2*atan(PK2*(FzF0 - (ax[i] - v[i]*r[i])*KZX))))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)) + ((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX))^2 + EP^2)^(0.5))*0.001)+EP))*((atan((v[i] + la*r[i])/(ux[i]+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*KZX) + PH1)) - ((PE2*(FzF0 - (ax[i] - v[i]*r[i])*KZX) + PE1)*(1 - PE3)*(((atan((v[i] + la*r[i])/(ux[i]+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*KZX) + PH1))/((((atan((v[i] + la*r[i])/(ux[i]+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*KZX) + PH1)^2 + EP^2)^(0.5)))*((((PK1*sin(2*atan(PK2*(FzF0 - (ax[i] - v[i]*r[i])*KZX))))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)) + ((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX))^2 + EP^2)^(0.5))*0.001)+EP))*((atan((v[i] + la*r[i])/(ux[i]+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*KZX) + PH1)) - atan((((PK1*sin(2*atan(PK2*(FzF0 - (ax[i] - v[i]*r[i])*KZX))))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)) + ((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)))/(((PD2*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzF0 - (ax[i] - v[i]*r[i])*KZX))^2 + EP^2)^(0.5))*0.001)+EP))*((atan((v[i] + la*r[i])/(ux[i]+EP)) - sa[i]) + PH2*(FzF0 - (ax[i] - v[i]*r[i])*KZX) + PH1)))))) + (PV2*(FzF0 - (ax[i] - v[i]*r[i])*KZX)^2 + PV1*(FzF0 - (ax[i] - v[i]*r[i])*KZX)));
  FYR=@NLexpression(n.ocp.mdl, [i = 1:L], (PD2*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*(FzR0 + (ax[i] - v[i]*r[i])*KZX))*sin(PC1*atan((((PK1*sin(2*atan(PK2*(FzR0 + (ax[i] - v[i]*r[i])*KZX))))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)) + ((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX))^2+EP^2)^(0.5))*0.001)+EP))*((atan((v[i] - lb*r[i])/(ux[i]+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*KZX) + PH1)) - ((PE2*(FzR0 + (ax[i] - v[i]*r[i])*KZX) + PE1)*(1 - PE3*(((atan((v[i] - lb*r[i])/(ux[i]+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*KZX) + PH1))/((((atan((v[i] - lb*r[i])/(ux[i]+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*KZX) + PH1)^2 + EP^2)^(0.5))))*((((PK1*sin(2*atan(PK2*(FzR0 + (ax[i] - v[i]*r[i])*KZX))))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)) + ((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX))^2+EP^2)^(0.5))*0.001)+EP))*((atan((v[i] - lb*r[i])/(ux[i]+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*KZX) + PH1)) - atan((((PK1*sin(2*atan(PK2*(FzR0 + (ax[i] - v[i]*r[i])*KZX))))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)) + ((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)))/(((PD2*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PD1*PC1*(FzR0 + (ax[i] - v[i]*r[i])*KZX))^2+EP^2)^(0.5))*0.001)+EP))*((atan((v[i] - lb*r[i])/(ux[i]+EP))) + PH2*(FzR0 + (ax[i] - v[i]*r[i])*KZX) + PH1)))))) + (PV2*(FzR0 + (ax[i] - v[i]*r[i])*KZX)^2 + PV1*(FzR0 + (ax[i] - v[i]*r[i])*KZX)));

  # vertical tire load
  FZ_rl_con=@NLconstraint(n.ocp.mdl, [i=1:L], 0 <=  0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) - KZYR*((FYF[i] + FYR[i])/m) - Fz_min)
  newConstraint!(n,FZ_rl_con,:FZ_rl_con);
  FZ_rr_con=@NLconstraint(n.ocp.mdl, [i=1:L], 0 <=  0.5*(FzR0 + KZX*(ax[i] - v[i]*r[i])) + KZYR*((FYF[i] + FYR[i])/m) - Fz_min)
  newConstraint!(n,FZ_rr_con,:FZ_rr_con);

  # linear tire and for now this also constrains the nonlinear tire model
  Fyf_con=@NLconstraint(n.ocp.mdl, [i=1:L], Fyf_min <=  (atan((v[i] + la*r[i])/(ux[i]+EP)) - sa[i])*Caf <= Fyf_max);
  newConstraint!(n,Fyf_con,:Fyf_con);
  Fyr_con=@NLconstraint(n.ocp.mdl, [i=1:L], Fyf_min <=   atan((v[i] - lb*r[i])/(ux[i]+EP))*Car <= Fyf_max);
  newConstraint!(n,Fyr_con,:Fyr_con);

  dx[:,1] = @NLexpression(n.ocp.mdl, [i=1:L], ux[i]*cos(psi[i]) - (v[i] + la*r[i])*sin(psi[i]));    # X position
  dx[:,2] = @NLexpression(n.ocp.mdl, [i=1:L], ux[i]*sin(psi[i]) + (v[i] + la*r[i])*cos(psi[i]));    # Y position
  dx[:,3] = @NLexpression(n.ocp.mdl, [i=1:L], (FYF[i] + FYR[i])/m - r[i]*ux[i]);                    # Lateral Speed
  dx[:,4] = @NLexpression(n.ocp.mdl, [i=1:L], (la*FYF[i]-lb*FYR[i])/Izz);                           # Yaw Rate
  dx[:,5] = @NLexpression(n.ocp.mdl, [i=1:L], r[i]);                                                # Yaw Angle
  return dx
end
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 10/01/2016, Last Modified: 6/22/2017 \n
--------------------------------------------------------------------------------------\n
"""
function ThreeDOFv1(n,
                    x0::Vector,
                     t::Vector,
                     U::Matrix,
                    t0::Float64,
                    tf::Float64)

    @unpack_Vpara n.ocp.params[1]
    # create splines
    sp_SA = linearSpline(t,U[:,1])
    sp_U = linearSpline(t,U[:,2])

    f = (dx,x,p,t) -> begin
    # states
    V   = x[3]  # 3. Lateral Speed
    R   = x[4]  # 4. Yaw Rate
    PSI = x[5]  # 5. Yaw angle

    # controls
    SA  = sp_SA[t] # Steering Angle
    U  = sp_U[t]   # Longitudinal Speed

    # set variables for tire equations
    Ax = 0

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*R)*sin(PSI)    # X position
    dx[2] 	= U*sin(PSI) + (V + la*R)*cos(PSI)    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - R*U         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz         # Yaw Rate
    dx[5]  	= R                                   # Yaw Angle
  end
  tspan = (t0,tf)
  prob = ODEProblem(f,x0,tspan)

  sol = DiffEqBase.solve(prob,Tsit5())
  U = [sp_SA,sp_U]
  return sol, U
end

"""
ThreeDOFv2_expr(n) & sol, U = ThreeDOFv2(n,x,u)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 10/20/2017, Last Modified:  5/31/2017 \n
--------------------------------------------------------------------------------------\n
# this vehicle model is controlled using steering rate and longitudinal jerk

"""
function ThreeDOFv2_expr(n)

  @unpack_Vpara n.ocp.params[1]

  # lateral tire load
  FYF=:(($PD2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))*sin($PC1*atan(((($PK1*sin(2*atan($PK2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))^2 + $EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] + $la*r[j])/(ux[j]+$EP)) - sa[j]) + $PH2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX) + $PH1)) - (($PE2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX) + $PE1)*(1 - $PE3)*(((atan((v[j] + $la*r[j])/(ux[j]+$EP)) - sa[j]) + $PH2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX) + $PH1))/((((atan((v[j] + $la*r[j])/(ux[j]+$EP)) - sa[j]) + $PH2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX) + $PH1)^2 + $EP^2)^(0.5)))*(((($PK1*sin(2*atan($PK2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))^2 + $EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] + $la*r[j])/(ux[j]+$EP)) - sa[j]) + $PH2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX) + $PH1)) - atan(((($PK1*sin(2*atan($PK2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX))^2 + $EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] + $la*r[j])/(ux[j]+$EP)) - sa[j]) + $PH2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX) + $PH1)))))) + ($PV2*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)^2 + $PV1*($FzF0 - (ax[j] - v[j]*r[j])*$KZX)));
  FYR=:(($PD2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))*sin($PC1*atan(((($PK1*sin(2*atan($PK2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))^2+$EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] - $lb*r[j])/(ux[j]+$EP))) + $PH2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX) + $PH1)) - (($PE2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX) + $PE1)*(1 - $PE3*(((atan((v[j] - $lb*r[j])/(ux[j]+$EP))) + $PH2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX) + $PH1))/((((atan((v[j] - $lb*r[j])/(ux[j]+$EP))) + $PH2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX) + $PH1)^2 + $EP^2)^(0.5))))*(((($PK1*sin(2*atan($PK2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))^2+$EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] - $lb*r[j])/(ux[j]+$EP))) + $PH2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX) + $PH1)) - atan(((($PK1*sin(2*atan($PK2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX))^2+$EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] - $lb*r[j])/(ux[j]+$EP))) + $PH2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX) + $PH1)))))) + ($PV2*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)^2 + $PV1*($FzR0 + (ax[j] - v[j]*r[j])*$KZX)));

  dx = Array{Expr}(8)
  dx[1] = :(ux[j]*cos(psi[j]) - (v[j] + $la*r[j])*sin(psi[j]))   # X position
  dx[2] = :(ux[j]*sin(psi[j]) + (v[j] + $la*r[j])*cos(psi[j]))   # Y position
  dx[3] = :(($FYF + $FYR)/$m - r[j]*ux[j])                       # Lateral Speed
  dx[4] = :(($la*$FYF-$lb*$FYR)/$Izz)                            # Yaw Rate
  dx[5] = :(r[j])                                                # Yaw Angle
  dx[6] = :(sr[j])                                               # Steering Angle
  dx[7] = :(ax[j])                                               # Longitudinal Speed
  dx[8] = :(jx[j])                                               # Longitudinal Acceleration

  # rear vertical tire load
  FZ_RL=:(0.5*($FzR0 + $KZX*(ax[j] - v[j]*r[j])) - $KZYR*(($FYF + $FYR)/$m))
  FZ_RR=:(0.5*($FzR0 + $KZX*(ax[j] - v[j]*r[j])) + $KZYR*(($FYF + $FYR)/$m))
  FZ_RL_con=:(0 <= $FZ_RL - $Fz_min)
  FZ_RR_con=:(0 <= $FZ_RR - $Fz_min)

  # front vertical tire load
  FZ_FL=:(0.5*($FzR0 - $KZX*(ax[j] - v[j]*r[j])) - $KZYF*(($FYF + $FYR)/$m))
  FZ_FR=:(0.5*($FzR0 - $KZX*(ax[j] - v[j]*r[j])) + $KZYF*(($FYF + $FYR)/$m))
  FZ_FL_con=:(0 <= $FZ_FL - $Fz_min)
  FZ_FR_con=:(0 <= $FZ_FR - $Fz_min)

  # linear tire forces
  Fyf_linear=:($Fy_min <= (atan((v[j] + $la*r[j])/(ux[j]+$EP)) - sa[j])*$Caf <= $Fy_max);
  Fyr_linear=:($Fy_min <= atan((v[j] - $lb*r[j])/(ux[j]+$EP))*$Car <= $Fy_max);

  # nonlinear accleleration bounds
  min_ax=:(0 <= ax[j] - ($AXC[5]*ux[j]^3 + $AXC[6]*ux[j]^2 + $AXC[7]*ux[j] + $AXC[8]))
  max_ax=:(ax[j] - ($AXC[1]*ux[j]^3 + $AXC[2]*ux[j]^2 + $AXC[3]*ux[j] + $AXC[4]) <= 0)

  con=[FZ_RL_con,FZ_RR_con,FZ_FL_con,FZ_FR_con,Fyf_linear,Fyr_linear,min_ax,max_ax]

  # expression for cost function
  tire_expr = :(2 + tanh(-($FZ_RL - $a_t)/$b_t) + tanh(-($FZ_RR - $a_t)/$b_t))

  return dx,con,tire_expr
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 10/01/2016, Last Modified: 6/22/2017 \n
--------------------------------------------------------------------------------------\n
"""
function ThreeDOFv2(n,
                   x0::Vector,
                   t::Vector,
                   U::Matrix,
                   t0::Float64,
                   tf::Float64)
    if length(t)!=size(U)[1]
      error(" \n The length of the time vector must match the length of the control input \n")
    end

    @unpack_Vpara n.ocp.params[1]

    # create splines
    sp_SR=linearSpline(t,U[:,1])
    sp_Jx=linearSpline(t,U[:,2])

    f = (dx,x,p,t) -> begin

    # states
    V   = x[3]  # 3. Lateral Speed
    R   = x[4]  # 4. Yaw Rate
    PSI = x[5]  # 5. Yaw angle
    SA  = x[6]  # 6. Steering Angle
    U   = x[7]  # 7. Longitudinal Speed
    Ax  = x[8]  # 8. Longitudinal Acceleration

    # controls
    SR  = sp_SR[t]
    Jx  = sp_Jx[t]

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*R)*sin(PSI)    # X position
    dx[2] 	= U*sin(PSI) + (V + la*R)*cos(PSI)    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - R*U         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz         # Yaw Rate
    dx[5]  	= R                                   # Yaw Angle
    dx[6]   = SR                                  # Steering Angle
    dx[7]  	= Ax                                  # Longitudinal Speed
    dx[8]  	= Jx                                  # Longitudinal Acceleration
  end
  tspan = (t0,tf)
  prob = ODEProblem(f,x0,tspan)
  sol = DiffEqBase.solve(prob,Tsit5())
  U = [sp_SR,sp_Jx]
  return sol, U
end

"""
ThreeDOFv3_expr(n) & sol, U = ThreeDOFv3(n,x,u)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 11/3/2018, Last Modified:  11/3/2018 \n
--------------------------------------------------------------------------------------\n
# this vehicle model is controlled using steering rate and it has a constant speed
"""
function ThreeDOFv3_expr(n)

    @unpack_Vpara n.ocp.params[1]
    ux = n.ocp.params[5]["X0"]["ux"];
    # lateral tire load
    FYF=:(($PD2*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*($FzF0 - ( - v[j]*r[j])*$KZX))*sin($PC1*atan(((($PK1*sin(2*atan($PK2*($FzF0 - ( - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX))^2 + $EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] + $la*r[j])/($ux+$EP)) - sa[j]) + $PH2*($FzF0 - ( - v[j]*r[j])*$KZX) + $PH1)) - (($PE2*($FzF0 - ( - v[j]*r[j])*$KZX) + $PE1)*(1 - $PE3)*(((atan((v[j] + $la*r[j])/($ux+$EP)) - sa[j]) + $PH2*($FzF0 - ( - v[j]*r[j])*$KZX) + $PH1))/((((atan((v[j] + $la*r[j])/($ux+$EP)) - sa[j]) + $PH2*($FzF0 - ( - v[j]*r[j])*$KZX) + $PH1)^2 + $EP^2)^(0.5)))*(((($PK1*sin(2*atan($PK2*($FzF0 - ( - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX))^2 + $EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] + $la*r[j])/($ux+$EP)) - sa[j]) + $PH2*($FzF0 - ( - v[j]*r[j])*$KZX) + $PH1)) - atan(((($PK1*sin(2*atan($PK2*($FzF0 - ( - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzF0 - ( - v[j]*r[j])*$KZX))^2 + $EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] + $la*r[j])/($ux+$EP)) - sa[j]) + $PH2*($FzF0 - ( - v[j]*r[j])*$KZX) + $PH1)))))) + ($PV2*($FzF0 - ( - v[j]*r[j])*$KZX)^2 + $PV1*($FzF0 - ( - v[j]*r[j])*$KZX)));
    FYR=:(($PD2*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*($FzR0 + ( - v[j]*r[j])*$KZX))*sin($PC1*atan(((($PK1*sin(2*atan($PK2*($FzR0 + ( - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX))^2+$EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] - $lb*r[j])/($ux+$EP))) + $PH2*($FzR0 + ( - v[j]*r[j])*$KZX) + $PH1)) - (($PE2*($FzR0 + ( - v[j]*r[j])*$KZX) + $PE1)*(1 - $PE3*(((atan((v[j] - $lb*r[j])/($ux+$EP))) + $PH2*($FzR0 + ( - v[j]*r[j])*$KZX) + $PH1))/((((atan((v[j] - $lb*r[j])/($ux+$EP))) + $PH2*($FzR0 + ( - v[j]*r[j])*$KZX) + $PH1)^2 + $EP^2)^(0.5))))*(((($PK1*sin(2*atan($PK2*($FzR0 + ( - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX))^2+$EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] - $lb*r[j])/($ux+$EP))) + $PH2*($FzR0 + ( - v[j]*r[j])*$KZX) + $PH1)) - atan(((($PK1*sin(2*atan($PK2*($FzR0 + ( - v[j]*r[j])*$KZX))))/((($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)) + (($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)))/((($PD2*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PD1*$PC1*($FzR0 + ( - v[j]*r[j])*$KZX))^2+$EP^2)^(0.5))*0.001)+$EP))*((atan((v[j] - $lb*r[j])/($ux+$EP))) + $PH2*($FzR0 + ( - v[j]*r[j])*$KZX) + $PH1)))))) + ($PV2*($FzR0 + ( - v[j]*r[j])*$KZX)^2 + $PV1*($FzR0 + ( - v[j]*r[j])*$KZX)));

    dx = Array{Expr}(6)
    dx[1] = :($ux*cos(psi[j]) - (v[j] + $la*r[j])*sin(psi[j]))   # X position
    dx[2] = :($ux*sin(psi[j]) + (v[j] + $la*r[j])*cos(psi[j]))   # Y position
    dx[3] = :(($FYF + $FYR)/$m - r[j]*$ux)                       # Lateral Speed
    dx[4] = :(($la*$FYF-$lb*$FYR)/$Izz)                            # Yaw Rate
    dx[5] = :(r[j])                                                # Yaw Angle
    dx[6] = :(sr[j])                                               # Steering Angle

    ax = 0.0;
    # rear vertical tire load
    FZ_RL=:(0.5*($FzR0 + $KZX*( - v[j]*r[j])) - $KZYR*(($FYF + $FYR)/$m))
    FZ_RR=:(0.5*($FzR0 + $KZX*( - v[j]*r[j])) + $KZYR*(($FYF + $FYR)/$m))
    FZ_RL_con=:(0 <= $FZ_RL - $Fz_min)
    FZ_RR_con=:(0 <= $FZ_RR - $Fz_min)

    # front vertical tire load
    FZ_FL=:(0.5*($FzR0 - $KZX*( - v[j]*r[j])) - $KZYF*(($FYF + $FYR)/$m))
    FZ_FR=:(0.5*($FzR0 - $KZX*( - v[j]*r[j])) + $KZYF*(($FYF + $FYR)/$m))
    FZ_FL_con=:(0 <= $FZ_FL - $Fz_min)
    FZ_FR_con=:(0 <= $FZ_FR - $Fz_min)

    # linear tire forces
    Fyf_linear=:($Fy_min <= (atan((v[j] + $la*r[j])/($ux+$EP)) - sa[j])*$Caf <= $Fy_max);
    Fyr_linear=:($Fy_min <= atan((v[j] - $lb*r[j])/($ux+$EP))*$Car <= $Fy_max);

    con=[FZ_RL_con,FZ_RR_con,FZ_FL_con,FZ_FR_con,Fyf_linear,Fyr_linear]

    # expression for cost function
    tire_expr = :(2 + tanh(-($FZ_RL - $a_t)/$b_t) + tanh(-($FZ_RR - $a_t)/$b_t))

  return dx,con,tire_expr
end

"""
# NOTE could simplify even more and modify tire force equation to remove Ax
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 11/3/2018, Last Modified:  11/3/2018 \n
--------------------------------------------------------------------------------------\n
"""
function ThreeDOFv3(n,
                   x0::Vector,
                   t::Vector,
                   U::Matrix,
                   t0::Float64,
                   tf::Float64)
    if length(t)!=size(U)[1]
      error(" \n The length of the time vector must match the length of the control input \n")
    end

    @unpack_Vpara n.ocp.params[1]

    # create spline
    sp_SR=linearSpline(t,U[:,1])

    f = (dx,x,p,t) -> begin

    # states
    V   = x[3]  # 3. Lateral Speed
    R   = x[4]  # 4. Yaw Rate
    PSI = x[5]  # 5. Yaw angle
    SA  = x[6]  # 6. Steering Angle

    # params
    U   = u0_  # Longitudinal Speed
    Ax  = 0    #  Longitudinal Acceleration

    # controls
    SR  = sp_SR[t]

    # diff eqs.
    dx[1]   = U*cos(PSI) - (V + la*R)*sin(PSI)    # X position
    dx[2] 	= U*sin(PSI) + (V + la*R)*cos(PSI)    # Y position
    dx[3]   = (@F_YF() + @F_YR())/m - R*U         # Lateral Speed
    dx[4]  	= (la*@F_YF()-lb*@F_YR())/Izz         # Yaw Rate
    dx[5]  	= R                                   # Yaw Angle
    dx[6]   = SR                                  # Steering Angle
  end
  tspan = (t0,tf)
  prob = ODEProblem(f,x0,tspan)
  sol = DiffEqBase.solve(prob,Tsit5())
  U = [sp_SR]  # NOTE poor choice of variable names to use U twice.
  return sol, U
end
