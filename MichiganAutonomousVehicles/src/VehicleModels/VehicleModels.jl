module VehicleModels

using Parameters
using JuMP
using Interpolations
using OrdinaryDiffEq
using DiffEqBase
using NLOptControl  # to use newConstraint!() and interpolateLagrange!() for checkCrash()
using Plots
import Plots.@layout
 
# funcitons in the VehicleModels.jl package
include("parameters.jl")
include("Three_DOF/Three_DOF.jl")
include("KinematicBicycle/KinematicBicycle.jl")
include("utils.jl")
include("Plots/VehicleModels_plots.jl")

export
  #########
  # Objects
  #########
  Vpara,

  ###########
  # Functions
  ###########
  checkCrash,
  FNames,

  # Three DOF
  ThreeDOFv1,
  ThreeDOFv2,
  ThreeDOFv2_expr,
  ThreeDOFv3,
  ThreeDOFv3_expr,

  # KinematicBicycle
  KinematicBicycle,
  KinematicBicycle_expr,
  KinematicBicycle2,
  KinematicBicycle_expr2,

  ###############################
  # Macros and support functions
  ###############################
  # Three DOF
  @F_YF,
  @F_YR,
  @FZ_RL,
  @FZ_RR,
  @FZ_FL,
  @FZ_FR,
  @Ax_min,
  @Ax_max,

  # parameters
  @unpack_Vpara,
  @pack_Vpara,

  # Parameters.jl
  @unpack,
  @pack,

  # Plots
  obstaclePlot,
  trackPlot,
  mainSim,
  posterP,
  posPlot,
  vtPlot,
  pSimGR,
  axLimsPlot

end # module
