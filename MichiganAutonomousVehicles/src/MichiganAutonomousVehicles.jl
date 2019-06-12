isdefined(Base, :__precompile__) && __precompile__()

module MichiganAutonomousVehicles
using NLOptControl
import YAML.load

include("VehicleModels/VehicleModels.jl")
using .VehicleModels
using Parameters # needed to export @unpack and @pack

include("CaseModule.jl")
using .CaseModule

include("AutonomousControl.jl")
using .AutonomousControl

export
    # CaseModule.jl
    setConfig,
    case2dfs,

    # AutonomousControl.jl
    initializeAutonomousControl,
    updateAutoParams!,
    avMpc,
    solverConfig,
    fixYAML,
    load, # from YAML
    configProb!,
    obstacleAvoidanceConstraints!,
    lidarConstraints!,
    objFunc!,
    goalRange,

    # VehicleModels.jl
    # Objects
    ###########
    Vpara,

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
end
