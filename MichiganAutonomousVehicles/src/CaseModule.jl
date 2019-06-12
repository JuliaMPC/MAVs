isdefined(Base, :__precompile__) && __precompile__()

module CaseModule

using DataFrames

export
      setConfig,
      case2dfs

"""
setConfig(c)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/25/2018, Last Modified: 2/25/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setConfig(c, key_name; kwargs...)
  for (key,value) in kwargs
    if haskey(c[key_name],string(key))
      c[key_name][string(key)]=value
    else
      error(string(" \n Unknown key: ", key, " for ", c[key_name], " used in setMisc() \n "))
    end
  end
  return c
end

"""
case2dfs(c)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 2/15/2018 \n
--------------------------------------------------------------------------------------\n
"""

function case2dfs(c)
    dfs = DataFrame()

    ##############
    # weights
    dfs[:wgoal] = c["weights"]["goal"]
    dfs[:wpsi] = c["weights"]["psi"] # 0
    dfs[:wtime] = c["weights"]["time"]
    dfs[:whaf] = c["weights"]["haf"] # 0
    dfs[:wce] = c["weights"]["ce"]
    dfs[:csa] = c["weights"]["sa"]
    dfs[:wsr] = c["weights"]["sr"]
    dfs[:wjx] = c["weights"]["jx"]
    dfs[:path] = c["weights"]["path"]
    dfs[:driver] = c["weights"]["driver"]
    # weights
    ##############

    ##############
    # tolerances
    # initial state tolerances
    dfs[:ixt] = c["tolerances"]["ix"]
    dfs[:iyt] = c["tolerances"]["iy"]
    dfs[:ivt] = c["tolerances"]["iv"]
    dfs[:irt] = c["tolerances"]["ir"]
    dfs[:ipsit] = c["tolerances"]["ipsi"]
    dfs[:isat] = c["tolerances"]["isa"]
    dfs[:iut] = c["tolerances"]["iu"]
    dfs[:iaxt] = c["tolerances"]["iax"]

    # final state tolerances
    dfs[:fxt] = c["tolerances"]["fx"]
    dfs[:fyt] = c["tolerances"]["fy"]
    dfs[:fvt] = c["tolerances"]["fv"]
    dfs[:frt] = c["tolerances"]["fr"]
    dfs[:fpsit] = c["tolerances"]["fpsi"]
    dfs[:fsat] = c["tolerances"]["fsa"]
    dfs[:fut] = c["tolerances"]["fu"]
    dfs[:faxt] = c["tolerances"]["fax"]
    # tolerances
    ##################

    #####################
    # solver settings
    dfs[:outlev] = c["solver"]["outlev"]
    dfs[:feastolAbs] = c["solver"]["feastol_abs"]
    dfs[:maxit] = c["solver"]["maxit"]
    dfs[:maxtimeCpu] = c["solver"]["maxtime_cpu"]
    dfs[:ftol] = c["solver"]["ftol"]
    dfs[:feastol] = c["solver"]["feastol"]
    dfs[:infeastol] = c["solver"]["infeastol"]
    dfs[:maxfevals] = c["solver"]["maxfevals"]
    dfs[:maxtimeReal] = c["solver"]["maxtime_real"]
    dfs[:opttol] = c["solver"]["opttol"]
    dfs[:opttolAbs] = c["solver"]["opttol_abs"]
    dfs[:xtol] = c["solver"]["xtol"]
    dfs[:acceptableObjChangeTol] = c["solver"]["acceptable_obj_change_tol"]
    dfs[:warmStartInitPoint] = c["solver"]["warm_start_init_point"]
    dfs[:dualInfTol] = c["solver"]["dual_inf_tol"]
    dfs[:acceptableTol] = c["solver"]["acceptable_tol"]
    dfs[:acceptableConstrViolTol] = c["solver"]["acceptable_constr_viol_tol"]
    dfs[:acceptableDualInfTol] = c["solver"]["acceptable_dual_inf_tol"]
    dfs[:acceptableComplInfTol] = c["solver"]["acceptable_compl_inf_tol"]
    dfs[:acceptableObjChangeTol] = c["solver"]["acceptable_obj_change_tol"]
    dfs[:divergingIteratesTol] = c["solver"]["diverging_iterates_tol"]
    # solver settings
    #####################

    ####################
    # obstacles
    dfs[:radius] = string(c["obstacle"]["radius"]')
    dfs[:sX] = string(c["obstacle"]["vx"]')
    dfs[:sy] = string(c["obstacle"]["vy"]')
    dfs[:Xi] = string(c["obstacle"]["x0"]')
    dfs[:Yi] = string(c["obstacle"]["y0"]')
    #dfs[:status] = string(c["obstacle"]["status"]')
    # obstacles
    ####################

    ###############
    # goal
    dfs[:xRef] = c["goal"]["x"]
    dfs[:yRef] = c["goal"]["yVal"]
    dfs[:psiRef] = c["goal"]["psi"]
    dfs[:goalTol] = c["goal"]["tol"]
    # goal
    ###############

    ###################
    # X0 parameters
    dfs[:xi] = c["X0"]["x"]
    dfs[:yi] = c["X0"]["yVal"]
    dfs[:vi] = c["X0"]["v"]
    dfs[:ri] = c["X0"]["r"]
    dfs[:psii] = c["X0"]["psi"]
    dfs[:sai] = c["X0"]["sa"]
    dfs[:ui] = c["X0"]["ux"]
    dfs[:axi] = c["X0"]["ax"]
    # X0 parameters
    ###################

    ###################
    # misc. parameters
    dfs[:model] = c["misc"]["model"]
    dfs[:Xmin] = c["misc"]["Xmin"]
    dfs[:Xmax] = c["misc"]["Xmax"]
    dfs[:Ymin] = c["misc"]["Ymin"]
    dfs[:Ymax] = c["misc"]["Ymax"]
    dfs[:tp] = c["misc"]["tp"]
    dfs[:tex] = c["misc"]["tex"]
    dfs[:sms] = c["misc"]["sm"]
    dfs[:smh] = c["misc"]["sm2"]
    dfs[:Lr] = c["misc"]["Lr"]
    dfs[:Lrd] = c["misc"]["L_rd"]

    if c["misc"]["integrationScheme"]==:lgrExplicit || c["misc"]["integrationScheme"]==:lgrImplicit
        dfs[:NI] = length(c["misc"]["Nck"])
        dfs[:colPts] = sum(c["misc"]["Nck"])
        dfs[:Nck] = string(c["misc"]["Nck"]')
    else
        dfs[:NI] = NaN
        dfs[:colPts] = c["misc"]["N"]
        dfs[:Nck] = NaN
    end
    dfs[:solver] = c["misc"]["solver"]
    dfs[:MPCmaxIter] = c["misc"]["mpc_max_iter"]
    dfs[:predictX0] = c["misc"]["PredictX0"]
    dfs[:fixedTp] = c["misc"]["FixedTp"]
    #dfs[:activeSafety] = c["misc"]["activeSafety"]
    #dfs[:followDriver] = c["misc"]["followDriver"]
    #dfs[:followPath] = c["misc"]["followPath"]
    #dfs[:NF] = c["misc"]["NF"]
    dfs[:integrationScheme] = c["misc"]["integrationScheme"]
    dfs[:tfMax] = c["misc"]["tfMax"]
    # misc. parameters
    ###################


    return dfs
end


end # module
