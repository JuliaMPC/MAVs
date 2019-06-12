using NLOptControl
using VehicleModels
using MAVs

#s=Settings(;format=:png,MPC=false,save=true);
#TODO check settings -> some og thermare killing it
#=

julia> include("testPathFollowing.jl")
ERROR: LoadError: KNITRO: Error initializing problem
 in #init_problem#6(::Array{Float64,1}, ::Array{Float64,1}, ::Function, ::KNITRO.KnitroProblem, ::Int32, ::Int32, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}) at /home/febbo/.julia/v0.5/KNITRO/src/ktr_functions.jl:546
 in (::KNITRO.#kw##init_problem)(::Array{Any,1}, ::KNITRO.#init_problem, ::KNITRO.KnitroProblem, ::Int32, ::Int32, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}) at ./<missing>:0
 in #initializeProblem#2(::Array{Float64,1}, ::Ptr{Void}, ::Function, ::KNITRO.KnitroProblem, ::Int32, ::Int32, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}) at /home/febbo/.julia/v0.5/KNITRO/src/KNITRO.jl:97
 in (::KNITRO.#kw##initializeProblem)(::Array{Any,1}, ::KNITRO.#initializeProblem, ::KNITRO.KnitroProblem, ::Int32, ::Int32, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Float64,1}, ::Array{Float64,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}, ::Array{Int32,1}) at ./<missing>:0
 in optimize!(::KNITRO.KnitroMathProgModel) at /home/febbo/.julia/v0.5/KNITRO/src/KnitroSolverInterface.jl:229
 in #solvenlp#162(::Bool, ::Function, ::JuMP.Model, ::JuMP.ProblemTraits) at /home/febbo/.julia/v0.5/JuMP/src/nlp.jl:1271
 in (::JuMP.#kw##solvenlp)(::Array{Any,1}, ::JuMP.#solvenlp, ::JuMP.Model, ::JuMP.ProblemTraits) at ./<missing>:0
 in #solve#109(::Bool, ::Bool, ::Bool, ::Array{Any,1}, ::Function, ::JuMP.Model) at /home/febbo/.julia/v0.5/JuMP/src/solvers.jl:170
 in #optimize#15(::Int64, ::Function, ::JuMP.Model, ::NLOptControl.NLOpt, ::NLOptControl.Result, ::NLOptControl.Settings) at /home/febbo/.julia/v0.5/NLOptControl/src/utils.jl:287
 in initializePathFollowing(::MAVs.CaseModule.Case) at /home/febbo/.julia/v0.5/MAVs/src/PathFollowing.jl:140
 in include_from_node1(::String) at ./loading.jl:488
while loading /home/febbo/.julia/v0.5/MAVs/examples/pathFollowing/testPathFollowing.jl, in expression starting on line 11

=#
# initialize problem\
#c=defineCase(;(:mode=>:path));
c=defineCase(;(:mode=>:caseStudy));

mdl,n,r,params=initializePathFollowing(c);

optimize(mdl,n,r,s); # first run

# postProcess
using PrettyPlots, Plots
pyplot()
r.results_dir = string(r.main_dir,"/results/","TMP/");
resultsDir(r.results_dir);
allPlots(n,r,s,1);
pSimPath(n,r,s,c,1)
