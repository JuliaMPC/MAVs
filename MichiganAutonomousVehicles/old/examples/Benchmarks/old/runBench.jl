using BenchmarkTools, JLD

# Define a parent BenchmarkGroup to contain our suite
#const suite = BenchmarkGroup()
#suite["psMethods"] = BenchmarkGroup(["integrationScheme","Nck"])

paramspath = joinpath(dirname(@__FILE__), "params.jld")

loadparams!(suite, BenchmarkTools.load(paramspath, "suite"), :evals, :samples);
results = run(suite; verbose = true) # run all benchmarks
BenchmarkTools.save("pr.jld", "results", results)   # save results to JLD file

master = load("master.jld", "results")
pr = load("pr.jld", "results")
regs = regressions(judge(minimum(pr), minimum(master))) # a BenchmarkGroup containing the regressions
pairs = leaves(regs) # an array of (ID, `TrialJudgement`) pairs


m1=median(suite["psMethods"])
