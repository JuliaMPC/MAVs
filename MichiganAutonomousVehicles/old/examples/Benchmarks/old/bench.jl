using MAVs, NLOptControl, BenchmarkTools, JLD, FileIO, DataFrames

#'''
#This function creates the benchmark tests
#'''
function bench(;baseBench::Bool=false)
  if baseBench
    name="master.jld"
  else
    name="pr.jld"
    master=load("master.jld", "results")
    m1=median(master["psMethods"])
  end
  # create a DataFrame
  fname = joinpath(dirname(@__FILE__), "data.jld")
  df = DataFrames.DataFrame()

  # Define a parent BenchmarkGroup to contain our suite
  const suite = BenchmarkGroup()

  # Add some child groups to our benchmark suite.
  suite["psMethods"] = BenchmarkGroup(["integrationScheme","Nck"])
  for scheme in (:lgrExplicit,:lgrImplicit)
    for Nck in ([10,8,6],[12,10,8,6])
      c=defineCase(;(:mode=>:autoBench));
      setMisc!(c;integrationScheme=scheme,Nck=Nck,max_cpu_time=20.0)
      n=initializeAutonomousControl(c);
      suite["psMethods"][scheme,Nck] = @benchmarkable optimize!($n) samples=10
    end
  end

  paramspath = joinpath(dirname(@__FILE__), "params.jld")
  if isfile(paramspath)
      loadparams!(suite, BenchmarkTools.load(paramspath, "suite"), :evals);
  else
      tune!(suite)
      BenchmarkTools.save(paramspath, "suite", params(suite));
  end
  results=run(suite,verbose=true);              # run all benchmarks
  BenchmarkTools.save(name, "results", results) # save results to JLD file

  m2=median(results["psMethods"]);

  # save the data
  file = jldopen(fname, "w")
  write(file, "df", df)
  close(file)

  if !baseBench
    return m2, judge(m2,m1)
  else
    return m2
  end
end

paramspath = joinpath(Pkg.dir("MAVs/examples/Benchmarks/"), "params.jld")

loadparams!(suite, BenchmarkTools.load(paramspath, "suite"), :evals);


master=load("master.jld", "results")

###########
file = joinpath(Pkg.dir("MAVs/examples/Benchmarks/"), "temp.jld")
res=master;
writeresults(file,res)
#https://github.com/JuliaCI/PkgBenchmark.jl/blob/master/src/runbenchmark.jl
function writeresults(file, res)
    save(File(format"JLD", file), "time", time(), "trials", res)
end

#https://github.com/JuliaCI/PkgBenchmark.jl/blob/master/src/runbenchmark.jl
function readresults(file)
    JLD.jldopen(file,"r") do f
        read(f, "trials")
    end
end

#=
if saveresults
               tosave = if promptsave
                   print("File results of this run? (commit=$(sha[1:6]), resultsdir=$resultsdir) (Y/n) ")
                   response = readline() |> strip
                   response == "" || lowercase(response) == "y"
               else true end
               if tosave
                   !isdir(resultsdir) && mkpath(resultsdir)
                   resfile = joinpath(resultsdir, sha*".jld")
                   writeresults(resfile, res)
                   info("Results of the benchmark were written to $resfile")
end
=#
#=
m=median(run(suite,verbose=true,seconds=10));
save("m2.jld","m",params(m));

# compare
tempDict=load("m1.jld")
m_temp=get(tempDict,"m1",0)

judge(m_temp,m)


##################OLD
#'''
# schemes=[:lgrExplicit,:lgrImplicit]
# m1=benchMark(schemes[1])
# m2=benchMark(schemes[2])
# judge(m1,m2)
#'''
function benchMark(scheme)
  c=defineCase(;(:mode=>:autoBench));
  setMisc!(c;integrationScheme=scheme)
  n=initializeAutonomousControl(c);
  b=@benchmark optimize!($n) samples=10
  return median(b)
end
=#
