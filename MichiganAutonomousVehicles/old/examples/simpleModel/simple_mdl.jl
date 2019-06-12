# start julia with: julia -p 4

using MAVs
using VehicleModels

mdl,n,c,r,params=initializeAutonomousControl(:c3);
pa=params[1];

function createParallelModels(n,c,pa)

#  http://docs.julialang.org/en/stable/manual/parallel-computing/#clustermanagers
  #addprocs(3);
  Xmin=[0.,180.,220.];
  #sm1=remotecall(initializeSimpleModel,2,(n,c,pa;x_min=Xmin[1]));
  #sm2=remotecall(initializeSimpleModel,3,(n,c,pa;x_min=Xmin[2]));
  #sm3=remotecall(initializeSimpleModel,4,(n,c,pa;x_min=Xmin[3]));
  sm1=@spawn initializeSimpleModel(n,c,pa;x_min=Xmin[1])
  sm2=@spawn initializeSimpleModel(n,c,pa;x_min=Xmin[2])
  sm3=@spawn initializeSimpleModel(n,c,pa;x_min=Xmin[3])

  sm=[sm1,sm2,sm3];
  return sm
end

sm=createParallelModels(n,c,pa);


#https://github.com/JuliaParallel/ClusterManagers.jl

#https://juliacomputing.com/domains/parallel-computing.html
#https://github.com/JuliaParallel
