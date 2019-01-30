using DataFrames, CSV
import YAML

function main(folder,case)
    N = 100 # number of points to plot
    pts = 4 # sample at several points in time

    dfsVeh = readtable(string(folder,"/state.csv"))
    dfsVehF = dfsVeh[1:Int32(floor(size(dfsVeh)[1]/N,0)):end,:]
    CSV.write(string(folder,"/stateFiltered.csv"), dfsVehF; quotechar = ' ')
    tV = dfsVehF[:t]
    xV = dfsVehF[:x]
    yV = dfsVehF[:y]
    psiV = dfsVehF[:psi]

    # load the case data
    data = YAML.load(open(string("/home/mavs/MAVs/ros/src/system/config/case/",case,".yaml")))

    # goal
    x = data["case"]["goal"]
    dfs = DataFrame(Any[values(x)...],Symbol[map(Symbol,keys(x))...])
    CSV.write(string(folder,"/goal.csv"), dfs; quotechar = ' ')

    # initial state
    x = data["case"]["actual"]["X0"]
    dfs = DataFrame(Any[values(x)...],Symbol[map(Symbol,keys(x))...])
    CSV.write(string(folder,"/initialState.csv"), dfs; quotechar = ' ')

    # obstacles
    x = data["case"]["actual"]["obstacle"]
    dfs = DataFrame(Any[values(x)...],Symbol[map(Symbol,keys(x))...])
    CSV.write(string(folder,"/obstacles.csv"), dfs; quotechar = ' ')

    # get the obstacle data
    R = data["case"]["actual"]["obstacle"]["radius"]
    X0 = data["case"]["actual"]["obstacle"]["x0"]
    Y0 = data["case"]["actual"]["obstacle"]["y0"]
    Vx = data["case"]["actual"]["obstacle"]["vx"]
    Vy = data["case"]["actual"]["obstacle"]["vy"]

    ################################################
    # save obstacle and vehicle data for ploting
    t = Vector(linspace(0,tV[end],pts)) # assuming it starts at 0!
    #t[3] = 19.5 # for s9_D2 to line up crash
    #indmin(abs.(t[2]-dfsVehF[:t])

    xv = zeros(pts)
    yv = zeros(pts)
    psi = zeros(pts)
    dfs = DataFrame()
    for j in 1:pts
      idx = indmin(abs.(t[j]-tV))
      xv[j] = xV[idx]
      yv[j] = yV[idx]
      psi[j] = psiV[idx]
    end
    dfs[:idx] = 1:pts
    dfs[:t] = round.(t,1,10) # round for graph
    dfs[:xv] = xv
    dfs[:yv] = yv
    dfs[:psi] = psi
    CSV.write(string(folder,"/veh.csv"), dfs; quotechar = ' ')

    for i in 1:length(R)
      xo = zeros(pts)
      yo = zeros(pts)
      ro = zeros(pts)
      clr = zeros(pts)
      dfs = DataFrame()
      for j in 1:pts
        xo[j] = X0[i] + Vx[i]*t[j]
        yo[j] = Y0[i] + Vy[i]*t[j]
        ro[j] = R[i]
        clr[j] = (pts-j-1)/pts*100
      end
      dfs[:idx] = 1:pts
      dfs[:t] = round.(t,1,10) # round for graph
      dfs[:xo] = xo
      dfs[:yo] = yo
      dfs[:ro] = ro
      dfs[:clr] = clr
      CSV.write(string(folder,"/obs_",i,".csv"), dfs; quotechar = ' ')
    end
    # save obstacle and vehicle data for ploting
    ################################################
end
#@show f = ARGS[1]
#c = ARGS[2]
#folder = parse(join(f," "))
#case = parse(join(c," "))
#main(folder,case)
main(ARGS[1],ARGS[2])
