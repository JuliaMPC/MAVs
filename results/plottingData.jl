using DataFrames, CSV

N = 100 # number of points to plot
pts = 4 # sample at several points in time

dfsVeh = readtable("state.csv")
dfsVehF = dfsVeh[1:Int32(floor(size(dfsVeh)[1]/N,0)):end,:]
CSV.write("stateFiltered.csv", dfsVehF; quotechar = ' ')

tV = dfsVehF[:t]
xV = dfsVehF[:x]
yV = dfsVehF[:y]
psiV = dfsVehF[:psi]

# get the obstacle data
dfsCase = readtable("case.csv")
R = eval(parse(dfsCase[:actualObstacleRadius][1]))
X0 = eval(parse(dfsCase[:actualObstacleX0][1]))
Y0 = eval(parse(dfsCase[:actualObstacleY0][1]))
Vx = eval(parse(dfsCase[:actualObstacleVx][1]))
Vy = eval(parse(dfsCase[:actualObstacleVy][1]))


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
CSV.write("veh.csv", dfs; quotechar = ' ')

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
  CSV.write(string("obs_",i,".csv"), dfs; quotechar = ' ')
end
# save obstacle and vehicle data for ploting
################################################
