
# steering angle model test
function threeDOFv1_test1(n)
  t0 = 0.0; tf = 20.0; t = Vector(linspace(t0,tf,100));
  U = zeros(length(t),2)
  sa(tt) = 0.15*sin(3*tt)
  U[:,1] = sa.(t)
  U[:,2] = 10.0
  X0 = [0.0, 0.0, 0.0, 0.0, 1.2038]
  sol, U = ThreeDOFv1(n,Vector(X0),t,U,t0,tf)
  actual = sol(sol.t[end])
  expected = [40.8714, 195.019, -0.170866, -0.0139167, 1.5131]
  A = zeros(length(actual))
  for i in 1:length(expected)
    A[i] = abs(expected[i]-actual[i])
  end
  return maximum(A)
end

# steering rate test
function threeDOFv2_test1(n)
  t0 = 0.0; tf = 20.0; t = Vector(linspace(t0,tf,100));
  U = zeros(length(t),2)
  sr(tt) = 0.45*cos(3*tt)
  U[:,1] = sr.(t)
  X0 = [0.0, 0.0, 0.0, 0.0, 1.2038, 0.0, 10.0, 0.0]
  sol, U = ThreeDOFv2(n,Vector(X0),t,U,t0,tf)
  actual = sol(sol.t[end])
  expected = [40.0552,195.17,-0.169685,-0.0114604,1.53446,-0.0439371,10.0,0.0]
  A = zeros(length(actual))
  for i in 1:length(expected)
    A[i] = abs(expected[i]-actual[i])
  end
  return maximum(A)
end

# constant speed test
function threeDOFv3_test1(n)
  t0 = 0.0; tf = 20.0; t = Vector(linspace(t0,tf,100));
  U = zeros(length(t),1)
  sr(tt) = 0.45*cos(3*tt)
  U[:,1] = sr.(t)
  X0 = [0.0, 0.0, 0.0, 0.0, 1.2038, 0.0, 10.0, 0.0]
  sol, U = ThreeDOFv3(n,Vector(X0),t,U,t0,tf)
  actual = sol(sol.t[end])
  expected = [24.2734,297.962,-1.18412,0.102708,1.70372,-0.0468365,10.0,0.0]
  A = zeros(length(actual))
  for i in 1:length(expected)
    A[i] = abs(expected[i]-actual[i])
  end
  return maximum(A)
end

@testset "threeDOF" begin
    n = NLOpt(); # constructor for test
    n.ocp.params = [Vpara()];
    @test threeDOFv1_test1(n) ≈ 0 atol=tol
    @test threeDOFv2_test1(n) ≈ 0 atol=tol
    @test threeDOFv3_test1(n) ≈ 0 atol=tol
end
