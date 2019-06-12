using SymPy, Plots
gr()

#TODO consider looking into how Michal Kaveslov did path following -> discrete points (states), but how did he know which ones to choose??

# track data
type Track
  R
  x0
  y0
  Xn
  Yn
end

# EX4 test data -> https://www.varsitytutors.com/hotmath/hotmath_help/topics/shortest-distance-between-a-point-and-a-circle
function Track()
  Track(7,
        4,
       -5,
       -4,
       -11);
 end

function initTrack(;R=R,x0=x0,y0=y0,Xn=Xn,Yn=Yn)
  t=Track();
  t.R=R;
  t.x0=x0;
  t.y0=y0;
  t.Xn=Xn;
  t.Yn=Yn;
  return t
end

function minD(t::Track)
  sign(sqrt((t.Xn-t.x0)^2 + (t.Yn-t.y0)^2)-t.R)*(sqrt((t.Xn-t.x0)^2 + (t.Yn-t.y0)^2)-t.R);
end

function plotTrack(t::Track)
  pp=plot(0,leg=:false);
  title!(@sprintf("min D (m) = %0.2f",minD(t)))
  pts = Plots.partialcircle(0,2π,100,t.R);
  x, y = Plots.unzip(pts)
  x += t.x0;
  y += t.y0;
  pts = collect(zip(x, y));
  plot!(pts,line=(:path,5,:solid,:green),label="Track");
  XX=[t.Xn;t.x0];YY=[t.Yn;t.y0];
  plot!(XX,YY,line=(:path,5,:solid),label="D # ");

  pts = Plots.partialcircle(0,2π,100,0.25);
  x, y = Plots.unzip(pts);
  x += t.Xn;  y += t.Yn;
  pts = collect(zip(x, y));
  plot!(pts,c=:black,fill=:true,label="Vehicle")
  #xlabel!("X (m)");  ylabel!("Y (m)");
  #xlims!(-10,10); ylims!(-15,10); #TODO --> use this as a position plot
end

format=:png;

# ex4
t4=Track();
plotTrack(Track(t4));
savefig(string("t4.",format));

# ex3
t3=initTrack(;R=5,x0=-3,y0=3,Xn=-2,Yn=0);
plotTrack(t3);
savefig(string("t3.",format));

# real track
t1=initTrack(;R=100,x0=100,y0=100,Xn=0,Yn=0);
plotTrack(t1);
savefig(string("t1.",format));


# this script is designed to calcualte the analytical distance from a point to an elipse (if there is one...)
# not able to find the analytical solution

function elllipseD()
  a,b,x,y=symbols("a,b,x,y",positive=true);
  x0,y0,Xn,Yn=symbols("x0,y0,Xn,Yn",positive=true);
  D=symbols("D",positive=true);
  epsilon=symbols("epsilon",positive=true)
  # equation for distance
  exp1=(x-Xn)^2 + (y-Yn)^2 - D^2;

  # derivative of D w.r.t x == slope of D
  exp3=diff(exp1,x) - (y-Yn)/(x-Xn+sign(x-Xn)*epsilon);  # only Chuck Noris can divide by zero

  # equation for ellipse
  exp2=(((x-x0)/a)^2 + ((y-y0)/b)^2)-1;

  # both equations
  exp = [exp1, exp2, exp3];

  # solve the system
  solve(exp,[D,x])

  ####################
  # test the solution
  ####################
  # test data - > all variables and parameters must be positive
  A_=5; B_=10;
  x0_=5; y0_=20;
  Xn_=10; Yn_=0;

  # find the nearest point
  solve(exp1,)# solve for x

  pp=plot(0,leg=:false);

  A=5; B=10;
  x0=5; y0=20;
  Xn=10; Yn=0;

  pts = Plots.partialcircle(0,2π,100,A)
  x, y = Plots.unzip(pts)
  x += x0;
  y = B/A*y + y0;
  #pts = collect(zip(x, y))
  plot!(Shape(x,y), c=:red,leg=true,label="Obstacle",leg=:bottomleft)
  scatter!((Xn,Yn), markershape = :square, markercolor = :black, markersize = 4,fill = (0, 1, :black),leg=true, grid=true,label="Vehicle")
end
