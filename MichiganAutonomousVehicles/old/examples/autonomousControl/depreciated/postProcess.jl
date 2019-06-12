using PrettyPlots

#gr()
#pgfplots()
a=2;
#pyplot(guidefont=font(a*17),tickfont=font(a*15),legendfont=font(a*12),titlefont=font(a*20))
#a=0.25;
#default(guidefont=font(a*17),tickfont=font(a*15),legendfont=font(a*12),titlefont=font(a*20))
plotSettings(;(:simulate=>true),(:mpc_markers =>(:circle,:blueviolet,0.0,0.0)),(:plant=>true),(:plantOnly=>false),(:size=>(a*900,a*600)),(:format=>"png"));

description=string(
"In this test: \n",c.m.name,"\n
* m.Nck=",c.m.Nck,"\n
* m.tp=",c.m.tp," \n
* m.tex=",c.m.tex,"\n
* m.max_cpu_time=",c.m.max_cpu_time," \n
* m.Nck=",c.m.Nck,"\n
* X0=",c.o.X0,"\n
* Y0=",c.o.Y0,"\n
* A=",c.o.A,"\n
* B=",c.o.B,"\n
* sx=",c.o.s_x,"\n
* sy=",c.o.s_y,"\n
")

results_dir=string("gazebo_zero_1",c.m.name,"/")
resultsDir!(n;results_name=results_dir,description=description);
savePlantData!(n)
if _pretty_defaults[:simulate];
  println("Plotting the Final Results!")
  mainSim(n,c;(:mode=>:open1))
end

optPlot(n)
posterP(n,c)


#plot(rand(10),xaxis=("test",(:guidefont=>20)))
