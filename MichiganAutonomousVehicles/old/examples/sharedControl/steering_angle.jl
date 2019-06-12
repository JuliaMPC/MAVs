using PrettyPlots, Plots
gr()
#pyplot()
#pgfplots()
# trying to match the driver steering angle
a1=-0.0206672256
a2=-0.0208976376

b1=-0.0064290878
b2=-0.0066664742

c1=0.0162199327
c2=0.0173577308

#L=n.numStatePoints;

L=linspace(0,6,60)
s0_1=c1;s0=c2;
s_k=zeros(L);
for k in 1:length(L)
  s_k[k]=s0+k*(s0-s0_1)
end

plot(L,s0*ones(L),label="s0")
plot!(L,s0_1*ones(L),label="s0_1")
plot!(L,s_k,label="s_k")

driver_obj=integrate!(mdl,n,r.x[:,6];D=sa_param,(:variable=>:control),(:integrand=>:squared),(:integrandAlgebra=>:subtract));
