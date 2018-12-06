function exampleLSQ1

%Converted from the Cute Set AMPL model growthls.mod
%   Problem :
%   *********
%   GROWTH problem in 3 variables
%   Fit the observed growth g(n) from Gaussian Elimination
%   with complete pivoting to a function of the form
%
%        x(1) * n ** ( x(2) + LOG(n) * x(3) )
%   SIF input: Nick Gould, Nov, 1991, modified by Ph. Toint, March 1994.
%   classification SUR2-AN-3-0
%   Solution: [1.4571; 0.4450; 0.1633]

x0=ones(3,1);

extendedFeatures.OutputFcn = @outfun;
extendedFeatures.JacobPattern = ones(12,3);
options = optimset('Display','iter','ScaleProblem','none');
knitromatlab_lsqnonlin(@Fun,x0,[],[],extendedFeatures,options)

function F = Fun(x)

    % Uncomment the command below to ensure that Knitro output is flushed to 
    % the screen regularly (depending on the Display/outlev setting).  This 
    % may be desirable on large, difficult problems that take a while to solve.  
    %disp '';
    
	F(1)  = (x(1) * (8.0^(x(2)+(log(8.0))*x(3))) - 8.0);
	F(2)  = (x(1) * (9.0^(x(2)+(log(9.0))*x(3))) - 8.4305);
	F(3)  = (x(1) * (10.0^(x(2)+(log(10.0))*x(3))) - 9.5294);
	F(4)  = (x(1) * (11.0^(x(2)+(log(11.0))*x(3))) - 10.4627);
	F(5)  = (x(1) * (12.0^(x(2)+(log(12.0))*x(3))) - 12.0);
	F(6)  = (x(1) * (13.0^(x(2)+(log(13.0))*x(3))) - 13.0205);
	F(7)  = (x(1) * (14.0^(x(2)+(log(14.0))*x(3))) - 14.5949);
	F(8)  = (x(1) * (15.0^(x(2)+(log(15.0))*x(3))) - 16.1078);
	F(9)  = (x(1) * (16.0^(x(2)+(log(16.0))*x(3))) - 18.0596);
	F(10) = (x(1) * (18.0^(x(2)+(log(18.0))*x(3))) - 20.4569);
	F(11) = (x(1) * (20.0^(x(2)+(log(20.0))*x(3))) - 24.25);
	F(12) = (x(1) * (25.0^(x(2)+(log(25.0))*x(3))) - 32.9863);
	
function terminate = outfun(x,optimValues,state)
    xTranpose = x'
    terminate = false;


