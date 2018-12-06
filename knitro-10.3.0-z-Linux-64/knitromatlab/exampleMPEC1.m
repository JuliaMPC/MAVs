function exampleMPEC1
%--------------------------------------------------------------------------
% Copyright (c) 2016 by Artelys
% All Rights Reserved.
%  
%
%  min   (x1 - 5)^2 + (2 x2 + 1)^2
%  s.t.  -1.5 x1 + 2 x2 + x3 - 0.5 x4 + x5 = 2
%        x3 complements (3 x1 - x2 - 3)
%        x4 complements (-x1 + 0.5 x2 + 4)
%        x5 complements (-x1 - x2 + 7)
%        x1, x2, x3, x4, x5 >= 0
%        
%  The complementarity constraints must be converted so that one nonnegative
%  variable complements another nonnegative variable.
%
%  The solution is (1, 0, 3.5, 0, 0, 0, 3, 6), with objective value 17.
%--------------------------------------------------------------------------

Jpattern = [];

Hpattern = sparse(zeros(8));
Hpattern(1,1) = 1;
Hpattern(2,2) = 1;

options = optimset('Algorithm', 'interior-point', 'Display','iter', ...
     'GradObj','on','GradConstr','on', ...
     'JacobPattern',Jpattern,'Hessian','user-supplied','HessPattern',Hpattern, ...
     'HessFcn',@hessfun,'MaxIter',1000, ...
     'TolX', 1e-15, 'TolFun', 1e-8, 'TolCon', 1e-8); 
     
A = []; b = [];
Aeq = [-1.5  2   1 -0.5 1  0  0  0;
        3   -1   0  0   0 -1  0  0;
       -1    0.5 0  0   0  0 -1  0;
       -1   -1   0  0   0  0  0 -1];
beq = [2 3 -4 -7];
lb = zeros(8,1);
ub = Inf*ones(8,1);
x0 = zeros(8,1);

extendedFeatures.ccIndexList1 = [6 7 8];
extendedFeatures.ccIndexList2 = [3 4 5];

[x,fval,exitflag,output,lambda] = ...
    knitromatlab(@objfun,x0,A,b,Aeq,beq,lb,ub,@constfun,extendedFeatures,options)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,g] = objfun(x)

% Uncomment the command below to ensure that Knitro output is flushed to 
% the screen regularly (depending on the Display/outlev setting).  This 
% may be desirable on large, difficult problems that take a while to solve.  
%disp '';
    
f = (x(1)-5)^2 + (2*x(2)+1)^2;

if nargout > 1
 g = zeros(8,1);
 g(1) = 2*(x(1)-5);
 g(2) = 4*(2*x(2)+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq,Gc,Gceq]= constfun(x)

c = [];
ceq=[];
Gc = [];
Gceq=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H]= hessfun(x,lambda)

H=sparse(zeros(8));

H(1,1) = 2;
H(2,2) = 4;
