function exampleNLP1
%--------------------------------------------------------------------------
% Copyright (c) 2016 by Artelys
% All Rights Reserved.
%
% Example problem formulated as an Matlab model used to
% demonstate using the Knitro Mex interface with Matlab.
% The problem is
%
%     minimize    1000 - x1^2 - 2*x2^2 - x3^2 - x1*x2 - x1*x3
%     subject to  8*x1 + 14*x2 + 7*x3         = 56
%                 -(x1^2 + x2^2 + x3^2 - 25) <= 0
%                 0 <= (x1, x2, x3) <= 10
%
% and has two local solutions:
%   the point (0,0,8) with objective 936.0, and
%   the point (7,0,0) with objective 951.0
%--------------------------------------------------------------------------

% Define the Jacobian sparsity pattern.
Jac = [1, 1, 1];
Jpattern = sparse(Jac);

% Define the Hessian sparsity pattern.
% You can supply the Hessian matrix via optimset by setting the
% options below and writing a routine such as "hessfun" below.
Hess = [1, 1, 1; 1, 1, 0; 1, 0, 1];
Hpattern = sparse(Hess);

% Specify the user options.  
% Choose the Interior Point 'Algorithm' by setting 'interior-point' below.
% Choose the Active Set 'Algorithm' by setting 'active-set' below.
% It is highly recommended to try both these 'Algorithms'.
options = optimset('Algorithm', 'interior-point', 'Display','iter', ...
    'GradObj','on','GradConstr','on', ...
    'JacobPattern',Jpattern,'Hessian','user-supplied','HessPattern',Hpattern, ...
    'HessFcn',@hessfun,'MaxIter',1000, ...
    'TolX', 1e-15, 'TolFun', 1e-8, 'TolCon', 1e-8);


% Specify an initial point.
x0 = [2;2;2];

% Specify some model characteristics.
A = []; b = [];
Aeq = [8 14 7];
beq = 56;
lb = [0;0;0];
ub = [10;10;10];

% Call Knitro to solve the optimization model.
% Specify some extra Knitro-specific options in "nlpoptions.opt".  Any Knitro
% options can be specified by passing in a Knitro options file. 
[x,fval,exitflag,output,lambda,grad,hess] = ...
   knitromatlab(@objfun,x0,A,b,Aeq,beq,lb,ub,@constfun,[],options,'nlpoptions.opt');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,g] = objfun(x)

% Uncomment the command below to ensure that Knitro output is flushed to 
% the screen regularly (depending on the Display/outlev setting).  This 
% may be desirable on large, difficult problems that take a while to solve.  
%disp '';
    
% Objective function
f = 1000 - x(1)^2 - 2*x(2)^2 - x(3)^2 - x(1)*x(2) - x(1)*x(3);

% its derivative wrt. x
if nargout > 1
   g(1)=-2*x(1)-x(2)-x(3);
   g(2)=-4*x(2)-x(1);
   g(3)=-2*x(3)-x(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq,Gc,Gceq]= constfun(x)
% Constraint function
ceq=[];
c(1)= -(x(1)^2 + x(2)^2 + x(3)^2 - 25);

% Gradients of the constraint functions wrt. x
if nargout > 2
   Gc(1,1)=-2*x(1);
   Gc(2,1)=-2*x(2);
   Gc(3,1)=-2*x(3);
   Gceq=[];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H]= hessfun(x,lambda)
% Hessian function.
% Only need to specify structural non-zero elements of the upper
% triangle (including diagonal)  
  
H(1,1) = -2 + lambda.ineqnonlin*(-2);
H(1,2) = -1;
%H(2,1) = H(1,2);
H(1,3) = -1;
%H(3,1) = H(1,3);
H(2,2) = -4 + lambda.ineqnonlin*(-2);
H(3,3) = -2 + lambda.ineqnonlin*(-2);
H = sparse(H);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
