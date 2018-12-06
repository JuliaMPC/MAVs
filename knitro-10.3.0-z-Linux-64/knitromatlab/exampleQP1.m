function exampleQP1
%--------------------------------------------------------------------------
% Copyright (c) 2016 by Artelys
% All Rights Reserved.
%
% Example problem formulated as an Matlab model used to
% demonstate using the Knitro Mex to solve a
% quadratic program (QP).    
%
%  We solve a QP problem
%
%     minimize      1/2 x' H x  + g'x
%     subject to   A_eq x = beq
%                     A x  <= b
%                 lb <= x  <= ub
%--------------------------------------------------------------------------

% Define the QP you want to solve.
%
%     minimize     0.5*(x1^2+x2^2+x3^2) + 11*x1 + x3
%     subject to   -6*x3  <= 5
%                 0 <= x1 
%                 0 <= x2
%                -3 <= x3 <= 2
% obj=-0.4861   x=0,0,-5/6
g = [11; 0; 1];
lb = [0; 0; -3];
ub = [inf; inf; 2];
H  = eye(3);
A = [ 0, 0, -6];
b = [5];
Aeq = [];
beq = [];

% Define the initial point
x0 = [0;0;0];

% Call knitromatlab wrapper function to solve the QP.
[x, lambda, exitflag] = knitromatlabqp (x0, g, H, A, b, Aeq, beq, lb, ub );


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x, lambda, exitflag] =  ... 
    knitromatlabqp (x0, g, H, A, b, Aeq, beq, lb, ub)

% This QP wrapper function takes a QP specified in a standard way and 
% transforms it in a way so that it can be solved using "knitromatlab".

% Define Jacobian and Hessian sparsity patter.
Jpattern = sparse(A);
Hpattern = sparse(H);

% Pass additional parameters via anonymous functions.
objfun = @(x) knitromatlabQPobjEval(x,H,g);
hessfun= @(x,lambda) knitromatlabQPhessEval(x,lambda,H);                       

% Set some Matlab user options.
options = optimset( 'HessFcn', hessfun, 'Hessian', 'user-supplied', ...
                    'JacobPattern', Jpattern, 'HessPattern', Hpattern);               

% Call knitromatlab to solve the problem.  Some Knitro specific options are 
% specified in the 'qpoptions.opt' file that is passed in.                 
[x, fval, exitflag, output, lambda] = ...
    knitromatlab(objfun, x0, A, b, Aeq, beq, lb, ub, [], [], options, 'qpoptions.opt');

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,grad] = knitromatlabQPobjEval(x,H,g)

% Uncomment the command below to ensure that Knitro output is flushed to 
% the screen regularly (depending on the Display/outlev setting).  This 
% may be desirable on large, difficult problems that take a while to solve.  
%disp '';
    
% Compute QP objective function and gradient.
f = 0.5*x'*H*x + g'*x;
grad = H*x + g;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Hess = knitromatlabQPhessEval(x,lambda,H)

% Compute QP Hessian.
Hess = sparse(H);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
