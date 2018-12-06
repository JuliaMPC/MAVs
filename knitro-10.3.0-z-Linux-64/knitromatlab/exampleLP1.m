function exampleLP1
%--------------------------------------------------------------------------
% Copyright (c) 2016 by Artelys
% All Rights Reserved.
%
% Example problem formulated as an Matlab model used to
% demonstate using the Knitro Mex to solve a
% linear program (LP).    
%
%  We solve an LP problem of the form
%
%     minimize      c'x
%     subject to   A_eq x = beq
%                     A x  <= b
%                 lb <= x  <= ub
%--------------------------------------------------------------------------

% Define the LP you want to solve.
% (This example from "Numerical Optimization", J. Nocedal and S. Wright)
%
%     minimize     -4*x1 - 2*x2
%     subject to   x1 + x2 + x3        = 5
%                  2*x1 + 0.5*x2 + x4  = 8
%                 0 <= (x1, x2, x3, x4) 
% obj=17.333 x=3.667,1.333,0,0

c = [-4; -2; 0; 0];
lb = [0; 0; 0; 0];
ub = [inf; inf; inf; inf];
Aeq  = [ 1,   1, 1, 0;
         2, 0.5, 0, 1];
beq = [5;
       8];
A = [];
b = [];

% Define the initial point.
% Typically, for LPs, it is best just to define this as all 0's and then
% set the Knitro user option "bar_initpt yes" (using a Knitro options file)
% so that Knitro applies its own initial point strategy.
x0 = [0; 0; 0; 0];

% Call knitromatlab wrapper function to solve the LP.
[x, lambda, exitflag] = knitromatlablp (x0, c, A, b, Aeq, beq, lb, ub );

x
lambda.eqlin
lambda.lower
lambda.upper

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x, lambda, exitflag] =  ... 
    knitromatlablp (x0, c, A, b, Aeq, beq, lb, ub)

% This LP wrapper function takes a LP specified in a standard way and 
% transforms it in a way so that it can be solved using "knitromatlab".

% Define Jacobian sparsity pattern.
Jpattern = sparse(Aeq);

% Pass additional parameters via anonymous functions to evaluate objective.
objfun = @(x) knitromatlabLPobjEval(x,c);               
  
% The Hessian is not used for an LP.  We will define it anyway and just
% make it an empty sparse matrix of proper dimension.
n = length(c);
Hpattern = sparse(n,n);
hessfun= @(x,lambda) knitromatlabLPhessEval(n);

% Set some Matlab user options.
options = optimset( 'HessFcn', hessfun, 'Hessian', 'user-supplied', ...
                    'JacobPattern', Jpattern, 'HessPattern', Hpattern);                

% Call knitromatlab to solve the problem.  Some Knitro specific options are 
% specified in the 'qpoptions.opt' file that is passed in.                 
[x, fval, exitflag, output, lambda] = ...
    knitromatlab(objfun, x0, A, b, Aeq, beq, lb, ub, [], [], options, 'lpoptions.opt');

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,grad] = knitromatlabLPobjEval(x,c)

% Uncomment the command below to ensure that Knitro output is flushed to 
% the screen regularly (depending on the Display/outlev setting).  This 
% may be desirable on large, difficult problems that take a while to solve.  
%disp '';
    
% Compute QP objective function and gradient.
f = c'*x;
grad = c;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Hess = knitromatlabLPhessEval(n)

% There is no Hessian for an LP.  
% Just set as empty sparse matrix of correct dimension.
Hess = sparse(n,n);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
