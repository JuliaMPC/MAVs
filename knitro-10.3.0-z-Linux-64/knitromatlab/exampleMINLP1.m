function exampleMINLP1

%--------------------------------------------------------------------------
% Copyright (c) 2016 by Artelys
% All Rights Reserved.
%
%  This file contains routines to implement problemDef.h for
%  test problem 1 (Synthesis of processing system) in
%  M. Duran & I.E. Grossmann,  "An outer approximation algorithm for
%  a class of mixed integer nonlinear programs", Mathematical
%  Programming 36, pp. 307-339, 1986.  The problem also appears as
%  problem synthes1 in the MacMINLP test set.
%  
%
%  min   5 x4 + 6 x5 + 8 x6 + 10 x1 - 7 x3 -18 log(x2 + 1)
%       - 19.2 log(x1 - x2 + 1) + 10
%  s.t.  0.8 log(x2 + 1) + 0.96 log(x1 - x2 + 1) - 0.8 x3 >= 0
%        log(x2 + 1) + 1.2 log(x1 - x2 + 1) - x3 - 2 x6 >= -2
%        x2 - x1 <= 0
%        x2 - 2 x4 <= 0
%        x1 - x2 - 2 x5 <= 0
%        x4 + x5 <= 1
%        0 <= x1 <= 2 
%        0 <= x2 <= 2
%        0 <= x3 <= 1
%        x1, x2, x3 continuous
%        x4, x5, x6 binary
%        
%
%  The solution is (1.30098, 0, 1, 0, 1, 0).
%--------------------------------------------------------------------------
 
 Jac = [1 1 1 0 0 0;
        1 1 1 0 0 1];
 Jpattern = sparse(Jac);
 
 Hess = [1 1 0 0 0 0;
         1 1 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0];
 Hpattern = sparse(Hess);
 
 options = optimset('Algorithm', 'active-set', 'Display','iter', ...
    'GradObj','on','GradConstr','on', ...
    'JacobPattern',Jpattern,'Hessian','user-supplied','HessPattern',Hpattern, ...
    'HessFcn',@hessfun,'MaxIter',1000, ...
    'TolX', 1e-15, 'TolFun', 1e-8, 'TolCon', 1e-8,'DerivativeCheck','On');
    
 A = [-1  1  0  0  0  0;
       0  1  0 -2  0  0;
       1 -1  0  0 -2  0;
       0  0  0  1  1  0];
 b = [0; 0; 0; 1];
 Aeq = []; beq = [];
 lb = [0; 0; 0; 0; 0; 0];
 ub = [2; 2; 1; 1; 1; 1];
 xType = [0; 0; 0; 1; 1; 1];
 objFnType = 0;
 cFnType = [1; 1];
 x0 = [2;0;1;0;1;0];
 
% Call Knitro to solve the optimization model.
[x,fval,exitflag,output,lambda] = ...
   knitromatlab_mip(@objfun,x0,A,b,Aeq,beq,lb,ub,@constfun,xType,objFnType,cFnType,[],options,'mipoptions.opt');
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,g] = objfun(x)

% Uncomment the command below to ensure that Knitro output is flushed to 
% the screen regularly (depending on the Display/outlev setting).  This 
% may be desirable on large, difficult problems that take a while to solve.  
%disp '';
     
dTmp1 = x(1) - x(2) + 1;
dTmp2 = x(2) + 1;

f = 5*x(4) + 6*x(5) + 8*x(6) + 10*x(1) - 7*x(3) - 18*log(dTmp2) - 19.2*log(dTmp1) + 10;

% its derivative wrt. x
if nargout > 1
   g(1)=10-19.2/dTmp1;
   g(2)=-18/dTmp2 + 19.2/dTmp1;
   g(3)=-7;
   g(4)=5;
   g(5)=6;
   g(6)=8;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq,Gc,Gceq]= constfun(x)

dTmp1 = x(1) - x(2) + 1;
dTmp2 = x(2) + 1;

% Constraint function
ceq=[];
c(1)= -(0.8*log(dTmp2) + 0.96*log(dTmp1) - 0.8*x(3));
c(2)= -(log(dTmp2) + 1.2*log(dTmp1) - x(3) - 2*x(6) + 2);

% Gradients of the constraint functions wrt. x
if nargout > 2
   Gc(1,1)=-(0.96/dTmp1);
   Gc(2,1)=-(0.8/(dTmp2) - 0.96/(dTmp1));
   Gc(3,1)=-(-0.8);
   Gc(1,2)=-(1.2/dTmp1);
   Gc(2,2)=-(1/dTmp2 - 1.2/dTmp1);
   Gc(3,2)=-(-1);
   Gc(6,2)=-(-2);
   Gceq=[];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H]= hessfun(x,lambda)
% Hessian function.
% Only need to specify structural non-zero elements of the upper
% triangle (including diagonal)  

dTmp1 = x(1) - x(2) + 1;
dTmp2 = x(2) + 1;
dTmp3 = dTmp1*dTmp1;
dTmp4 = dTmp2*dTmp2;
  
H(1,1) = 19.2/dTmp3 - lambda.ineqnonlin(1)*(-0.96/dTmp3) - lambda.ineqnonlin(2)*(-1.2/dTmp3);
H(1,2) = -19.2/dTmp3 - lambda.ineqnonlin(1)*(0.96/dTmp3) - lambda.ineqnonlin(2)*(1.2/dTmp3);
H(2,2) = 19.2/dTmp3 + 18/dTmp4 - lambda.ineqnonlin(1)*(-0.96/dTmp3 - 0.8/dTmp4) - lambda.ineqnonlin(1)*(-1.2/dTmp3 - 1.0/dTmp4);
H = sparse(H);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
