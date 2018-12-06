function [x,fval,exitflag,output,lambda] = knitrolink(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options,KnitroOptions,varargin)
%--------------------------------------------------------------------------
% Copyright (c) 2016 by Artelys
% All Rights Reserved.
%
% knitrolink uses the Artelys Knitro (R) solver to solve problems of the form:
%    min fun(x)  subject to:  A*x  <= b, Aeq*x  = beq (linear constraints)
%     x                     C(x) <= 0, Ceq(x) = 0   (nonlinear constraints)
%                              lb <= x <= ub        (bounds)
%
%   x = knitrolink(FUN,X0,A,b) starts at X0 and finds a minimum x to the
%   function FUN, subject to the linear inequalities A*x <= B. FUN accepts
%   input x and returns a scalar function value F evaluated at x. X0 may be
%   a scalar, vector, or matrix. 
%
%   x = knitrolink(FUN,X0,A,b,Aeq,beq) minimizes FUN subject to the 
%   linear equalities Aeq*x = beq as well as A*x <= b. (Set A = [] and 
%   B = [] if no inequalities exist.)
%
%   x = knitrolink(FUN,X0,A,B,Aeq,Beq,lb,ub) defines a set of lower and
%   upper bounds on the design variables, x, so that a solution is found in 
%   the range LB <= X <= UB. Use empty matrices for LB and UB if no bounds
%   exist. Set LB(i) = -Inf if X(i) is unbounded below; set UB(i) = Inf if
%   X(i) is unbounded above.
%
%   x = knitrolink(FUN,X0,A,b,Aeq,beq,lb,ub,NONLCON) subjects the
%   minimization to the constraints defined in NONLCON. The function
%   NONLCON accepts x and returns the vectors c and ceq, representing the
%   nonlinear inequalities and equalities respectively. knitrolink
%   minimizes FUN such that c(X) <= 0 and ceq(X) = 0. (Set lb = [] and/or 
%   ub = [] if no bounds exist.)
%
%   x = knitrolink(FUN,X0,A,b,Aeq,beq,lb,ub,NONLCON,OPTIONS) minimizes
%   with the default optimization parameters replaced by values in the
%   structure OPTIONS, an argument created with the OPTIMSET function. See
%   OPTIMSET for details. For a list of options accepted by FMINCON refer
%   to the documentation. Use OPTIONS = [] as a place holder if no options
%   are set.
%  
%   x = knitrolink(FUN,X0,A,b,Aeq,beq,lb,ub,NONLCON,OPTIONS,
%   knitroOptionsFile) minimizes with the default optimization 
%   parameters replaced by the values in the Knitro (R) options text file 
%   whose name is given in the string knitroOptionsFile.
%
%   [x,FVAL] = knitrolink(FUN,X0,...) returns the value of the objective 
%   function FUN at the solution x.
%
%   [x,FVAL,EXITFLAG] = KTRLINK(FUN,X0,...) returns an EXITFLAG that
%   describes the exit condition of knitrolink. For possible values of
%   EXITFLAG and the corresponding exit conditions, refer to Knitro (R) 
%   documentation.
%
%   [x,FVAL,EXITFLAG,OUTPUT] = knitrolink(FUN,X0,...) returns a 
%   structure OUTPUT with the number of iterations taken in 
%   OUTPUT.iterations, the number of function evaluations in 
%   OUTPUT.funcCount, the algorithm used in OUTPUT.algorithm, the 
%   first-order optimality in OUTPUT.firstorderopt, and the exit message in
%   OUTPUT.message.
%
%   [x,FVAL,EXITFLAG,OUTPUT,LAMBDA] = knitrolink(FUN,X0,...) returns the
%   Lagrange multipliers at the solution x: LAMBDA.lower for lb,
%   LAMBDA.upper for ub, LAMBDA.ineqlin is for the linear inequalities,
%   LAMBDA.eqlin is for the linear equalities, LAMBDA.ineqnonlin is for the
%   nonlinear inequalities, and LAMBDA.eqnonlin is for the nonlinear
%   equalities.
%--------------------------------------------------------------------------

if nargin < 11 
    KnitroOptions = '';
    if nargin < 10 
        options = [];
        if nargin < 9 
            nonlcon = [];
            if nargin < 8 
                ub = [];
                if nargin < 7 
                    lb = [];
                    if nargin < 6 
                        beq = [];
                        if nargin < 5 
                            Aeq =[];
                            if nargin < 4
                                b = [];
                                if nargin < 3
                                    A = [];
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

[x,fval,exitflag,output,lambda] = knitromatlab_mip(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,[],[],[],[],options,KnitroOptions,varargin{:});
