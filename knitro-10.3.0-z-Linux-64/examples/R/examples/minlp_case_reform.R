# Copyright (C) 2016 Artelys.
#
# Solving MIP using knitromip with
#
#  min   5*x4 + 6*x5 + 8*x6 + 10*x1 - 7*x3 -18*log(x2 + 1)
#       - 19.2*log(x1 - x2 + 1) + 10
#  s.t.  0.8*log(x2 + 1) + 0.96*log(x1 - x2 + 1) - 0.8*x3 >= 0
#        log(x2 + 1) + 1.2*log(x1 - x2 + 1) - x3 - 2*x6 >= -2
#        x2 - x1 <= 0
#        x2 - 2*x4 <= 0
#        x1 - x2 - 2*x5 <= 0
#         x4 + x5 <= 1
#        0 <= x1 <= 2
#        0 <= x2 <= 2
#        0 <= x3 <= 1
#        x1, x2, x3 continuous
#        x4, x5, x6 binary
#
#  The solution is (1.30098, 0, 1, 0, 1, 0).
#

# Load KnitroR package
library('KnitroR')

# Objective callback
eval_f <- function(x){
    return ( 5*x[4]+6*x[5]+8*x[6]+10*x[1]-7*x[3]-18*log(x[2]+1)-19.2*log(x[1]-x[2]+1)+10)
}

# Objective gradient callback
eval_grad_f <- function(x){
    return ( c(10.0-(19.2/(x[1]-x[2]+1.0)),
               (-18.0/(x[2]+1.0))+(19.2/(x[1]-x[2]+1.0)),
               -7.0,
               5.0,
               6.0,
               8.0) )
}

# Constraints callback
eval_g <- function(x){
    return ( c(0.8*log(x[2]+1.0)+0.96*log(x[1]-x[2]+1.0)-0.8*x[3],
               log(x[2]+1.0)+1.2*log(x[1]-x[2]+1.0)-x[3]-2.0*x[6],
               x[2]-x[1],
               x[2]-2.0*x[4],
               x[1]-x[2]-2.0*x[5],
               x[4]+x[5]) )

}

# Constraints jacobian callback
eval_jac_g <- function(x){
    tmp1 <- x[1]-x[2]+1.0
    tmp2 <- x[2]+1.0
    return ( c(0.96/tmp1, (-0.96/tmp1)+(0.8/tmp2), -0.8, 
			   1.2/tmp1, (-1.2/tmp1)+(1.0/tmp2),-1.0, -2.0,
			   -1.0, 1.0,
			   1.0, -2.0, 
			   1.0, -1.0, -2.0,
			   1.0, 1.0) )		   
}

# Sparsity pattern of jacobian
jIndC <- c(rep(0,3), rep(1,4), rep(2,2), rep(3,2), rep(4,3), rep(5,2))
jIndV <- c(0,1,2,0,1,2,5,0,1,1,3,0,1,4,3,4)

# Bounds on variables
xL <- c(rep(0,3), rep(0, 3))
xU <- c(2, 2, 1, rep(2, 3))

# Bounds on constraints
cL <- c(0, -2, rep(-1e20, 4))
cU <- c(rep(1e20, 2), rep(0,3), 1)

# Knitro's MINLP parameters
xType <- c(rep(0,3), rep(1,3))
xPriorities <- c(rep(0,6))
cFnType <- c(rep(2,2), rep(1,4))
objfntype <- 2

# Set knitro options
knitro_opts <- list("derivcheck" = 1,
            "hessopt" = 2)

# Strategy on integer variables
xIndex <- c(4, 5)
xStrategy <- c(1, 1)

# Call Knitro MINLP solver
mip_sol <- knitromip(objective = eval_f,
                     gradient = eval_grad_f,
                     constraints = eval_g,
                     jacobian = eval_jac_g,
                     jacIndexCons = jIndC,
                     jacIndexVars = jIndV,
                     xL = xL,
                     xU = xU,
                     cL = cL,
                     cU = cU,
                     xType = xType,
                     cFnType = cFnType,
                     objfntype = objfntype,
                     xIndex = xIndex,
                     xStrategy = xStrategy,
                     options = knitro_opts)
