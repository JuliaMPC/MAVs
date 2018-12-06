# Copyright (C) 2016 Artelys.
#
# Solving MPEC using knitro 
#
#  min (x0 - 5)^2 + (2*x1 + 1)^2        
#  s.t.
#  2*(x1 - 1) - 1.5*x0 + x2 - 0.5*x3 + x4 = 0
#  3*x0 - x1 - 3 - x5 = 0
#  -x0 + 0.5*x1 + 4 - x6 = 0
#  -x0 - x1 + 7 - x7 = 0
#  0 <= x5 complements x2 >= 0
#  0 <= x6 complements x3 >= 0
#  0 <= x7 complements x4 >= 0
#  xi >= 0, i = 0 to 7
#

# Load KnitroR package
library('KnitroR')

# Objective callback
eval_f <- function(x){
	return ( (x[1]-5.0)^2+(2*x[2]+1.0)^2 )
}

# Gradient callback
eval_grad_f <- function(x){
	return ( c(2.0*x[1],4.0*x[2],rep(0,6)) )
}

# Constraints callback
eval_g <- function(x){
	return ( c(2.0*(x[2]-1.0)-1.5*x[1]+x[3]-0.5*x[4]+x[5],
			   3.0*x[1]-x[2]-3.0-x[6],
			   -x[1]+0.5*x[2]+4.0-x[7],
			   -x[1]-x[2]+7.0-x[8]) )
}

# Jacobian callback
eval_jac_g <- function(x){
	return ( c(2.0,-1.5,1.0,-0.5,1.0,3.0,-1.0,-1.0,-1.0,0.5,4.0,-1.0,-1.0,-1.0,-1.0) )
}

# Jacobian sparsity pattern
jIndC <- c(rep(0,5), rep(1,3), rep(2,3), rep(3,3))
jIndV <- c(0:4, 0, 1, 5, 0, 1, 6, 0, 1, 7)

# Bounds on variables
xL <- rep(0,8)
xU <- rep(1e20,8)

# Bounds on constraints
cL <- rep(0,4)
cU <- rep(0,4)

# Knitro options
knitro_opts <- list("ms_enable"=1)

# Launch Knitro
mpec_sol = knitro(objective = eval_f, 
				   gradient = eval_grad_f, 
                   constraints = eval_g,
				   jacobian = eval_jac_g,
				   jacBitMap= c(rep(1,5),rep(0,3),1,1,rep(0,3),1,0,0,1,1,rep(0,4),1,0,1,1,rep(0,5),1),
                   cL = cL, 
                   cU = cU,
				   xL = xL,
				   xU = xU,
				   numCompConstraints = 3,
				   ccIdxList1 = c(5,6,7),
				   ccIdxList2 = c(2,3,4),
		constraintsTypes = c(1, 1, 1, 1),
               options = knitro_opts)
