# Copyright (C) 2016 Artelys.
#
# Example problem, number 71 from the Hock & Schittkowsky collection
#
# min    x1*x4*(x1 + x2 + x3) + x3
# s.t.   x1*x2*x3*x4 >= 25
#        x1^2 + x2^2 + x3^2 + x4^2 = 40
#        1 <= x1,x2,x3,x4 <= 5
#
# Initial point: (1,5,5,1)
# Optimal solution: (1.00000000, 4.74299963, 3.82114998, 1.37940829)

# Load KnitroR package
library('KnitroR')

#
# f(x) = x1*x4*(x1 + x2 + x3) + x3
#
eval_f <- function( x ) {
    return( x[1]*x[4]*(x[1] + x[2] + x[3]) + x[3] )
}

eval_grad_f <- function( x ) {
    return( c( x[1] * x[4] + x[4] * (x[1] + x[2] + x[3]),
               x[1] * x[4],
               x[1] * x[4] + 1.0,
               x[1] * (x[1] + x[2] + x[3]) ) )
}

# constraint functions
eval_g <- function( x ) {
    return( c( x[1] * x[2] * x[3] * x[4],
               x[1]^2 + x[2]^2 + x[3]^2 + x[4]^2 ) )
}

# Dense jacobian sparsity pattern
jIndC <- c(0,0,0,0,1,1,1,1)
jIndV <- c(0,1,2,3,0,1,2,3)

# Jacobian callback
eval_jac_g <- function( x ) {
    return( c ( x[2]*x[3]*x[4],
                x[1]*x[3]*x[4],
                x[1]*x[2]*x[4],
                x[1]*x[2]*x[3],
                2.0*x[1],
                2.0*x[2],
                2.0*x[3],
                2.0*x[4] ) )
}

# Dense hessian of Lagrangian
# Sparsity pattern
hIndR <- c( 0, 0, 0, 0,
               1, 1, 1,
                  2, 2,
                 3 )
hIndC <- c( 0, 1, 2, 3,
               1, 2, 3,
          2, 3,
                     3 )
# Hessian of Lagrangian callback
eval_h <- function( x,  lambda, sig ) {

    hss <- numeric(10)
    hss[1] = sig * (2*x[4])

    hss[2] = sig * (x[4])
    hss[3] = 0

    hss[4] = sig * (x[4])
    hss[5] = 0
    hss[6] = 0

    hss[7] = sig * (2*x[1] + x[2] + x[3])
    hss[8] = sig * (x[1])
    hss[9] = sig * (x[1])
    hss[10] = 0

    # First constraint
    hss[2] = hss[2] + lambda[1] * (x[3] * x[4])

    hss[4] = hss[4] + lambda[1] * (x[2] * x[4])
    hss[5] = hss[5] + lambda[1] * (x[1] * x[4])

    hss[7] = hss[7] + lambda[1] * (x[2] * x[3])
    hss[8] = hss[8] + lambda[1] * (x[1] * x[3])
    hss[9] = hss[9] + lambda[1] * (x[1] * x[2])

    # Second constraint
    hss[1] = hss[1] + lambda[2] * 2
    hss[3] = hss[3] + lambda[2] * 2
    hss[6] = hss[6] + lambda[2] * 2
    hss[10] = hss[10] + lambda[2] * 2

    return ( hss )
}

# Initial guess
x0 <- c( 1, 5, 5, 1 )

# Lower and upper bounds
lb <- c( 1, 1, 1, 1 )
ub <- c( 5, 5, 5, 5 )

# Lower and upper bounds on constraints
constraint_lb <- c(  25, 40 )
constraint_ub <- c( Inf, 40)

sol <- knitro(x0=x0,
              objective=eval_f,
              gradient=eval_grad_f,
              constraints=eval_g,
              jacobian=eval_jac_g,
              hessianLag=eval_h,
              jacIndexCons=jIndC,
              jacIndexVars=jIndV,
              hessIndexRows=hIndR,
              hessIndexCols=hIndC,
              xL=lb,
              xU=ub,
              cL=constraint_lb,
              cU=constraint_ub)
