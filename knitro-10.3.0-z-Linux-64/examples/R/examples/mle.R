# Copyright (C) 2016 Artelys.
#
# An example of maximum likelihood estimation with R and Knitro routines using different approaches
#

# Set seed for reproducibility purposes
set.seed(0)

# Parameter of the Gamma distribution
k = 2.21        # shape
theta = 12.07   # scale

# (optional) Initial point, which will be used for optimization
k0 = 1.0
theta0 = 1.0

# Generate a sample
n = 1000        # Size of the sample, make it grow to challenge efficiency of the different approaches
y = rgamma(n, k, scale=theta)

# Epsilon value to avoid numerical issues in 0
eps = 1e-15

# Add a truncated white noise
noise = rnorm(n, mean = 0, sd = 0.5)
y = y + noise # comment to remove noise
y[y<0] = eps    # shift on 0 since the support of gamma law is positive 


# Uncomment to plot distribution
# hist(y, prob=T, nclass=12)
# curve(dgamma(x, k, scale=theta), add=TRUE)


# ---------- MLE using R routines ---------- #
library(stats4)

# Log likelyhood function
cnt = 0 # Counter for function evaluations (trace does not include finite differences)
gammall <- function(k = 1.0, theta = 1.0) {
  cnt <<- cnt + 1
  -sum(dgamma(y, k, scale=theta, log = TRUE))
}


# Maximum likelihood estimation using stats4 package from R
cnt = 0
time = system.time(
  Rmle <- mle(gammall, start=list(k=k0, theta=theta0), method="L-BFGS-B", lower = c(eps, eps),
             control=list("trace"=2))
)
statsEval = cnt
print( sprintf("%-25s=     %d", "Function evaluations", statsEval) )
print( sprintf("%-25s=     %.02f", "Total time", time[1]) )
print( sprintf("stats4 mle finished", cnt, time[1]) )


# ---------- MLE using Knitro ---------- #
library(KnitroR)

# Define objective
evalf <- function(x) {
  return (gammall(k=x[1], theta=x[2]))
}

# Straightforward optimization using forward finite differences and BFGS
ktrMLE <- knitro(objective = evalf, x0 = c(k0, theta0), xL = c(eps,eps))

## Explicity setting default parameters and removing output
knitro_opts <- list("gradopt"=2, "hessopt"=2, "outlev"=0)

ktrMLE <- knitro(objective = evalf,
                x0 = c(k0, theta0), options = knitro_opts,
                xL = c(eps,eps))


##### Using exact gradient and hessian #####

# Gradient of the gamma distribution
gamma_diff <- function(x, k, theta) {
  ret = c(
    t(-1.0*log(theta) + 1.0*log(x) - 1.0*psigamma(k)), # diff in k
    t(1.0*(-k*theta + x)/theta**2) # diff in theta
  )
  dim(ret) <- c(length(x),2) # to matrix
  return(ret)
}

# Computing gradient on the whole sample
evalg <- function(x) {
  res = gamma_diff(y, x[1], x[2])
  return ( -colSums(res) )
}

# Hessian
gamma_hess <- function(x, k, theta) {
  len = length(x)
  ret = c(
    - len*psigamma(k,1), #d2f/dk2
    - len/theta, # d2f/dkdt
    sum((1.0*k*theta - 2.0*x)/theta**3) # d2f/dt2
  )
  #dim(ret) <- c(length(x),3) # to matrix
  return(ret)
}

evalh <- function( x, hessian_lambda, obj_factor ) {
  return (-obj_factor*gamma_hess(y, x[1], x[2]))
}

## Use exact gradient and hessian and activate derivatives check
# Derivative check should always be used at least once when exact derivatives are set
# this prevent many mistakes
knitro_opts <- list("gradopt"=1, "hessopt"=1, "derivcheck"=3, "outmode"=1)

ktrMLEexact <- knitro(objective = evalf, gradient=evalg,
                     hessian=evalh, hessIndexRows = c(0, 0, 1), hessIndexCols = c(0,1,1),
                     x0 = c(k0, theta0), xL = c(eps,eps), options = knitro_opts)


# Once you have validated gradient and hessian, derivatives check is not required
# for subsequent runs.
# Also, when exact gradients / hessian are provided, they are automatically used,
# unless gradopt or hessopt specifies otherwise
ktrMLEexact <- knitro(objective = evalf, gradient=evalg,
                     hessian=evalh, hessIndexRows = c(0, 0, 1), hessIndexCols = c(0,1,1),
                     x0 = c(k0, theta0), xL = c(eps,eps))


# ---------- Comparing results ---------- #

# Norm 2 of the difference between Knitro and mle
sqrt(sum(ktrMLEexact$x - Rmle@coef)^2)

# difference between objectives
ktrMLEexact$objective - Rmle@min

# Comparing number of function evaluations
ktrMLEexact$objEval - statsEval
