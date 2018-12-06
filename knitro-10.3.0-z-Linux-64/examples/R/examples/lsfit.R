# Copyright (C) 2016 Artelys.
#
# Example showing how to use the knitrolsq function
# to fit a nonlinear model with a 1-dimensional parameter
#

# Load KnitroR package
library('KnitroR')

# Set seed for reproducibility purposes
# Feel free to modify or delete the seed.
set.seed(0)

## Generate random data
# Number of points
len <- 1000
# Create random points
x = runif(len)
dfx <- data.frame(x)

# Evaluate nonlinear function at created points
y = x^3 + runif(len, min = -0.5, max = 0.5)
dfy <- data.frame(y)

# Plot
plot(x, y)
s <- seq(from = 0, to = 1, length = 50)
lines(s, s^3, col = "black", lty = 2)

## Residuals
nlform <- function(pow, sX, sY){
    return (sY - sX^pow)
}

## Jacobian
nljac <- function(pow, sX, sY){
    return (-log(sX)*sX^pow)
}

## Fit using knitrolsq with finite-difference computation of derivatives
ktrfitFD <- knitrolsq(dimp = 1,
                      par0 = 1.0,
                      dataFrameX = dfx,
                      dataFrameY = dfy,
                      residual = nlform)

## Fit using nls function
df <- data.frame(x, y)
m <- nls(y ~ I(x^power), data = df, start = list(power = 1), trace = T)

# Plot fit obtained by nls in green
lines(s, predict(m, list(x = s)), col = "green")
# Plot fir obtained by knitrolsq (finite-diff) in red
lines(s, lapply(s, function(x) x^(ktrfitFD$param)), col = "red", lty = 2)

## Fit using knitrolsq with user-defined jacobian of formula
ktrfitUD <- knitrolsq(dimp = 1,
                      par0 = 1.0,
                      dataFrameX = dfx,
                      dataFrameY = dfy,
                      residual = nlform,
                      jacobian = nljac,
					  jacIndexRows = c(0:(len-1)),
					  jacIndexCols = rep(0, len))

# Plot fit obtained by knitrolsq (user-defined) in blue
lines(s, lapply(s, function(x) x^(ktrfitUD$param)), col = "blue", lty = 2)

# Display results
print( paste('True parameter=', toString(3), sep=' ') )
print( '===============================================' )
print( 'Estimation obtained by nls: ' )
print( summary(m) )
print( '===============================================' )
print( paste('Estimated parameter using knitrolsq with finite-difference gradient=', toString(ktrfitFD$param), sep=' ') )
print( '===============================================' )
print( paste('Estimated parameter using knitrolsq with used-defined gradient=', toString(ktrfitUD$param), sep=' ') )