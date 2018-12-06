# Import KnitroR package
library('KnitroR')

# Residuals function to be used in knitrovlsq, which returns a vector of residuals
# computed as the difference between the nonlinear fitting model and the target value.
nlres <- function(x, dfX, dfY) {
       dMatX <- data.matrix(dfX)
       dVecX <- as.vector(t(dMatX))

       dMatY <- data.matrix(dfY)
       dVecY <- as.vector(t(dMatY))
		
       return( c( x[1] * dVecX[1]^(x[2]) - dVecY[1],
				  x[1] * dVecX[2]^(x[2]) - dVecY[2],
				  x[1] * dVecX[3]^(x[2]) - dVecY[3],
				  x[1] * dVecX[4]^(x[2]) - dVecY[4],
				  x[1] * dVecX[5]^(x[2]) - dVecY[5],
				  x[1] * dVecX[6]^(x[2]) - dVecY[6] ) )
}

# Residuals function to be used in nls, which returns a vector of residuals
# computed as the difference between the nonlinear fitting model
# f(x,p) = p1*x^p2 and the target value y.
nlsres <- function(x1, x2, dfX, dfY) {

    dMatX <- data.matrix(dfX)
    dVecX <- as.vector(t(dMatX))

    dMatY <- data.matrix(dfY)
    dVecY <- as.vector(t(dMatY))

    return( c( x1 * dVecX[1]^(x2) - dVecY[1],
               x1 * dVecX[2]^(x2) - dVecY[2],
               x1 * dVecX[3]^(x2) - dVecY[3],
               x1 * dVecX[4]^(x2) - dVecY[4],
               x1 * dVecX[5]^(x2) - dVecY[5],
               x1 * dVecX[6]^(x2) - dVecY[6] ) )
}

# Jacobian of the residuals function R(p) = F(X, p) -Y
nljac <- function(x, dfX, dfY) {

    dMatX <- data.matrix(dfX)
    dVecX <- as.vector(t(dMatX))

    dMatY <- data.matrix(dfY)
    dVecY <- as.vector(t(dMatY))
	
	
    return( c( dVecX[1]^(x[2]), x[1] * log(dVecX[1]) * dVecX[1]^(x[2]),
               dVecX[2]^(x[2]), x[1] * log(dVecX[2]) * dVecX[2]^(x[2]),
               dVecX[3]^(x[2]), x[1] * log(dVecX[3]) * dVecX[3]^(x[2]),
               dVecX[4]^(x[2]), x[1] * log(dVecX[4]) * dVecX[4]^(x[2]),
               dVecX[5]^(x[2]), x[1] * log(dVecX[5]) * dVecX[5]^(x[2]),
               dVecX[6]^(x[2]), x[1] * log(dVecX[6]) * dVecX[6]^(x[2]) ) )
}

# x samples as a data frame
x <- c(1.309, 1.471, 1.49, 1.565, 1.611, 1.68)
dfx <- data.frame(x)
# y samples, or target values as a data frame
y <- c(2.138, 3.421, 3.597, 4.34, 4.882, 5.66)
dfy <- data.frame(y)

# Call knitrovlsq from the initial guess p1=1, p2=1.
# Record computation time.
ktrSolLsq <- knitrolsq(dimp = 2,
						par0 = c(1, 1),
						dataFrameX = dfx,
						dataFrameY = dfy,
						residual = nlres,
						jacobian = nljac,
						options = list("outlev" = 6, "outmode"=2))

# Call the R build-in function nls on the same nonlinear fitting problem.
# Record computation time.
df <- data.frame(x, y)
system.time(m <- nls( ~ nlsres(p1, p2, x, y), data = df, start = list(p1 = 1, p2 = 1), trace = T))
print(m)
