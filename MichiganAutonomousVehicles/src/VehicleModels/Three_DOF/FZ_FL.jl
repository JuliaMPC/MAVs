macro FZ_FL()#(N, F_yf, F_yr, V, Ax, R)
	code=quote
		# define the lateral tire forces
		F_yf = @F_YF()
		F_yr = @F_YR()
		FZ_fl = zeros(length(Ax),1)
		for ii in eachindex(Ax)
			FZ_fl[ii] = 0.5*(FzF0 - KZX*(Ax[ii] - V[ii]*R[ii])) - KZYF*((F_yf[ii] + F_yr[ii])/m)
		end
		FZ_fl
	end
	return esc(code)
end
