macro FZ_RL()#(N, F_yf, F_yr, V, Ax, R)
	code=quote
		# define the lateral tire forces
		F_yf = @F_YF()
		F_yr = @F_YR()
		FZ_rl = zeros(length(Ax),1)
		for ii in eachindex(Ax)
			FZ_rl[ii] = 0.5*(FzR0 + KZX*(Ax[ii] - V[ii]*R[ii])) - KZYR*((F_yf[ii] + F_yr[ii])/m)
		end
		FZ_rl
	end
	return esc(code)
end
