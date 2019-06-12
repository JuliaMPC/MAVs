macro FZ_RR()#(N, F_yf, F_yr, V, Ax, r)
	code = quote
		# define the lateral tire forces
		F_yf = @F_YF()
		F_yr = @F_YR()
		FZ_rr = zeros(length(Ax),1)
		for ii in eachindex(Ax)
			FZ_rr[ii] = 0.5*(FzR0 + KZX*(Ax[ii] - V[ii]*R[ii])) + KZYR*((F_yf[ii] + F_yr[ii])/m)
		end
		FZ_rr
	end
	return esc(code)
end
