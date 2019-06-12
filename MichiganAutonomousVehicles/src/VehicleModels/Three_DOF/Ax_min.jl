macro Ax_min()
	code = quote
		# minimum longitudinal acceleration for given speed
		Ax_min = zeros(length(U),1)
		for i in eachindex(U)
			Ax_min[i,1] = AXC[5]*U[i]^3 + AXC[6]*U[i]^2 + AXC[7]*U[i] + AXC[8]
		end
		Ax_min
	end
	return esc(code)
end
