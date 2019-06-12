@with_kw immutable Vpara @deftype Float64
    # define model parameters

    # KinematicBicycle.jl only
    ax_min =  -2.
    ax_max = 2.
    # KinematicBicycle.jl only

    m          = 2.6887e+03
    Izz        = 4.1101e+03
    la         = 1.5775        # distance from CoG to front axle
    lb         = 1.7245        # distance from CoG to rear axle
    FzF0       = 1.3680e+04    # static front axle load
    FzR0       = 1.2696e+04    # static rear axle load

    # modified to match code: OCOA151019
    #KZX        = 805.    # KZX = dFZX_Coeff = dF: longitudinal load transfer coefficient
    #KZYR       = 1130.   # KZYR = dFZYR_Coeff: lateral load transfer coefficient - rear axle
    #KZYF       = 860.    # KZYF = dFZYF_Coeff: lateral load transfer coefficient - front axle

    # modified to match code: OCOA151019
    KZX        = 806.    # KZX = dFZX_Coeff = dF: longitudinal load transfer coefficient
    KZYR       = 1076.   # KZYR = dFZYR_Coeff: lateral load transfer coefficient - rear axle
    KZYF       = 675.    # KZYF = dFZYF_Coeff: lateral load transfer coefficient - front axle

    # defines polynominal for acceleration bounds
    AXC::Array{Float64,1} = [-0.000128015180401862,	0.00858618595422724,	-0.225657108071454,	3.08283259993589,	-0.000138537090018958,	0.00684702729623608,	-0.120391102052425,	-3.55886697370079]

    # vehicle Limits
    x_min    = 0.
    x_max    = 400.
    y_min    = 0.
    y_max    = 400.
    sa_min   = -30*pi/180
    sa_max   = 30*pi/180
    psi_min  = -2*pi
    psi_max  = 2*pi
    u_min    = 0.01   #5.
    u_max    = 29.
    sr_min   = -5*pi/180
    sr_max   = 5*pi/180
    jx_min   = -5.
    jx_max   = 5.

    # tire parameters
    FZ0     = 35000.0
    PCY1    = 1.5874               #Shape factor Cfy for lateral forces
    PDY1    = 0.73957              #Lateral friction Muy
    PDY2    = -0.075004            #Variation of friction Muy with load
    PEY1    = 0.37562              #Lateral curvature Efy at Fznom
    PEY2    = -0.069325            #Variation of curvature Efy with load
    PEY3    = 0.29168              #Zero order camber dependency of curvature Efy
    PKY1    = -10.289              #Maximum value of stiffness Kfy/Fznom
    PKY2    = 3.3343               #Load at which Kfy reaches maximum value
    PHY1    = 0.0056509            #Horizontal shift Shy at Fznom
    PHY2    = -0.0020257           #Variation of shift Shy with load
    PVY1    = 0.015216             #Vertical shift in Svy/Fz at Fznom
    PVY2    = -0.010365            #Variation of shift Svy/Fz with load
    PC1     = PCY1
    PD1     = PDY1 - PDY2
    PD2     = PDY2/FZ0
    PE1     = PEY1 - PEY2
    PE2     = PEY2/FZ0
    PE3     = PEY3
    PK1     = PKY1*FZ0
    PK2     = 1/(PKY2*FZ0)
    PH1     = PHY1 - PHY2
    PH2     = PHY2/FZ0
    PV1     = PVY1 - PVY2
    PV2     = PVY2/FZ0

    Caf =  -8.4138e+04	# cornering stiffness--front axle (N/rad)
    Car =  -7.8126e+04	# cornering stiffness-- rear axle (N/rad)
    #Fyf_min = -7500;Fyf_max = 7500;
    Fy_min = -7500
    Fy_max = 7500

    # constrained initial states
    x0_     = 200.
    y0_     = 0.
    psi0_   = pi/2
    v0_     = 0.
    u0_     = 15.
    sa0_    = 0.
    sr0_    = 0.
    ax0_    = 0.
    jx0_    = 0.
    r0_     = 0.

    # leave these parameters here
    Fz_min  = 1000.
    Fz_off  = 100.
    a_t     = Fz_min + 3*Fz_off  # soft tire force constraint constants
    b_t     = Fz_off
    EP      = 0.01
end
