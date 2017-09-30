
include("../code/FLOWVLM20170929/src/FLOWVLM.jl")
vlm = FLOWVLM
save_path = "temp00/"
run_name = "unstallable"


function optwing(VARS)

    function calc_CDp(Vinf; Cftype="transition")

        if Cftype=="laminar"
            _Cf = Cf_lam
        elseif Cftype=="turbulent"
            _Cf = Cf_tur
        else
            _Cf = Cf
        end

        qinf = calc_qinf(Vinf)

        # Fuselage
        deff = sqrt(4*w^2/pi)
        CDp_f = k_revbody(l/deff) * _Cf(calc_Re(Vinf, l)) * (4*w*l)/Sref
        # Wing
        Sexp_w = 2*( (b_w/2 - w/2 )*barc_w )
        CDp_w = k_lift(t_w, lambda_w) * _Cf(calc_Re(Vinf, barc_w)) * (
        2*(1+0.2*t_w)*Sexp_w)/Sref
        # Canard
        Sexp_c = 2*( (b_c/2 - w/2 )*barc_c )
        CDp_c = k_lift(t_c, lambda_c) * _Cf(calc_Re(Vinf, barc_c)) * (
        2*(1+0.2*t_c)*Sexp_c)/Sref
        # Horizontal tail
        tail_t_over_c = t_t/barc_t
        lambda_tail = atan2(l_troot-l_ttip, h_t)
        Sexp_t = h_t*barc_t
        CDp_t = k_lift(tail_t_over_c, lambda_tail) * _Cf(
        calc_Re(Vinf, barc_t)) * (
        2*(1+0.2*tail_t_over_c)*Sexp_t)/Sref

        return [CDp_f, CDp_w, CDp_c, CDp_t]
    end

    function calc_CDtot(Vinf; Cftype="transition")
        qinf = calc_qinf(Vinf)
        K = 0.38

        # Distributing the lift between lifting bodies
        CL = L / (qinf*Sref)         # Total CL (full airplane)
        CL_t = 0                     # CL on tail
        CL_c = CL*CLratio            # CL on canard
        CL_w = CL - (CL_t+CL_c)      # CL on wing

        # Parasitic drag
        CDp = calc_CDp(Vinf; Cftype=Cftype)
        CDp_f, CDp_w, CDp_c, CDp_t = CDp     # Each component's

        # Induced drag
        ## Wing
        einv = 0.99*(1-2*(w/b_w)^2)
        AR = b_w / barc_w
        CDiv_w = K*CDp_w*CL_w^2
        CDi_w = CL_w^2/(pi*AR*einv) + CDiv_w
        ## Canard
        einv = 0.99*(1-2*(w/b_c)^2)
        AR = b_c / barc_c
        CDiv_c = K*CDp_c*CL_c^2
        CDi_c = CL_c^2/(pi*AR*einv) + CDiv_c
        ## Vertical tail
        einv = 0.99
        AR = h_t / barc_t
        CDiv_t = K*CDp_t*CL_t^2
        CDi_t = CL_t^2/(pi*AR*einv) + CDiv_t

        return [0.0, CDi_w, CDi_c, CDi_t] + CDp
    end



    # ------ CONSTANTS
    g = 9.82          # (m/s^2) gravity
    rhoinf = 1.225    # (kg/m^3) air density at sea level and 15Cdegs
    mu = 1.846/10^5   # (kg/m*s) air dynamic viscosity

    # ------ MATERIALS
    rho_c = 3.6       # (kg/m^2) Cloroplast area density
    rho_f = 24.8        # (kg/m^3) HD EPS foam density

    #Design Variables
    AOA = 0.0#VARS[1]#5.0
    magVinf = VARS[1]#11.0 #m/s

    # ------ GEOMETRIC PARAMETERS
    # Fuselage
    w = 8/100         # (m) width
    l = 0.75          # (m) length

    # Wing
    b_w = 0.95        # (m) span
    lambda_w = 30*pi/180        # (rad) sweep
    c_wtip = 0.10     # (m) tip chord
    c_wmid= 0.10     # (m) middle chord
    c_wroot = 0.20    # (m) root chord
    t_w = 0.1         # (ratio) ave max thickness/chord
    barc_w = (c_wtip+c_wroot)/2
    AR_w = b_w/c_wtip # Aspect ratio
    tr_w = c_wtip/c_wmid      # Taper ratio
    twist_wtip = VARS[2]*pi/180      # Tip twist
    twist_wmid = VARS[3]*pi/180      # Middle twist
    twist_wroot = VARS[4]*pi/180   # Root twist
    gamma_w = 0*pi/180# Dihedral

    # canard
    b_c = 0.5*b_w     # (m) span
    lambda_c = 0*pi/180        # (rad) sweep
    c_ctip = 0.10     # (m) tip chord
    c_croot = 0.10    # (m) root chord
    t_c = 0.15        # (ratio) ave max thickness/chord
    barc_c = (c_ctip+c_croot)/2
    AR_c = b_c/c_ctip   # Aspect ratio
    tr_c = c_ctip/c_croot      # Taper ratio
    twist_ctip = 5*pi/180      # Tip twist
    twist_croot = 5*pi/180   # Root twist
    gamma_c = 0*pi/180# Dihedral
    # vertical tail
    h_t = 0.17        # (m) height
    l_troot = 0.20    # (m) root length
    l_ttip = l_troot/3# (m) tip length
    t_t = 0.04        # (m) thickness
    barc_t = (l_ttip+l_troot)/2

    Sref = 2*b_w*barc_w      # Reference area

    # ------ AIRPLANE ASSEMBLY
    x_w, y_w, z_w = l*2/4, 0, w/4     # Wing's position in fuselage
    x_c, y_c, z_c = l*1/16, 0, 0.0     # Canard's position in fuselage
    x_t, y_t, z_t = l-l_troot, 0, 0.0  # Tail's position in fuselage

    # ------ USEFUL FUNCTIONS
    calc_Re(Vinf, l) = rhoinf*Vinf*l/mu
    calc_qinf(Vinf) = 1/2*rhoinf*Vinf.^2
    Vinfmin, Vinfmax = 3, 20 # (m/s)


    # ------ ASSUMPTIONS
    CLratio = 0.1    # Canard-wing distribution of lift
    #   CLratio = CLcanard/CLtot

    # Mass of electronic components
    M_base = (9 + 16 + 47 + 14 + 36 + 3 + 32 + 15 + 9*4 + 50)/1000
    # Mass of structural components
    M_f = rho_c * 4*w*l
    M_w = rho_f * b_w/2/cos(lambda_w) * barc_w^2 * t_w
    M_c = rho_f * b_c/2/cos(lambda_c) * barc_c^2 * t_c
    M_t = rho_f * barc_t * h_t*t_t
    # Total mass
    Mtot = sum([M_base, M_f, M_w/2, M_c/2, M_t/2]) #Masses of airfoils will be less than square
    # Required lift in cruise
    L = Mtot*g



    # Coefficient of friction, Cf = tau / qinf
    Cf_lam(Re) = 1.328/sqrt(Re)              # Average coeff laminar flow over flat plate
    Cf_tur(Re) = 0.455/(log10(Re))^(2.58)    # Average coeff turbulent flow over flat plate
    Cf(Re) = Re<5*10^5 ? Cf_lam(Re) : Cf_tur(Re)
    # Form factors
    k_lift(t_over_c, sweep) = 1 + 2*cos(sweep)*t_over_c + 100*(t_over_c)^4        # Lifting surface
    k_revbody(l_over_d) = l_over_d<15 ? 1.675-0.09*l_over_d+0.003*l_over_d^2 : 1  # Body of revolution


    # VLM


    # run(`mkdir $save_path`)

    # Number of lattices
    n = 1
    n_w = 20*n
    n_c = 10*n
    n_t = 5*n
    n_f = 2*n
    # Lattices expansions
    r_w = 5.0
    r_c = 5.0
    r_t = 1.0
    r_f = 1.0


    # Creates components

    # Center wing is same width as canard
    centerwing = vlm.simpleWing(b_c, AR_w, tr_w, twist_wroot*180/pi,
    lambda_w*180/pi, gamma_w*180/pi;
    twist_tip=twist_wmid*180/pi, n=n_w, r=r_w)
    # Wing tips
    t_y_tip = b_w/2
    t_x_tip = b_c/2*tan(lambda_w*180/pi)+b_w/2*tan(lambda_w*180/pi) #t_y_tip*tan(lambda3)
    t_z_tip = b_c/2*tan(lambda_w*180/pi)+b_w/2*tan(lambda_w*180/pi)
    t_c_tip = c_wtip
    t_twist_tip = twist_wtip
    t_y_mid = b_c/2 #starts at the center section
    t_x_mid = b_c/2*tan(lambda_w*180/pi)
    t_z_mid = b_c/2*tan(lambda_w*180/pi)
    t_c_mid = c_wmid
    t_twist_mid = twist_wmid

    ## Left wingtip
    wingtip_l = vlm.Wing(t_x_tip, -t_y_tip, t_z_tip, t_c_tip, t_twist_tip)
    vlm.addchord(wingtip_l, t_x_mid, t_y_mid, t_z_mid, t_c_mid, t_twist_mid,
    n_w; r=r_w)
    ## Left wingtip
    wingtip_r = vlm.Wing(t_x_mid, t_y_mid, t_z_mid, t_c_mid, t_twist_mid)
    vlm.addchord(wingtip_r, t_x_tip, t_y_tip, t_z_tip, t_c_tip, t_twist_tip,
    n_w; r=r_w)



    #CANARD
    canard = vlm.simpleWing(b_c, AR_c, tr_c, twist_croot*180/pi,
    lambda_c*180/pi, gamma_c*180/pi;
    twist_tip=twist_ctip*180/pi, n=n_c, r=r_c)

    fuselage = vlm.Wing(0.0, -w/2, w/2, l, 0.0)
    vlm.addchord(fuselage, 0.0, w/2, w/2, l, 0.0, n_f; r=r_f)
    vlm.addchord(fuselage, 0.0, w/2, -w/2, l, 0.0, n_f; r=r_f)
    vlm.addchord(fuselage, 0.0, -w/2, -w/2, l, 0.0, n_f; r=r_f)
    vlm.addchord(fuselage, 0.0, -w/2, w/2, l, 0.0, n_f; r=r_f)

    tail = vlm.Wing(0.0, 0, 0, l_troot, 0.0)
    vlm.addchord(tail, l_troot-l_ttip, h_t, 0.0, l_ttip, 0.0, n_t; r=r_t)

    # Assembly
    system = vlm.WingSystem()
    vlm.addwing(system, "Fuselage", fuselage)
    vlm.addwing(system, "CenterWing", centerwing)
    vlm.addwing(system, "WingTipLeft", wingtip_l)
    vlm.addwing(system, "WingTipRight", wingtip_r)
    vlm.addwing(system, "Canard", canard)
    vlm.addwing(system, "VTail", tail)
    vlm.setcoordsystem(system, [x_w,y_w,z_w], [1.0 0 0; 0 1 0; 0 0 1] ;wings=["CenterWing"])
    # vlm.setcoordsystem(system, [x_w,y_w,z_w], [1.0 0 0; 0 1 0; 0 0 1] ;wings=["wingtip_r"])
    # vlm.setcoordsystem(system, [x_w,y_w,z_w], [1.0 0 0; 0 1 0; 0 0 1] ;wings=["wingtip_l"])
    vlm.setcoordsystem(system, [x_c,y_c,z_c], [1.0 0 0; 0 1 0; 0 0 1] ;wings=["Canard"])
    vlm.setcoordsystem(system, [x_t,y_t,z_t], [1.0 0 0; 0 0 1; 0 -1 0] ;wings=["VTail"])



    Vinf(X, t) = magVinf*[cos(AOA*pi/180), 0, sin(AOA*pi/180)]
    vlm.setVinf(system, Vinf)
    vlm.save(system, run_name; save_horseshoes=true, path=save_path)

    # strng = save_path
    # for aux1 in ["Fuselage", "Wing", "VTail", "Canard"]
    #   strng = strng * run_name * "_"*aux1*"_vlm.vtk;"
    # end
    # run(`/Applications/ParaView-5.4.0.app/Contents/MacOS/paraview --data=$(strng)`)
    # run(`rm -rf $save_path`)


    function Vinf(X, t)
        return magVinf*[ cos(AOA*pi/180), 0.0, sin(AOA*pi/180)]
    end

    vlm.solve(system, Vinf)
    vlm.calculate_field(system, "CFtot"; S=b_w^2/AR_w)
    S=b_w^2/AR_w

    info = vlm.fields_summary(system)
    CLsystem= info["CL"]
    Lift = CLsystem*0.5*1.10*magVinf^2*S
    CDinv_system=info["CD"]




    #Such values are for the full airplane, so let's split it up between components:


    CDp_f, CDp_w, CDp_c, CDp_t=calc_CDp(magVinf; Cftype="transition")
    CDp_tot = CDp_f+CDp_w+CDp_c+CDp_t

    CDtot = CDp_tot+CDinv_system

    OBJECTIVE = CDtot
    CONSTRAINTS = zeros(2)
    CONSTRAINTS[1] = CLsystem-1.0
    CONSTRAINTS[2] = L-Lift

    println("CONSTRAINTS $CONSTRAINTS
    Lift $Lift
    Clsystem $CLsystem
    Vars $VARS")


    return OBJECTIVE, CONSTRAINTS, false
end

#Load the models
fileLoc = splitdir(@__FILE__)
#Load Snopt
push!(LOAD_PATH,"$(fileLoc[1])/../../Snopt.jl/src")
using Snopt

numVars = 5
# Vinf, root twist, middle twist, tip twist
lb = [1.0,0.0,0.0,-5.0]
ub = [30.0,15.0,15.0,15.0]
xstart = (ub+lb)/2

# ----- Define Optimizer Options ----- #
options = Dict{String, Any}()
options["Derivative level"] = 0
# options["Function precision"] = 3.00E-4
options["Difference interval"] = 1e-5
options["Central difference interval"] = 1e-5
options["Iterations limit"] = 1e8
options["Major iterations limit"] = 500
options["Minor iterations limit"]= 1e8
options["Major optimality tolerance"] = 1e-4
options["Minor optimality  tolerance"] = 1e-4
options["Major feasibility tolerance"] = 1e-4
options["Minor feasibility tolerance"] = 1e-4
options["Minor print level"] = 1
options["Print frequency"] = 100
options["Scale option"] = 2
options["Scale tolerance"] = .95


xopt, fopt, info = snopt(optwing, xstart, lb, ub, options)
println(fopt)
println(xopt)
println(info)
