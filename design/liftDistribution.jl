fileLoc = splitdir(@__FILE__)
using PyPlot
include("$(fileLoc[1])/../code/FLOWVLM20170929/src/FLOWVLM.jl")
vlm = FLOWVLM
save_path = "temp00/"
run_name = "unstallable"

save = false                            # Saves the vlm and opens it in paraview
display = false                         # Calls paraview
save_horseshoes = true                 # Saves the vlm's horseshoes
load_snopt = false                     # Loads and runs the optimizer
mkdir = false                          # Creates and removes the save path
save_fdom = false                      # Geneates and saves the fluid domain
add_fuselage = true                    # Includes the fuselage and vertical tail
boxed_fuselage = false                 # Models the fuselage as a box (not
                                       #    physically right)
sweep_AOA = true                       # Generates an anymation sweeping the AOA
if sweep_AOA
  plotcl = true
  save = false
  display = false
  save_horseshoes = false
  load_snopt = false
  save_fdom = false
end

global ite_num = -1

include("parameters.jl")

#
# lb = [1.0,-5.0,-5.0,-5.0,-5.0]
# ub = [5.0,15.0,15.0,15.0,15.0]
# VARS = (ub+lb)/10


################################################################################
#   OPTIMIZATION
################################################################################
function optwing(VARS,AOA)
  magVinf = VARS[1]#11.0 #m/s
  twist_wtip = VARS[4]#*pi/180      # Tip twist
  twist_wmid = VARS[3]#*pi/180      # Middle twist
  twist_wroot = VARS[2]#*pi/180   # Root twist
  twist_ctip = VARS[5]#*pi/180      # Tip twist
  twist_croot = VARS[5]#*pi/180   # Root twist

  ##############################################################################
  #   AERODYNAMICS FUNCTIONS
  ##############################################################################
  # Coefficient of friction, Cf = tau / qinf
  Cf_lam(Re) = 1.328/sqrt(Re)              # Average coeff laminar flow over flat plate
  Cf_tur(Re) = 0.455/(log10(Re))^(2.58)    # Average coeff turbulent flow over flat plate
  Cf(Re) = Re<5*10^5 ? Cf_lam(Re) : Cf_tur(Re)
  # Form factors
  k_lift(t_over_c, sweep) = 1 + 2*cos(sweep)*t_over_c + 100*(t_over_c)^4        # Lifting surface
  k_revbody(l_over_d) = l_over_d<15 ? 1.675-0.09*l_over_d+0.003*l_over_d^2 : 1  # Body of revolution

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

  ####### END OF FUNTIONS ######################################################



  ##############################################################################
  #   VLM
  ##############################################################################
  if mkdir; run(`mkdir $save_path`); end;

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


  ## Left wingtip
  wingtip_l = vlm.Wing(t_x_tip, -t_y_tip, t_z_tip, t_c_tip, t_twist_tip)
  vlm.addchord(wingtip_l, t_x_mid, -t_y_mid, t_z_mid, t_c_mid, t_twist_mid,
  n_w; r=r_w)
  vlm.addchord(wingtip_l, 0.0, 0.0, 0.0, c_wroot, twist_wroot,
  n_w; r=1.0)

  ## Right wingtip
  wingtip_r = vlm.Wing(0.0, 0.0, 0.0, c_wroot, twist_wroot)
  vlm.addchord(wingtip_r, t_x_mid, t_y_mid, t_z_mid, t_c_mid, t_twist_mid,
  n_w; r=r_w)
  vlm.addchord(wingtip_r,t_x_tip, t_y_tip, t_z_tip, t_c_tip, t_twist_tip,n_w; r=1.0)

  #CANARD
  canard = vlm.simpleWing(b_c, AR_c, tr_c, twist_croot,
  lambda_c*180/pi, gamma_c*180/pi;
  twist_tip=twist_ctip, n=n_c, r=r_c)

  Owing = [x_w,y_w,z_w]

  # Assembly
  system = vlm.WingSystem()
  vlm.addwing(system, "WingTipLeft", wingtip_l)
  vlm.addwing(system, "WingTipRight", wingtip_r)
  vlm.addwing(system, "Canard", canard)
  # vlm.addwing(system, "CenterWing", centerwing)
  # vlm.setcoordsystem(system, Owing, [1.0 0 0; 0 1 0; 0 0 1] ;wings=["CenterWing"])
  vlm.setcoordsystem(system, Owing, [1.0 0 0; 0 1 0; 0 0 1] ;wings=["WingTipRight"])
  vlm.setcoordsystem(system, Owing, [1.0 0 0; 0 1 0; 0 0 1] ;wings=["WingTipLeft"])
  vlm.setcoordsystem(system, [x_c,y_c,z_c], [1.0 0 0; 0 1 0; 0 0 1] ;wings=["Canard"])

  if add_fuselage
    if boxed_fuselage
      fuselage = vlm.Wing(0.0, -w/2, w/2, l, 0.0)
      vlm.addchord(fuselage, 0.0, w/2, w/2, l, 0.0, n_f; r=r_f)
      vlm.addchord(fuselage, 0.0, w/2, -w/2, l, 0.0, n_f; r=r_f)
      vlm.addchord(fuselage, 0.0, -w/2, -w/2, l, 0.0, n_f; r=r_f)
      vlm.addchord(fuselage, 0.0, -w/2, w/2, l, 0.0, n_f; r=r_f)
    else
      fuselage = vlm.Wing(0.0, 0.0, w/2, l, 0.0)
      vlm.addchord(fuselage, 0.0, 0.0, -w/2, l, 0.0, n_f; r=r_f)
    end

    tail = vlm.Wing(0.0, 0, 0, l_troot, 0.0)
    vlm.addchord(tail, l_troot-l_ttip, h_t, 0.0, l_ttip, 0.0, n_t; r=r_t)

    vlm.addwing(system, "Fuselage", fuselage)
    vlm.addwing(system, "VTail", tail)

    vlm.setcoordsystem(system, [x_t,y_t,z_t], [1.0 0 0; 0 0 1; 0 -1 0] ;wings=["VTail"])
  end

  # Freestream function
  Vinf(X, t) = magVinf*[cos(AOA*pi/180), 0, sin(AOA*pi/180)]
  unitVinf = Vinf(0,0)/norm(Vinf(0,0))

  # Generates the fluid domain
  if save_fdom
    P_min = [0.0, 0.0, 0.0]
    P_max = [b_w*3, b_w*3/2, b_w*7/12]
    NDivs = [12, 3, 1]*2^3
    fdom = vlm.PP.FluidDomain(P_min, P_max, NDivs)
    fdom_O = [-b_w/6, -(P_max-P_min)[2]/2, -(P_max-P_min)[3]*3/4]
    fdom_Oaxis = [0 0 0; 0 1.0 0; 0 0 0]
    fdom_Oaxis[1, :] = unitVinf[:]
    fdom_Oaxis[3, :] = cross(fdom_Oaxis[1, :], fdom_Oaxis[2, :])[:]
    vlm.PP.setcoordsystem(fdom, fdom_O, fdom_Oaxis)
  end

  # Solves the VLM
  # vlm.setVinf(system, Vinf)
  vlm.solve(system, Vinf)
  vlm.calculate_field(system, "CFtot"; S=b_w^2/AR_w)
  vlm.calculate_field(system, "Ftot";rhoinf=rhoinf, S=b_w^2/AR_w)
  vlm.calculate_field(system, "Cftot/CFtot"; S=b_w^2/AR_w)
  # S=vlm.planform_area(system)
  S=b_w^2/AR_w+ b_c^2/AR_c

  # Calculates the U field in the fluid domain
  if save_fdom
    fdom_calcU(X) = vlm.Vind(system, X) + Vinf(X,0)
    vlm.PP.calculate(fdom, [Dict("field_name"=>"U",
                                  "field_type"=>"vector",
                                  "field_function"=>fdom_calcU)])
  end

  if save
    vlm.save(system, run_name; save_horseshoes=save_horseshoes, path=save_path,
                                num= ( ite_num==-1 ? nothing : ite_num ) )

    strng = save_path
    for aux1 in vcat([ "WingTipLeft","WingTipRight", "Canard"],
                          add_fuselage ? ["Fuselage", "VTail"]:[])
      strng = strng * run_name * "_"*aux1*"_vlm.vtk;"
    end

    if save_fdom
      vlm.PP.save(fdom, run_name; path=save_path, num=ite_num)
      strng = strng * run_name * "_fdom.vtk;"
    end

    # run(`/Applications/ParaView-5.4.0.app/Contents/MacOS/paraview --data=$(strng)`)
    if display; run(`paraview --data=$(strng)`); end;
    if mkdir; run(`rm -rf $save_path`); end;
  end

  infosys = vlm.fields_summary(system)
  CLsystem= infosys["CL"]
  Liftsystem= infosys["L"]
  CDinv_system=infosys["CD"]
  Ssystem = vlm.planform_area(system)
  # ClCL = system.sol["Cl/CL"]
  # Cl = ClCL*CLsystem
  # localClmax,maxidx = find(Cl)
  # ym = wingtip_r._ym
  # println(ym)


  infocanard = vlm.fields_summary(canard)
  CLcanard= infocanard["CL"]
  Liftcanard= infocanard["L"]
  CDinv_canard=infocanard["CD"]

  # Liftwings = Liftsystem-Liftcanard


  infowingl = vlm.fields_summary(wingtip_l)
  CLwingl= infowingl["CL"]
  Liftwingl= infowingl["L"]
  CDinv_wingl=infowingl["CD"]
  S_l = vlm.planform_area(wingtip_l)

  infowingr = vlm.fields_summary(wingtip_r)
  CLwingr= infowingr["CL"]
  Liftwingr= infowingr["L"]
  CDinv_wingr=infowingr["CD"]
  S_r = vlm.planform_area(wingtip_r)
  ClCL = system.sol["Cl/CL"]
  Cl = ClCL*CLsystem
  localClmax,maxidx = findmax(Cl)
  ymr = wingtip_r._ym

  if plotcl
    if AOA==21.0
      PyPlot.plot(ymr,ones(ymr)*1.4,"k-",label = "Critical cl = 1.4")
    end
    PyPlot.plot(ymr[1:end-2],Cl[1:length(ymr)-2],"+-",label = "AOA = $AOA")
    PyPlot.xlabel("y/b")
    PyPlot.ylabel("cl")
    PyPlot.legend(loc = "best")
  end
  Swing = S_l+S_r

  Liftwings = Liftwingl + Liftwingr

  #Such values are for the full airplane, so let's split it up between components:
  CDp_f, CDp_w, CDp_c, CDp_t=calc_CDp(magVinf; Cftype="transition")
  CDp_tot = CDp_f+CDp_w+CDp_c+CDp_t

  CDtot = CDp_tot+CDinv_system

  ####### END OF VLM ###########################################################


  OBJECTIVE = CDtot*100
  CONSTRAINTS = zeros(4)
  CONSTRAINTS[1] = CLsystem-1.5
  CONSTRAINTS[2] = L-Liftwings+Liftcanard
  CONSTRAINTS[3] = Liftcanard-Liftwings*CLratio*1.3
  CONSTRAINTS[4] = Liftwings*CLratio*.7-Liftcanard


  Dinv = (Liftwings)^2/(0.5*rhoinf*magVinf^2*pi*b_w^2) #TODO: ask about how to do this for the system, ie wing and canard
  CDinv_elliptic = Dinv/(0.5*rhoinf*magVinf^2*Swing)
  einv_wing = (CDinv_wingr+CDinv_wingl)/CDinv_elliptic

  Dinv = (Liftwings+Liftcanard)^2/(0.5*rhoinf*magVinf^2*pi*b_w^2) #TODO: ask about how to do this for the system, ie wing and canard
  CDinv_elliptic = Dinv/(0.5*rhoinf*magVinf^2*Ssystem)
  einv_system = CDinv_system/CDinv_elliptic

  println("CONSTRAINTS $CONSTRAINTS
  Liftcanard $Liftcanard
  Liftwings $Liftwings
  Liftsystem $Liftsystem
  CLcanard $CLcanard
  Clsystem $CLsystem
  Vars $VARS
  einv_wing $einv_wing
  einv_system $einv_system
  localClmax $localClmax
  L/D $(CLsystem/CDtot)")


      return OBJECTIVE, CONSTRAINTS, false
end
####### END OF OPTIMIZATION ####################################################







################################################################################
#   MAIN
################################################################################
if load_snopt
  #Load the models
  #Load Snopt
  push!(LOAD_PATH,"$(fileLoc[1])/../../Snopt.jl/src")
  using Snopt
end

# numVars = 4
# Vinf, root twist, middle twist, tip twist, canard twist
lb = [5.9,4.0,4.0,0.001,0.001]
ub = [11.0,15.0,12.0,8.0,5.0]
xstart = (ub+lb)/2.0
xstart = [11.0, 7.0, 5.4953, 5.1, 4.24008]

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

function optwing(VARS)
    return optwing(VARS,AOA)
end

if load_snopt
  AOA = 0.0
  xopt, fopt, info = snopt(optwing, xstart, lb, ub, options)
  println(fopt)
  println(xopt)
  println(info)
end

if sweep_AOA
  AOA1 = linspace(21.0,-3.0,7)
  for i = 1:length(AOA1)
    AOA = AOA1[i]
    xopt2 = [5.40, 4.72145, 6.06702, 5.36544, 4.24697]
    out = optwing(xopt2)
    global ite_num = i
  end

else
  xopt2 = [5.40, 4.72145, 6.06702, 5.36544, 4.24697]
  AOA = 20.0
  out = optwing(xopt2)
end
####### END OF MAIN ####################################################
