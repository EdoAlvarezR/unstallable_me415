##############################################################################
#   PARAMETERS
##############################################################################
# ------ CONSTANTS
g = 9.82           # (m/s^2) gravity
rhoinf= 1.225      # (kg/m^3) air density at sea level and 15Cdegs
mu = 1.846/10^5    # (kg/m*s) air dynamic viscosity

# ------ DESIGN PARAMETERS
magVinf = 5.40     # (m/s) pptimum cruise speed

# ------ MATERIALS
rho_c = 0.483       # (kg/m^2) Cloroplast area density
rho_f = 24.8     # (kg/m^3) HD EPS foam density

# ------ GEOMETRIC PARAMETERS
# Fuselage
w = 8/100         # (m) width
l = (560.000-230.000)/1000          # (m) length

# Wing
b_w = 475*2/1000        # (m) span
lambda_w = 30*pi/180        # (rad) sweep
c_wtip = 0.10     # (m) tip chord
c_wmid= 0.16     # (m) middle chord
c_wroot = 0.20    # (m) root chord
t_w = 0.1         # (ratio) ave max thickness/chord
barc_w = (c_wtip+c_wroot)/2
AR_w = 0.5*b_w/c_wtip # Aspect ratio of center section
tr_w = c_wtip/c_wmid      # Taper ratio
twist_wtip = 5.36544*pi/180      # Tip twist
twist_wmid = 6.06702*pi/180      # Middle twist
twist_wroot = 4.72145*pi/180   # Root twist
gamma_w = 5.0*pi/180# Dihedral


# canard
vertical_offset = .0 #m
b_c = 2*200/1000     # (m) span
lambda_c = 0*pi/180        # (rad) sweep
c_ctip = 0.1     # (m) tip chord
c_croot = 0.1    # (m) root chord
t_c = 0.15        # (ratio) ave max thickness/chord
barc_c = (c_ctip+c_croot)/2
AR_c = b_c/c_ctip   # Aspect ratio
tr_c = c_ctip/c_croot      # Taper ratio
twist_ctip = 4.24697*pi/180      # Tip twist
twist_croot = 4.24697*pi/180   # Root twist
gamma_c = 0*pi/180# Dihedral

# Winglet
h_wl = 153/1000        # (m) height
l_wlroot = 0.153    # (m) root length
l_wltip = 0.025# (m) tip length
t_wl = 0.04        # (m) thickness
barc_wl = (l_wltip+l_wlroot)/2

# # vertical tail
# h_t = 0.12        # (m) height
# l_troot = 0.15    # (m) root length
# l_ttip = l_troot/3# (m) tip length
# t_t = 0.04        # (m) thickness
# barc_t = (l_ttip+l_troot)/2


# Center wing is same width as canard
# centerwing = vlm.simpleWing(b_c, AR_w, tr_w, twist_wroot*180/pi,
# lambda_w*180/pi, gamma_w*180/pi;
# twist_tip=twist_wmid*180/pi, n=n_w, r=r_w)
# Wing tips
t_y_tip = b_w/2
t_x_tip = b_w/2*tan(lambda_w) #t_y_tip*tan(lambda3)
t_z_tip = 0.0#b_c/2*tan(lambda_w)+b_w/2*tan(lambda_w)
t_c_tip = c_wtip
t_twist_tip = twist_wtip
t_y_mid = b_c/2 #starts at the center section
t_x_mid = b_c/2*tan(lambda_w)
t_z_mid = 0.0#b_c/2*tan(lambda_w*180/pi)
t_c_mid = c_wmid
t_twist_mid = twist_wmid

Sref = 2*b_w*barc_w      # Reference area

# ------ AIRPLANE ASSEMBLY
x_w, y_w, z_w = l*2/4, 0, -w*1/8   # Wing's position in fuselage
x_c, y_c, z_c = l*1/16, 0, z_w+vertical_offset  # Canard's position in fuselage
# x_t, y_t, z_t = l-l_troot, 0, w/2  # Tail's position in fuselage

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
M_wl = rho_c * barc_wl * h_wl
# M_t = rho_f * barc_t * h_t*t_t
# Total mass
Mtot = sum([M_base, M_f, M_w, M_c, M_wl])
# Required lift in cruise
L = Mtot*g

println("Total mass: $(Mtot) (kg)")

####### END OF PARAMETERS ####################################################
