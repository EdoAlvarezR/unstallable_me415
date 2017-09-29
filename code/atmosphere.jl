
using PyPlot

# -------- ASSUMPTIONS
Tsl = 20+273  # (K) Sea-level temperature
Psl = 101352.9# (Pa) Sea-level pressure
R = 286.9     # (J/kgK)=(m^2/s^2kgK) Air gas constant
# gamma = 1.4   # Heat capacity ratio of air


# ------- ATMOSPHERIC MODELS
# Temperature (h in km)
T_curve(h, T0) = T0 - 71.5 + 2*log(1+exp(35.75-3.25*h) + exp(-3 + 0.0003*h.^3))

# Pressure (h in km)
P_curve(h, P0) = P0 * exp(-0.118*h - 0.0015*h.^2./(1-0.018*h+0.0011*h.^2))

# Air density (h in km)
rho_curve(h, P0, T0) = P_curve(h, P0)./(R*T_curve(h, T0))
