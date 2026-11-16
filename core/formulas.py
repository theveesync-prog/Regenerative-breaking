"""Formula registry: every calculated quantity has a name, LaTeX, a plain-text
tooltip and a function that substitutes live numbers.

`ctx` is a flat dict built by the UI from the input parameters + summary.
"""

F = {}


def _add(key, group, title, latex, plain, subst=None, symbols=""):
    F[key] = dict(group=group, title=title, latex=latex, plain=plain, subst=subst, symbols=symbols)


# ---- Vehicle dynamics --------------------------------------------------
_add("v0", "Vehicle dynamics", "Initial speed (m/s)", r"v_0 = \frac{v_{kmh}}{3.6}",
     "v0 = v_kmh / 3.6", lambda c: rf"v_0 = {c['v0_kmh']:.1f}/3.6 = {c['v0_ms']:.2f}\ \mathrm{{m/s}}")
_add("decel_dist", "Vehicle dynamics", "Deceleration from stopping distance", r"a = \frac{v_0^2}{2d}",
     "a = v0^2 / (2 d)", lambda c: rf"a = {c['v0_ms']:.2f}^2 / (2\times{c['stop_distance']:.1f}) = {c['decel']:.2f}\ \mathrm{{m/s^2}}")
_add("t_stop", "Vehicle dynamics", "Stopping time", r"t_s = \frac{v_0}{a}",
     "t_stop = v0 / a", lambda c: rf"t_s = {c['v0_ms']:.2f}/{c['decel']:.2f} = {c['stop_time_s']:.2f}\ \mathrm{{s}}")
_add("s_stop", "Vehicle dynamics", "Stopping distance", r"s = \frac{v_0^2}{2a}",
     "s = v0^2 / (2 a)", lambda c: rf"s = {c['v0_ms']:.2f}^2/(2\times{c['decel']:.2f}) = {c['stop_distance_m']:.1f}\ \mathrm{{m}}")
_add("v_t", "Vehicle dynamics", "Speed vs time", r"v(t) = v_0 - a\,t",
     "v(t) = v0 - a t")
_add("omega_w", "Vehicle dynamics", "Wheel angular speed", r"\omega_w = \frac{v}{r}",
     "omega_wheel = v / r")
_add("omega_m", "Vehicle dynamics", "Motor angular speed and RPM", r"\omega_m = G\,\omega_w,\quad N = \frac{60\,\omega_m}{2\pi}",
     "omega_motor = G * omega_wheel;  RPM = 60 * omega_motor / (2 pi)",
     lambda c: rf"N_{{peak}} = 60\times{c['gear_ratio']:.2f}\times{c['v0_ms']:.2f}/(2\pi\times{c['wheel_radius']:.3f}) = {c['peak_rpm']:.0f}\ \mathrm{{rpm}}")
_add("gear", "Drivetrain", "Auto gear ratio", r"G = \frac{\omega_{m,max}\; r}{v_{top}}",
     "G = (omega_motor_max * r_wheel) / v_top  -> motor hits max RPM at top speed",
     lambda c: rf"G = ({c['max_rpm']:.0f}\cdot2\pi/60)\times{c['wheel_radius']:.3f}/{c['top_speed_ms']:.1f} = {c['auto_gear']:.2f}")
# ---- Road resistance ---------------------------------------------------
_add("f_roll", "Road resistance", "Rolling resistance", r"F_{roll} = C_{rr}\,m\,g\cos\theta",
     "F_roll = Crr * m * g * cos(theta)", lambda c: rf"F_{{roll}} = {c['crr']:.4f}\times{c['mass']:.0f}\times9.81 = {c['f_roll']:.0f}\ \mathrm{{N}}")
_add("f_drag", "Road resistance", "Aerodynamic drag", r"F_{drag} = \tfrac12\rho\,C_d A\,v^2",
     "F_drag = 0.5 * rho * Cd * A * v^2  (rho = 1.225 kg/m3)", lambda c: rf"F_{{drag}}(v_0) = 0.5\times1.225\times{c['cd']:.2f}\times{c['area']:.2f}\times{c['v0_ms']:.1f}^2 = {c['f_drag0']:.0f}\ \mathrm{{N}}")
_add("f_res", "Road resistance", "Total road resistance", r"F_{res} = F_{roll}+F_{drag}+m g\sin\theta",
     "F_res = F_roll + F_drag + m g sin(theta)")
# ---- Braking system ----------------------------------------------------
_add("f_req", "Braking", "Required braking force", r"F_{req} = m\,a - F_{res}",
     "F_req = m a - F_res   (road resistance already helps slow the car)",
     lambda c: rf"F_{{req}}(0) = {c['mass']:.0f}\times{c['decel']:.2f} - {c['f_res0']:.0f} = {c['peak_required_brake_force_n']:.0f}\ \mathrm{{N}}")
_add("f_regen", "Braking", "Regenerative force (limits)", r"F_{reg}=\min\!\left(\lambda F_{req},\ \frac{T_{avail}G}{\eta_{dt} r},\ \frac{P_{chg}}{\eta_m\eta_{dt} v}\right)",
     "F_regen = min(axle share * F_req, torque limit, battery power limit)")
_add("f_fric", "Braking", "Friction braking", r"F_{fric} = F_{req}-F_{reg},\quad P_{fric}=F_{fric}\,v",
     "F_fric = F_req - F_regen;  P_fric = F_fric * v")
_add("torque", "Motor", "Regenerative motor torque", r"T_m = \frac{F_{reg}\,r\,\eta_{dt}}{G}",
     "T_motor = F_regen * r * eta_dt / G")
_add("t_env", "Motor", "Torque / power envelope", r"T_{avail}=\min\!\left(T_{max},\ \frac{P_{max}}{\omega_m}\right)\cdot\min\!\left(1,\frac{N}{N_{fade}}\right)",
     "T_avail = min(Tmax, Pmax/omega_m) * min(1, rpm/rpm_fade)   (constant-torque, then constant-power, fades near 0 rpm)")
_add("p_mech", "Motor", "Motor mechanical power", r"P_m = T_m\,\omega_m = F_{reg}\,v\,\eta_{dt}",
     "P_mech = T_motor * omega_motor = F_regen * v * eta_dt")
_add("eta_m", "Motor", "Motor efficiency map", r"\eta_m=\eta_0-c_l(l-0.7)^2-c_s(s-0.45)^2",
     "eta = eta_rated - c_load*(T/Tmax - 0.7)^2 - c_speed*(rpm/rpm_max - 0.45)^2  (or constant)")
# ---- Battery -----------------------------------------------------------
_add("p_bat", "Battery", "Battery charging power", r"P_{bat} = \min\!\left(P_m\,\eta_m,\ P_{chg,max}(SOC)\right)",
     "P_bat = min(P_mech * eta_motor, P_charge_max(SOC)); taper to zero as SOC -> SOC_max")
_add("soc", "Battery", "SOC update", r"SOC_{k+1} = \min\!\left(SOC_k + \frac{P_{bat}\eta_c\,\Delta t}{3.6\times10^{6}\,C}\,100,\ SOC_{max}\right)",
     "SOC += P_bat * eta_chg * dt / (capacity_J) * 100, capped at SOC_max")
# ---- Energy ------------------------------------------------------------
_add("ke", "Energy", "Kinetic energy", r"E_k = \tfrac12 m v_0^2",
     "KE = 0.5 m v0^2", lambda c: rf"E_k = 0.5\times{c['mass']:.0f}\times{c['v0_ms']:.2f}^2/3.6\times10^6 = {c['ke_kwh']:.4f}\ \mathrm{{kWh}}")
_add("e_rec", "Energy", "Recovered electrical energy", r"E_{elec} = \int P_{bat}\,dt",
     "E_elec = integral of battery charging power over the stop",
     lambda c: rf"E_{{elec}} = {c['recovered_kwh']:.4f}\ \mathrm{{kWh}}")
_add("balance", "Energy", "Energy balance", r"E_k = E_{res} + E_{fric} + E_{elec} + E_{dt,loss} + E_{m,loss}",
     "KE = road resistance + friction + recovered + drivetrain loss + motor loss (should close to ~0 %)",
     lambda c: rf"\text{{residual}} = {c['balance_residual_pct']:.4f}\,\%")
_add("recovery", "Energy", "Energy recovery %", r"\eta_{rec} = \frac{E_{elec}}{E_k}\times100",
     "recovery % = E_elec / KE * 100", lambda c: rf"\eta_{{rec}} = {c['recovered_kwh']:.4f}/{c['ke_kwh']:.4f}\times100 = {c['recovery_pct']:.1f}\,\%")


def plain(key):
    f = F[key]
    return f"{f['title']}: {f['plain']}"
