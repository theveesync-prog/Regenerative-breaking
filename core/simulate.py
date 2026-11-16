"""Regenerative-braking time simulation for one vehicle + motor + battery."""
import math
import numpy as np
import pandas as pd

from . import vehicle as veh
from . import motor as mot
from . import battery as bat

J_PER_KWH = 3.6e6


def default_params():
    return dict(
        mass=1500.0, v0_kmh=60.0, mode="decel", decel=3.0, stop_distance=40.0,
        wheel_radius=0.32, crr=0.010, cd=0.30, area=2.2, grade_deg=0.0, mu=0.8,
        gear_ratio=9.0, drivetrain_eff=0.95, regen_share=1.0,
        battery_kwh=40.0, soc0=50.0, soc_max=95.0, max_charge_kw=60.0, charge_eff=0.98,
        const_eff=False, steps=1500,
    )


def simulate(p, motor):
    """Run one braking event.

    p     : parameter dict (see default_params)
    motor : dict with max_torque_nm, max_power_kw, efficiency, max_rpm,
            regen_fade_rpm, c_load, c_speed
    Returns dict(df=DataFrame, summary=dict).
    """
    m, r, G, eta_dt = p["mass"], p["wheel_radius"], p["gear_ratio"], p["drivetrain_eff"]
    v0 = veh.kmh_to_ms(p["v0_kmh"])
    a = veh.decel_from_distance(v0, p["stop_distance"]) if p["mode"] == "distance" else p["decel"]
    grade = math.radians(p["grade_deg"])
    t_stop = v0 / a
    s_stop = v0 * v0 / (2 * a)

    t = np.linspace(0.0, t_stop, int(p["steps"]) + 1)
    v = np.maximum(v0 - a * t, 0.0)
    dt = t[1] - t[0]

    t_max, p_max = motor["max_torque_nm"], motor["max_power_kw"] * 1000.0
    cap_j = p["battery_kwh"] * J_PER_KWH
    f_roll = veh.rolling_force(m, p["crr"], grade)
    f_grade = veh.grade_force(m, grade)

    n = len(t)
    cols = {k: np.zeros(n) for k in (
        "f_drag", "f_res", "f_req", "f_regen", "f_fric", "torque", "rpm", "p_mech_motor",
        "p_elec", "p_fric", "soc", "batt_kwh", "eta_motor", "limit")}
    # limit codes: 0 none, 1 torque/power, 2 battery power, 3 SOC cap, 4 axle share
    soc = p["soc0"]
    for i in range(n):
        vi = v[i]
        f_drag = veh.drag_force(p["cd"], p["area"], vi)
        f_res = f_roll + f_drag + f_grade
        f_req = max(m * a - f_res, 0.0)
        omega_w = vi / r
        rpm = omega_w * G * 60.0 / (2 * math.pi)
        f_axle = f_req * p["regen_share"]
        t_avail = mot.available_torque(rpm, t_max, p_max, motor["regen_fade_rpm"])
        f_trq = t_avail * G / (eta_dt * r)
        p_lim = bat.max_charge_power(soc, p["soc_max"], p["max_charge_kw"])

        eta_m = motor["efficiency"]
        f_reg, limit = 0.0, 0
        for _ in range(3):  # efficiency depends on torque -> short fixed-point loop
            f_bat = p_lim / (eta_m * eta_dt * vi) if vi > 1e-6 else 0.0
            f_reg = min(f_axle, f_trq, f_bat)
            torque = f_reg * r * eta_dt / G
            eta_m = mot.efficiency(rpm, torque, motor, p["const_eff"])
        if f_reg < f_req - 1e-9:
            if f_reg == f_bat:
                limit = 3 if p_lim == 0.0 else 2
            elif f_reg == f_trq:
                limit = 1
            else:
                limit = 4
        torque = f_reg * r * eta_dt / G
        p_mech_m = torque * (rpm * 2 * math.pi / 60.0)
        p_el = p_mech_m * eta_m

        cols["f_drag"][i], cols["f_res"][i], cols["f_req"][i] = f_drag, f_res, f_req
        cols["f_regen"][i], cols["f_fric"][i] = f_reg, f_req - f_reg
        cols["torque"][i], cols["rpm"][i] = torque, rpm
        cols["p_mech_motor"][i], cols["p_elec"][i] = p_mech_m, p_el
        cols["p_fric"][i] = (f_req - f_reg) * vi
        cols["soc"][i], cols["batt_kwh"][i] = soc, soc / 100.0 * p["battery_kwh"]
        cols["eta_motor"][i], cols["limit"][i] = eta_m, limit
        soc = min(soc + p_el * p["charge_eff"] * dt / cap_j * 100.0, p["soc_max"])

    df = pd.DataFrame({"t": t, "v": v, "v_kmh": v * 3.6, "s": v0 * t - 0.5 * a * t**2})
    df["omega_wheel"] = v / r
    df["omega_motor"] = df["omega_wheel"] * G
    for k, arr in cols.items():
        df[k] = arr

    tz = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    ke0 = 0.5 * m * v0**2
    e_res = tz(df.f_res * v, t)
    e_brake = tz(df.f_req * v, t)
    e_regen_wheel = tz(df.f_regen * v, t)
    e_fric = tz(df.p_fric, t)
    e_motor_mech = tz(df.p_mech_motor, t)
    e_elec = tz(df.p_elec, t)
    e_dt_loss = e_regen_wheel - e_motor_mech
    e_motor_loss = e_motor_mech - e_elec
    e_batt_loss = e_elec * (1 - p["charge_eff"])
    kwh = lambda j: j / J_PER_KWH
    balance = ke0 - (e_res + e_fric + e_elec + e_dt_loss + e_motor_loss)

    lim = df["limit"].values
    summary = dict(
        v0_ms=v0, decel=a, stop_time_s=t_stop, stop_distance_m=s_stop,
        ke_kwh=kwh(ke0), road_resistance_kwh=kwh(e_res), total_braking_kwh=kwh(e_brake),
        mech_regen_kwh=kwh(e_motor_mech), wheel_regen_kwh=kwh(e_regen_wheel),
        friction_kwh=kwh(e_fric), recovered_kwh=kwh(e_elec),
        drivetrain_loss_kwh=kwh(e_dt_loss), motor_loss_kwh=kwh(e_motor_loss),
        battery_loss_kwh=kwh(e_batt_loss), conversion_loss_kwh=kwh(e_dt_loss + e_motor_loss),
        recovery_pct=100 * e_elec / ke0 if ke0 else 0.0,
        recovery_of_braking_pct=100 * e_elec / e_brake if e_brake else 0.0,
        regen_share_pct=100 * e_regen_wheel / e_brake if e_brake else 0.0,
        final_soc=float(df.soc.iloc[-1]), delta_soc=float(df.soc.iloc[-1]) - p["soc0"],
        peak_motor_power_kw=float(df.p_mech_motor.max()) / 1000,
        peak_battery_power_kw=float(df.p_elec.max()) / 1000,
        peak_torque_nm=float(df.torque.max()), peak_rpm=float(df.rpm.max()),
        avg_motor_eff=e_elec / e_motor_mech if e_motor_mech else 0.0,
        balance_residual_pct=100 * abs(balance) / ke0 if ke0 else 0.0,
        grip_limit_decel=p["mu"] * veh.G_ACC,
        grip_exceeded=bool(a > p["mu"] * veh.G_ACC),
        pct_time_torque_limited=100 * float(np.mean(lim == 1)),
        pct_time_battery_limited=100 * float(np.mean(lim == 2)),
        soc_cap_reached=bool(np.any(lim == 3)),
        peak_required_brake_force_n=float(df.f_req.max()),
    )
    return dict(df=df, summary=summary)
