"""EV Regenerative Braking Simulator - Streamlit dashboard (Python twin of the MATLAB/Simulink model)."""
import io
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from plotly.subplots import make_subplots

from core import formulas as fm
from core import vehicle as veh
from core.data import load_cars, load_motors
from core.simulate import default_params, simulate

st.set_page_config(page_title="EV Regenerative Braking Simulator", layout="wide")
MOTORS, CARS = load_motors(), load_cars()
CAR_BY_NAME = {c["name"]: c for c in CARS}
CUSTOM = "Custom vehicle"
COLORS = {"PMSM": "#2563eb", "BLDC": "#16a34a", "ACIM": "#d97706", "SRM": "#9333ea", "DC Series": "#dc2626"}

# ----------------------------------------------------------------- sidebar
sb = st.sidebar
sb.title("Inputs")
car_name = sb.selectbox("Car preset", [CUSTOM] + list(CAR_BY_NAME),
                        help="Loads mass, wheel radius, Cd, frontal area, battery and motor ratings. Every value stays editable.")
car = CAR_BY_NAME.get(car_name)
if car:
    q = {"speculative": "SPECULATIVE", "low": "low", "medium": "medium"}[car["confidence"]]
    sb.caption(f"Data confidence: **{q}** - {car['note']}  [source]({car['source']})")
d = default_params()


def cv(key, fallback):
    return car[key] if car else fallback


def num(label, val, lo, hi, step, key, **kw):
    # key includes car so values reset when a preset is chosen
    return st.number_input(label, lo, hi, float(val), step, key=f"{key}|{car_name}", **kw)


with sb.expander("Vehicle", expanded=True):
    mass = num("Vehicle mass (kg)", cv("mass_kg", 1500), 300.0, 4000.0, 10.0, "mass", help=fm.plain("f_req"))
    v0_kmh = num("Initial speed (km/h)", 60.0, 5.0, 250.0, 1.0, "v0", help=fm.plain("v0"))
    mode = st.radio("Braking input", ["Deceleration", "Stopping distance"], horizontal=True,
                            help="Pick a target deceleration, or a target stopping distance (a = v0^2 / 2d).")
    if mode == "Deceleration":
        decel = st.slider("Braking deceleration (m/s²)", 0.5, 9.0, 3.0, 0.1, help=fm.plain("t_stop"))
        stop_dist = d["stop_distance"]
    else:
        stop_dist = st.slider("Target stopping distance (m)", 5.0, 200.0, 40.0, 1.0, help=fm.plain("decel_dist"))
        decel = d["decel"]
    wheel_r = num("Wheel radius (m)", cv("wheel_radius_m", 0.32), 0.15, 0.6, 0.005, "r", format="%.3f", help=fm.plain("omega_w"))
    crr = num("Rolling resistance coeff.", 0.010, 0.002, 0.05, 0.001, "crr", format="%.3f", help=fm.plain("f_roll"))
    cd = num("Aerodynamic drag coeff. Cd", cv("drag_coefficient", 0.30), 0.15, 0.8, 0.01, "cd", help=fm.plain("f_drag"))
    area = num("Frontal area (m²)", cv("frontal_area_m2", 2.2), 1.0, 5.0, 0.05, "area")
    grade = st.slider("Road grade (°, + uphill)", -10.0, 10.0, 0.0, 0.5)
    mu = st.slider("Tyre-road friction μ", 0.2, 1.1, 0.8, 0.05, help="Grip check: braking deceleration above μ·g is physically impossible.")

with sb.expander("Motor", expanded=True):
    mtype = st.selectbox("Motor technology", list(MOTORS),
                         index=list(MOTORS).index(car["motor_type"]) if car else 0, key=f"mt|{car_name}")
    base = MOTORS[mtype]
    use_car_motor = st.checkbox("Use car's stock torque/power ratings", value=bool(car), disabled=not car,
                                help="Off = use the illustrative library ratings of the selected motor technology.")
    tmax_def = car["motor_max_torque_nm"] if (car and use_car_motor) else base["max_torque_nm"]
    pmax_def = car["motor_max_power_kw"] if (car and use_car_motor) else base["max_power_kw"]
    tmax = st.number_input("Max torque (Nm)", 20.0, 1500.0, float(tmax_def), 5.0, key=f"tm|{car_name}|{mtype}|{use_car_motor}", help=fm.plain("t_env"))
    pmax = st.number_input("Max power (kW)", 5.0, 600.0, float(pmax_def), 1.0, key=f"pm|{car_name}|{mtype}|{use_car_motor}")
    eff = st.number_input("Rated efficiency (%)", 50.0, 99.0, base["efficiency"] * 100, 0.5, key=f"ef|{mtype}", help=fm.plain("eta_m")) / 100
    const_eff = st.checkbox("Constant efficiency (no efficiency map)", value=False)
    motor = dict(base, max_torque_nm=tmax, max_power_kw=pmax, efficiency=eff)
    st.caption(f"**{base['name']}** - {base['note']}")

with sb.expander("Drivetrain", expanded=True):
    top_speed = st.number_input("Top speed (km/h) - for auto gear ratio", 60.0, 350.0, float(cv("top_speed_kmh", 160)), 5.0, key=f"vt|{car_name}")
    auto_g = veh.auto_gear_ratio(motor["max_rpm"], wheel_r, top_speed)
    g_mode = st.radio("Gear ratio", ["Auto", "Manual"], horizontal=True, help=fm.plain("gear"))
    if g_mode == "Auto":
        gear = auto_g
        st.info(f"Auto gear ratio = **{gear:.2f}** (motor at {motor['max_rpm']:,} rpm at {top_speed:.0f} km/h)")
    else:
        g_init = car["gear_ratio"] if (car and car["gear_ratio"]) else auto_g
        gear = st.number_input("Gear ratio", 1.0, 30.0, float(round(g_init, 2)), 0.05, key=f"g|{car_name}")
        st.caption(f"Recommended for this motor/car: {auto_g:.2f}")
    dt_eff = st.slider("Drivetrain efficiency (%)", 70.0, 100.0, 95.0, 0.5) / 100
    regen_share = st.slider("Max share of braking on regen axle (%)", 10, 100,
                            {"AWD": 100, "FWD": 70, "RWD": 60}.get(car["drive_layout"], 100) if car else 100,
                            help="Axle limit: only the driven axle(s) can regenerate. 100 % = all-wheel regen.") / 100

with sb.expander("Battery", expanded=True):
    batt = num("Battery capacity (kWh)", cv("battery_kwh", 40.0), 2.0, 200.0, 1.0, "bat")
    soc0 = st.slider("Initial SOC (%)", 0, 100, 50)
    soc_max = st.slider("Maximum SOC (%)", 50, 100, 95)
    max_chg = num("Max charging power (kW)", cv("max_charge_kw", 60.0), 1.0, 400.0, 1.0, "chg", help=fm.plain("p_bat"))
    chg_eff = st.slider("Charge efficiency (%)", 85.0, 100.0, 98.0, 0.5) / 100

P = default_params()
P.update(mass=mass, v0_kmh=v0_kmh, mode="distance" if mode == "Stopping distance" else "decel", decel=decel,
         stop_distance=stop_dist, wheel_radius=wheel_r, crr=crr, cd=cd, area=area, grade_deg=grade, mu=mu,
         gear_ratio=gear, drivetrain_eff=dt_eff, regen_share=regen_share, battery_kwh=batt, soc0=float(soc0),
         soc_max=float(soc_max), max_charge_kw=max_chg, charge_eff=chg_eff, const_eff=const_eff)

res = simulate(P, motor)
S, df = res["summary"], res["df"]


def ctx():
    c = dict(S, **P)
    c.update(top_speed_ms=top_speed / 3.6, auto_gear=auto_g, max_rpm=motor["max_rpm"],
             f_roll=veh.rolling_force(mass, crr, np.radians(grade)),
             f_drag0=veh.drag_force(cd, area, S["v0_ms"]),
             f_res0=float(df.f_res.iloc[0]), stop_distance=S["stop_distance_m"], decel=S["decel"])
    return c


# ---------------------------------------------------------------- helpers
def line_fig(rows, title_y=None, height=300):
    fig = go.Figure()
    for name, y, color, dash in rows:
        fig.add_trace(go.Scatter(x=df.t, y=y, name=name, mode="lines", line=dict(color=color, dash=dash, width=2),
                                 hovertemplate=f"{name}: %{{y:.2f}}<br>t=%{{x:.2f}} s<extra></extra>"))
    fig.update_layout(height=height, margin=dict(l=10, r=10, t=30, b=10), hovermode="x unified",
                      xaxis_title="Time (s)", yaxis_title=title_y, legend=dict(orientation="h", y=1.15))
    return fig


def metric(col, label, value, key, delta=None):
    col.metric(label, value, delta, help=fm.plain(key))


# ------------------------------------------------------------------- tabs
st.title("EV Regenerative Braking Simulator")
st.caption(f"{car_name} · {mtype} · {P['v0_kmh']:.0f} km/h → 0 · illustrative model, not a certified vehicle simulation")
tab1, tab2, tab3, tab4, tab5 = st.tabs(["Single motor", "Compare 5 motors", "Sweeps", "Formulas", "Car presets"])

with tab1:
    if S["grip_exceeded"]:
        st.error(f"Requested deceleration {S['decel']:.2f} m/s² exceeds the grip limit μ·g = {S['grip_limit_decel']:.2f} m/s². The tyres would lock/slip.")
    if S["soc_cap_reached"]:
        st.warning("Battery reached its maximum SOC: regeneration was cut and friction brakes took over.")
    r1 = st.columns(4)
    metric(r1[0], "Recovered electrical energy", f"{S['recovered_kwh']*1000:.1f} Wh", "e_rec")
    metric(r1[1], "Energy recovery", f"{S['recovery_pct']:.1f} %", "recovery")
    metric(r1[2], "Final battery SOC", f"{S['final_soc']:.3f} %", "soc", f"{S['delta_soc']:+.3f} %")
    metric(r1[3], "Peak motor power", f"{S['peak_motor_power_kw']:.1f} kW", "p_mech")
    r2 = st.columns(4)
    metric(r2[0], "Stopping time", f"{S['stop_time_s']:.2f} s", "t_stop")
    metric(r2[1], "Stopping distance", f"{S['stop_distance_m']:.1f} m", "s_stop")
    metric(r2[2], "Peak motor torque", f"{S['peak_torque_nm']:.0f} Nm", "torque")
    metric(r2[3], "Peak motor RPM", f"{S['peak_rpm']:,.0f}", "omega_m")

    g1, g2 = st.columns(2)
    g1.subheader("Vehicle speed vs time")
    g1.plotly_chart(line_fig([("Speed (km/h)", df.v_kmh, "#2563eb", "solid")], "km/h"), width='stretch')
    g2.subheader("Motor torque vs time")
    g2.plotly_chart(line_fig([("Regen torque (Nm)", df.torque, "#16a34a", "solid"),
                              ("Limit (Nm)", [min(tmax, 1e9)] * len(df), "#9ca3af", "dot")], "Nm"), width='stretch')
    g3, g4 = st.columns(2)
    g3.subheader("Motor speed vs time")
    g3.plotly_chart(line_fig([("Motor speed (rpm)", df.rpm, "#d97706", "solid")], "rpm"), width='stretch')
    g4.subheader("Motor mechanical & battery charging power")
    g4.plotly_chart(line_fig([("Motor mechanical power (kW)", df.p_mech_motor / 1000, "#2563eb", "solid"),
                              ("Battery charging power (kW)", df.p_elec / 1000, "#16a34a", "solid"),
                              ("Friction braking power (kW)", df.p_fric / 1000, "#dc2626", "dash")], "kW"), width='stretch')
    g5, g6 = st.columns(2)
    g5.subheader("Braking forces: required, regen, friction")
    g5.plotly_chart(line_fig([("Required (N)", df.f_req, "#111827", "solid"), ("Regenerative (N)", df.f_regen, "#16a34a", "solid"),
                              ("Friction (N)", df.f_fric, "#dc2626", "dash"), ("Road resistance (N)", df.f_res, "#9ca3af", "dot")], "N"), width='stretch')
    g6.subheader("Battery SOC vs time")
    g6.plotly_chart(line_fig([("SOC (%)", df.soc, "#9333ea", "solid")], "%"), width='stretch')

    st.subheader("Energy analysis")
    e = pd.DataFrame({"Item": ["Initial kinetic energy", "Road resistance (rolling+drag)", "Total braking energy (brakes)", "Mechanical regenerative energy (motor shaft)",
                               "Friction braking energy", "Drivetrain loss", "Motor loss", "Recovered electrical energy", "Battery charge loss (inside pack)"],
                      "kWh": [S["ke_kwh"], S["road_resistance_kwh"], S["total_braking_kwh"], S["mech_regen_kwh"], S["friction_kwh"],
                              S["drivetrain_loss_kwh"], S["motor_loss_kwh"], S["recovered_kwh"], S["battery_loss_kwh"]]})
    c1, c2 = st.columns([1, 1])
    c1.dataframe(e.style.format({"kWh": "{:.5f}"}), hide_index=True, width='stretch')
    parts = {"Recovered (electrical)": S["recovered_kwh"], "Motor loss": S["motor_loss_kwh"], "Drivetrain loss": S["drivetrain_loss_kwh"],
             "Friction brakes": S["friction_kwh"], "Road resistance": S["road_resistance_kwh"]}
    pie = go.Figure(go.Pie(labels=list(parts), values=list(parts.values()), hole=0.5,
                           marker=dict(colors=["#16a34a", "#d97706", "#9333ea", "#dc2626", "#9ca3af"]), sort=False))
    pie.update_layout(height=300, margin=dict(l=0, r=0, t=10, b=0))
    c2.plotly_chart(pie, width='stretch')
    st.caption(f"Energy-balance check: residual {S['balance_residual_pct']:.4f} % of kinetic energy · "
               f"torque/power-limited {S['pct_time_torque_limited']:.0f} % of the stop · battery-limited {S['pct_time_battery_limited']:.0f} %")
    st.download_button("Download time-series CSV", df.to_csv(index=False), "regen_simulation.csv", "text/csv")

with tab2:
    st.markdown("All five motors run under **identical vehicle, drivetrain and battery settings** from the sidebar, using the library ratings "
                "(torque/power/efficiency) from the brief. The gear ratio follows the sidebar mode (Auto = recomputed per motor for its max RPM).")
    rows, dfs = [], {}
    for k, m in MOTORS.items():
        pk = dict(P, const_eff=const_eff)
        if g_mode == "Auto":
            pk["gear_ratio"] = veh.auto_gear_ratio(m["max_rpm"], wheel_r, top_speed)
        rr = simulate(pk, m)
        dfs[k] = rr["df"]
        s = rr["summary"]
        rows.append({"Motor": k, "Gear ratio": pk["gear_ratio"], "Recovered (Wh)": s["recovered_kwh"] * 1000, "Recovery %": s["recovery_pct"],
                     "Final SOC %": s["final_soc"], "Peak motor power (kW)": s["peak_motor_power_kw"], "Peak torque (Nm)": s["peak_torque_nm"],
                     "Peak RPM": s["peak_rpm"], "Friction energy (Wh)": s["friction_kwh"] * 1000, "Conversion loss (Wh)": s["conversion_loss_kwh"] * 1000,
                     "Avg motor eff %": s["avg_motor_eff"] * 100})
    cmp = pd.DataFrame(rows)
    best = cmp.loc[cmp["Recovery %"].idxmax(), "Motor"]
    st.success(f"Highest energy recovery under these conditions: **{best}** ({cmp['Recovery %'].max():.1f} %)")
    st.dataframe(cmp.style.format({"Gear ratio": "{:.2f}", "Recovered (Wh)": "{:.1f}", "Recovery %": "{:.1f}", "Final SOC %": "{:.3f}", "Peak motor power (kW)": "{:.1f}",
                                   "Peak torque (Nm)": "{:.0f}", "Peak RPM": "{:,.0f}", "Friction energy (Wh)": "{:.1f}", "Conversion loss (Wh)": "{:.1f}",
                                   "Avg motor eff %": "{:.1f}"}).highlight_max(subset=["Recovery %"], color="#bbf7d0"), hide_index=True, width='stretch')
    b1, b2 = st.columns(2)
    bar = go.Figure(go.Bar(x=cmp["Motor"], y=cmp["Recovery %"], marker_color=[COLORS[m] for m in cmp["Motor"]],
                           hovertemplate="%{x}: %{y:.1f} %<extra></extra>"))
    bar.update_layout(height=320, title="Energy recovery %", margin=dict(l=10, r=10, t=40, b=10), yaxis_title="% of kinetic energy")
    b1.plotly_chart(bar, width='stretch')
    pf = go.Figure()
    for k, dd in dfs.items():
        pf.add_trace(go.Scatter(x=dd.t, y=dd.p_elec / 1000, name=k, line=dict(color=COLORS[k])))
    pf.update_layout(height=320, title="Battery charging power", hovermode="x unified", xaxis_title="Time (s)", yaxis_title="kW", margin=dict(l=10, r=10, t=40, b=10))
    b2.plotly_chart(pf, width='stretch')
    sf = go.Figure()
    for k, dd in dfs.items():
        sf.add_trace(go.Scatter(x=dd.t, y=dd.soc, name=k, line=dict(color=COLORS[k])))
    sf.update_layout(height=300, title="Battery SOC", hovermode="x unified", xaxis_title="Time (s)", yaxis_title="%", margin=dict(l=10, r=10, t=40, b=10))
    st.plotly_chart(sf, width='stretch')
    st.download_button("Download comparison CSV", cmp.to_csv(index=False), "motor_comparison.csv", "text/csv")

with tab3:
    st.markdown("Sensitivity of **energy recovery %** to one variable while everything else stays as in the sidebar.")
    sweep = st.selectbox("Sweep variable", ["Initial speed (km/h)", "Deceleration (m/s²)", "Vehicle mass (kg)", "Initial SOC (%)", "Max charging power (kW)"])
    rng = {"Initial speed (km/h)": ("v0_kmh", np.linspace(20, 160, 15)), "Deceleration (m/s²)": ("decel", np.linspace(0.5, min(8, mu * 9.81), 15)),
           "Vehicle mass (kg)": ("mass", np.linspace(800, 3000, 15)), "Initial SOC (%)": ("soc0", np.linspace(5, soc_max, 15)),
           "Max charging power (kW)": ("max_charge_kw", np.linspace(5, 200, 15))}
    key, xs = rng[sweep]
    fig = go.Figure()
    for k, m in MOTORS.items():
        ys = []
        for x in xs:
            pk = dict(P, **{key: float(x), "mode": "decel" if key == "decel" else P["mode"]})
            if g_mode == "Auto":
                pk["gear_ratio"] = veh.auto_gear_ratio(m["max_rpm"], wheel_r, top_speed)
            pk["steps"] = 400
            ys.append(simulate(pk, m)["summary"]["recovery_pct"])
        fig.add_trace(go.Scatter(x=xs, y=ys, name=k, mode="lines+markers", line=dict(color=COLORS[k])))
    fig.update_layout(height=420, hovermode="x unified", xaxis_title=sweep, yaxis_title="Energy recovery (%)", margin=dict(l=10, r=10, t=20, b=10))
    st.plotly_chart(fig, width='stretch')

with tab4:
    st.markdown("Every formula with **your current numbers substituted**. The same formulas appear as hover tooltips (ⓘ) on the inputs and results.")
    c = ctx()
    groups = {}
    for k, f in fm.F.items():
        groups.setdefault(f["group"], []).append(k)
    for gname, keys in groups.items():
        st.subheader(gname)
        for k in keys:
            f = fm.F[k]
            with st.expander(f["title"], expanded=False):
                st.latex(f["latex"])
                if f["subst"]:
                    st.latex(f["subst"](c))
                st.caption(f["plain"])

with tab5:
    st.markdown("Presets are **illustrative**: public listings disagree, so check the *confidence* column and the source link before relying on a number.")
    tbl = pd.DataFrame(CARS)[["name", "drive_layout", "mass_kg", "tyre", "wheel_radius_m", "drag_coefficient", "frontal_area_m2", "battery_kwh",
                              "motor_max_power_kw", "motor_max_torque_nm", "motor_type", "top_speed_kmh", "confidence", "source"]]
    st.dataframe(tbl, hide_index=True, width='stretch', column_config={"source": st.column_config.LinkColumn("source")})
