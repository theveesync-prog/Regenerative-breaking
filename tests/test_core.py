import math
import pytest
from core.simulate import simulate, default_params
from core.data import load_motors, load_cars
from core import vehicle as veh, motor as mot, battery as bat

MOTORS = load_motors()


@pytest.mark.parametrize("name", list(MOTORS))
def test_energy_balance_closes(name):
    s = simulate(default_params(), MOTORS[name])["summary"]
    assert s["balance_residual_pct"] < 0.5


@pytest.mark.parametrize("name", list(MOTORS))
def test_recovered_never_exceeds_ke_and_limits_hold(name):
    p = default_params()
    r = simulate(p, MOTORS[name])
    s, df = r["summary"], r["df"]
    assert 0 <= s["recovered_kwh"] <= s["ke_kwh"]
    assert df.p_mech_motor.max() <= MOTORS[name]["max_power_kw"] * 1000 * 1.001
    assert df.torque.max() <= MOTORS[name]["max_torque_nm"] * 1.001
    assert df.p_elec.max() <= p["max_charge_kw"] * 1000 * 1.001
    assert df.soc.max() <= p["soc_max"] + 1e-9


def test_kinematics_hand_calc():
    p = default_params()
    p.update(mass=1500, v0_kmh=60, decel=3.0)
    s = simulate(p, MOTORS["PMSM"])["summary"]
    assert s["stop_time_s"] == pytest.approx(16.6667 / 3.0, rel=1e-3)
    assert s["stop_distance_m"] == pytest.approx(16.6667**2 / 6.0, rel=1e-3)
    assert s["ke_kwh"] == pytest.approx(0.0579, abs=1e-4)


def test_distance_mode_sets_decel():
    p = default_params()
    p.update(mode="distance", stop_distance=30.0, v0_kmh=54.0)
    s = simulate(p, MOTORS["PMSM"])["summary"]
    assert s["stop_distance_m"] == pytest.approx(30.0, rel=1e-6)


def test_soc_ceiling_blocks_regen():
    p = default_params()
    p.update(soc0=95.0, soc_max=95.0)
    s = simulate(p, MOTORS["PMSM"])["summary"]
    assert s["recovered_kwh"] == 0 and s["soc_cap_reached"]


def test_battery_power_limit_reduces_recovery():
    hi, lo = default_params(), default_params()
    lo["max_charge_kw"] = 10.0
    a = simulate(hi, MOTORS["PMSM"])["summary"]["recovered_kwh"]
    b = simulate(lo, MOTORS["PMSM"])["summary"]["recovered_kwh"]
    assert b < a


def test_auto_gear_puts_motor_at_max_rpm_at_top_speed():
    g = veh.auto_gear_ratio(12000, 0.33, 160)
    rpm = (160 / 3.6) / 0.33 * g * 60 / (2 * math.pi)
    assert rpm == pytest.approx(12000, rel=1e-6)


def test_motor_envelope():
    t_max, p_max = 180, 60000
    nb = mot.base_speed_rpm(t_max, p_max)
    assert mot.available_torque(nb * 0.5, t_max, p_max, 0) == pytest.approx(t_max)
    assert mot.available_torque(nb * 2, t_max, p_max, 0) == pytest.approx(t_max / 2)
    assert mot.available_torque(50, t_max, p_max, 200) < mot.available_torque(500, t_max, p_max, 200)


def test_battery_taper():
    assert bat.max_charge_power(95, 95, 60) == 0
    assert bat.max_charge_power(50, 95, 60) == 60000
    assert 0 < bat.max_charge_power(90, 95, 60) < 60000


def test_car_presets_load_and_simulate():
    for c in load_cars():
        p = default_params()
        p.update(mass=c["mass_kg"], wheel_radius=c["wheel_radius_m"], cd=c["drag_coefficient"],
                 area=c["frontal_area_m2"], battery_kwh=c["battery_kwh"], max_charge_kw=c["max_charge_kw"],
                 gear_ratio=veh.auto_gear_ratio(12000, c["wheel_radius_m"], c["top_speed_kmh"]))
        m = dict(MOTORS[c["motor_type"]], max_torque_nm=c["motor_max_torque_nm"], max_power_kw=c["motor_max_power_kw"])
        assert simulate(p, m)["summary"]["balance_residual_pct"] < 0.5
