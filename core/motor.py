"""Motor torque/power envelope and efficiency map."""
import numpy as np


def base_speed_rpm(t_max, p_max_w):
    """Speed where the constant-torque region ends and constant-power begins."""
    return p_max_w / t_max * 60.0 / (2.0 * np.pi)


def regen_fade(rpm, fade_rpm):
    """Regen capability collapses near standstill (back-EMF too low)."""
    return float(np.clip(rpm / fade_rpm, 0.0, 1.0)) if fade_rpm > 0 else 1.0


def available_torque(rpm, t_max, p_max_w, fade_rpm):
    """Max regenerative torque at motor speed `rpm` (Nm)."""
    if rpm <= 0:
        return 0.0
    omega = rpm * 2.0 * np.pi / 60.0
    return min(t_max, p_max_w / omega) * regen_fade(rpm, fade_rpm)


def efficiency(rpm, torque, motor, constant=False):
    """Motor efficiency. Constant = rated value; otherwise a parametric map
    peaking at ~70 % rated torque / 45 % max speed:
        eta = eta_rated - c_load*(l-0.7)^2 - c_speed*(s-0.45)^2
    """
    eta0 = motor["efficiency"]
    if constant:
        return eta0
    load = torque / motor["max_torque_nm"]
    speed = rpm / motor["max_rpm"]
    eta = eta0 - motor["c_load"] * (load - 0.7) ** 2 - motor["c_speed"] * (speed - 0.45) ** 2
    return float(np.clip(eta, 0.5, eta0))
