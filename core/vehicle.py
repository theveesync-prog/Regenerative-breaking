"""Vehicle dynamics and road-resistance helpers."""
import math

G_ACC = 9.81
RHO_AIR = 1.225


def kmh_to_ms(v_kmh):
    return v_kmh / 3.6


def rolling_force(mass, crr, grade_rad=0.0):
    return crr * mass * G_ACC * math.cos(grade_rad)


def drag_force(cd, area, v):
    return 0.5 * RHO_AIR * cd * area * v * v


def grade_force(mass, grade_rad):
    """Gravity component along the road (positive uphill, i.e. resists motion)."""
    return mass * G_ACC * math.sin(grade_rad)


def decel_from_distance(v0, distance):
    """Constant deceleration needed to stop from v0 in `distance` metres."""
    return v0 * v0 / (2.0 * distance)


def auto_gear_ratio(max_motor_rpm, wheel_radius, top_speed_kmh, margin=1.0):
    """Gear ratio that puts the motor at max RPM at top speed.

    G = (omega_motor_max * r_wheel) / v_top   (margin < 1 keeps head-room)
    """
    omega_max = max_motor_rpm * 2.0 * math.pi / 60.0 * margin
    return omega_max * wheel_radius / kmh_to_ms(top_speed_kmh)
