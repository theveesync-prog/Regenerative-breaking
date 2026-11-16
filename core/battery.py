"""Battery charge-acceptance limits."""


def max_charge_power(soc, soc_max, p_max_kw, taper_width=10.0):
    """Charge-power limit (W). Tapers linearly to zero over the last
    `taper_width` SOC points below the SOC ceiling; zero at/above the ceiling."""
    if soc >= soc_max:
        return 0.0
    scale = min(1.0, (soc_max - soc) / taper_width)
    return p_max_kw * 1000.0 * scale
