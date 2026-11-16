# EV Regenerative Braking Simulator

Interactive simulator for regenerative braking: pick a car (Tata Harrier/Nexon/Safari-expected, Hyundai Ioniq 5, Mahindra BE 6, Tesla Model 3, ...), tweak the vehicle/drivetrain/battery, choose one of five motor technologies (PMSM, BLDC, ACIM, SRM, DC Series) and a deceleration or stopping distance. The gear ratio can be calculated automatically, and every result has a formula tooltip.

Two implementations share the same equations and data (`data/cars.json`, `data/motors.json`):

| | Path | Status |
|---|---|---|
| **MATLAB / Simulink** (primary) | `matlab/` | Written but **not executed** (no MATLAB where it was authored). Run it and report errors. |
| **Python / Streamlit** (localhost preview + reference) | `app.py`, `core/` | Tested: `pytest` (18 tests) and verified in a browser. |

## Python preview
```
pip install -r requirements.txt
streamlit run app.py        # http://localhost:8501
pytest
```
Tabs: Single motor (8 KPIs, 6 interactive plots, energy table) | Compare 5 motors | Sweeps | Formulas (live-substituted) | Car presets.

## MATLAB / Simulink (R2020b+; Simulink for the .slx)
```matlab
cd matlab
RegenApp                                  % interactive app (pure-MATLAB engine, instant updates)
[cars, motors] = loadPreset();
P = defaultParams(motors.PMSM);
T = compare_motors(P, motors, true, 160); % 5-motor comparison
build_regen_model(P);                     % builds regen_braking.slx
out = sim('regen_braking');               % y = out.simout_regen
runtests('tests')                         % includes a check against the Python reference (48.5 Wh / 83.9 %)
```

## Model summary
`F_req = m*a - F_res`; `F_regen = min(axle share*F_req, T_avail*G/(eta_dt*r), P_chg/(eta_m*eta_dt*v))`; torque/power envelope (constant torque, then constant power, fade near 0 rpm); parametric efficiency map; SOC-tapered charge limit with SOC ceiling; energy balance `KE = road + friction + recovered + drivetrain loss + motor loss` is checked on every run.

## Data caveat
Car presets are illustrative. Public listings disagree (e.g. Tata Safari EV is not launched; Harrier EV, Punch EV, BE 6 figures conflict). Each preset has a `confidence` and a `source`; verify against official brochures. The 5-motor figures are the illustrative values from the project brief; `max_rpm`, fade RPM and efficiency-map coefficients are modelling assumptions.
