function R = regenSim(P)
%REGENSIM Pure-MATLAB braking simulation (fast; drives the app). Mirrors core/simulate.py.
%   R.t, R.v, R.s, R.soc, R.f_*, R.torque, R.rpm, R.p_mech, R.p_elec, R.p_fric, R.summary
v0 = P.v0_kmh/3.6;
if strcmp(P.mode,'distance'), a = v0^2/(2*P.stop_distance); else, a = P.decel; end
P.decel = a;
tStop = v0/a; t = linspace(0, tStop, P.steps+1); dt = t(2)-t(1);
v = max(v0 - a*t, 0); s = v0*t - 0.5*a*t.^2;
n = numel(t); Y = zeros(n,9); soc = zeros(1,n); s0 = P.soc0;
for i = 1:n
    Y(i,:) = regenCore(v(i), s0, P);
    soc(i) = s0;
    s0 = min(s0 + Y(i,9)*dt, P.soc_max);
end
R.t = t; R.v = v; R.v_kmh = v*3.6; R.s = s; R.soc = soc;
R.f_res = Y(:,1)'; R.f_req = Y(:,2)'; R.f_regen = Y(:,3)'; R.f_fric = Y(:,4)';
R.torque = Y(:,5)'; R.rpm = Y(:,6)'; R.p_mech = Y(:,7)'; R.p_elec = Y(:,8)';
R.p_fric = R.f_fric.*v;
ke = 0.5*P.mass*v0^2; kwh = @(j) j/3.6e6;
eElec = trapz(t,R.p_elec); eMech = trapz(t,R.p_mech); eFric = trapz(t,R.p_fric);
eRes = trapz(t,R.f_res.*v); eBrake = trapz(t,R.f_req.*v);
S.decel = a; S.stop_time_s = tStop; S.stop_distance_m = v0^2/(2*a);
S.ke_kwh = kwh(ke); S.recovered_kwh = kwh(eElec); S.mech_regen_kwh = kwh(eMech);
S.friction_kwh = kwh(eFric); S.total_braking_kwh = kwh(eBrake); S.road_resistance_kwh = kwh(eRes);
S.conversion_loss_kwh = kwh(trapz(t,R.f_regen.*v) - eElec);
S.recovery_pct = 100*eElec/ke; S.final_soc = soc(end);
S.peak_motor_power_kw = max(R.p_mech)/1000; S.peak_torque_nm = max(R.torque); S.peak_rpm = max(R.rpm);
S.balance_residual_pct = 100*abs(ke - (eRes + eFric + trapz(t,R.f_regen.*v)))/ke;
S.grip_exceeded = a > P.mu*9.81;
R.summary = S;
end
