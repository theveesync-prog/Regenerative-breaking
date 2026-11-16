function out = regenCore(v, soc, P)
%REGENCORE Instantaneous regenerative-braking solution at speed v (m/s) and SOC (%).
%   Shared by regenSim.m (pure MATLAB) and the Simulink model (via regenStepWrap).
%   out = [f_res f_req f_regen f_fric torque rpm p_mech p_elec dSOCdt]
%   Mirrors core/simulate.py. Forces N, torque Nm, power W, dSOCdt in %/s.
g = 9.81; rho = 1.225;
M = P.motor; r = P.wheel_radius; G = P.gear_ratio; etaDt = P.drivetrain_eff;
th = P.grade_deg*pi/180;
fRes = P.crr*P.mass*g*cos(th) + 0.5*rho*P.cd*P.area*v^2 + P.mass*g*sin(th);
fReq = max(P.mass*P.decel - fRes, 0);
rpm  = v/r*G*60/(2*pi);
omegaM = rpm*2*pi/60;

% torque/power envelope with low-speed fade
if rpm > 0
    tAvail = min(M.max_torque_nm, M.max_power_kw*1000/omegaM) * min(1, rpm/M.regen_fade_rpm);
else
    tAvail = 0;
end
fTrq  = tAvail*G/(etaDt*r);
fAxle = fReq*P.regen_share;

% battery charge limit (linear taper over last 10 SOC points, zero at ceiling)
if soc >= P.soc_max
    pLim = 0;
else
    pLim = P.max_charge_kw*1000*min(1, (P.soc_max - soc)/10);
end

etaM = M.efficiency; fReg = 0;
for k = 1:3
    if v > 1e-6, fBat = pLim/(etaM*etaDt*v); else, fBat = 0; end
    fReg = min([fAxle, fTrq, fBat]);
    torque = fReg*r*etaDt/G;
    etaM = motorEfficiency(rpm, torque, M, P.const_eff);
end
torque = fReg*r*etaDt/G;
pMech = torque*omegaM;
pElec = pMech*etaM;
dSOC  = pElec*P.charge_eff/(P.battery_kwh*3.6e6)*100;
out = [fRes, fReq, fReg, fReq - fReg, torque, rpm, pMech, pElec, dSOC];
end

function eta = motorEfficiency(rpm, torque, M, constEff)
eta0 = M.efficiency;
if constEff, eta = eta0; return; end
l = torque/M.max_torque_nm; s = rpm/M.max_rpm;
eta = min(eta0, max(0.5, eta0 - M.c_load*(l-0.7)^2 - M.c_speed*(s-0.45)^2));
end
