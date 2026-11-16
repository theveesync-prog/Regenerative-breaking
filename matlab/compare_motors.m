function T = compare_motors(P, motors, autoGear, topSpeedKmh)
%COMPARE_MOTORS Run all five motors under identical vehicle/battery conditions.
%   T = compare_motors(P, motors, autoGear, topSpeedKmh)
if nargin < 3, autoGear = true; end
if nargin < 4, topSpeedKmh = 160; end
names = fieldnames(motors); rows = cell(numel(names), 1); figure('Name','Motor comparison');
for k = 1:numel(names)
    Pk = P; Pk.motor = motors.(names{k});
    if autoGear, Pk.gear_ratio = autoGearRatio(Pk.motor.max_rpm, Pk.wheel_radius, topSpeedKmh); end
    R = regenSim(Pk); S = R.summary;
    rows{k} = table(string(names{k}), Pk.gear_ratio, S.recovered_kwh*1000, S.recovery_pct, S.final_soc, ...
        S.peak_motor_power_kw, S.peak_torque_nm, S.peak_rpm, 'VariableNames', ...
        {'Motor','GearRatio','Recovered_Wh','Recovery_pct','FinalSOC','PeakPower_kW','PeakTorque_Nm','PeakRPM'});
    subplot(1,2,1); hold on; plot(R.t, R.p_elec/1000, 'DisplayName', names{k});
    subplot(1,2,2); hold on; plot(R.t, R.soc, 'DisplayName', names{k});
end
subplot(1,2,1); xlabel('Time (s)'); ylabel('Battery charging power (kW)'); legend; grid on;
subplot(1,2,2); xlabel('Time (s)'); ylabel('SOC (%)'); legend; grid on;
T = vertcat(rows{:}); disp(T);
end
