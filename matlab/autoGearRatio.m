function G = autoGearRatio(maxMotorRpm, wheelRadius, topSpeedKmh, margin)
%AUTOGEARRATIO Gear ratio that puts the motor at max RPM at top speed.
%   G = (omega_motor_max * r_wheel) / v_top
if nargin < 4, margin = 1.0; end
omegaMax = maxMotorRpm * 2*pi/60 * margin;
G = omegaMax * wheelRadius / (topSpeedKmh/3.6);
end
