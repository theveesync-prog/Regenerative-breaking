function tests = testRegen
%TESTREGEN matlab.unittest function tests. Run: runtests('tests')
tests = functiontests(localfunctions);
end

function P = baseline()
[~, motors] = loadPreset();
P = defaultParams(motors.PMSM);
end

function testEnergyBalance(t)
R = regenSim(baseline());
verifyLessThan(t, R.summary.balance_residual_pct, 0.5);
end

function testRecoveryBelowKineticEnergy(t)
S = regenSim(baseline()).summary;
verifyLessThanOrEqual(t, S.recovered_kwh, S.ke_kwh);
verifyGreaterThan(t, S.recovered_kwh, 0);
end

function testMatchesPythonReference(t)
% Python (core/simulate.py) with defaults + PMSM gives recovered = 48.5 Wh, recovery = 83.9 %
S = regenSim(baseline()).summary;
verifyEqual(t, S.recovered_kwh*1000, 48.5, 'RelTol', 0.02);
verifyEqual(t, S.recovery_pct, 83.9, 'AbsTol', 1.0);
end

function testSocCeilingBlocksRegen(t)
P = baseline(); P.soc0 = 95; P.soc_max = 95;
verifyEqual(t, regenSim(P).summary.recovered_kwh, 0);
end

function testAutoGear(t)
G = autoGearRatio(12000, 0.33, 160);
rpm = (160/3.6)/0.33*G*60/(2*pi);
verifyEqual(t, rpm, 12000, 'RelTol', 1e-9);
end
