function y = regenStepWrap(u)
%REGENSTEPWRAP Interpreted-MATLAB-Function wrapper used by regen_braking.slx.
%   u = [v; soc].  Parameters come from the base-workspace struct P (cached).
persistent P
if isempty(P), P = evalin('base', 'P'); end
y = regenCore(u(1), u(2), P).';
end
