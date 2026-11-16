function mdl = build_regen_model(P)
%BUILD_REGEN_MODEL Programmatically build regen_braking.slx (needs Simulink).
%   build_regen_model(P) with P from defaultParams(motor) (+ vehicle/battery edits).
%   The struct is placed in the base workspace as P (with P.v0_ms) and the model is saved here.
%   Signals logged to 'simout_regen' (columns): t is the time vector;
%   [v s soc E_elec_J E_fric_J f_res f_req f_regen f_fric torque rpm p_mech p_elec]
mdl = 'regen_braking';
v0 = P.v0_kmh/3.6;
if strcmp(P.mode,'distance'), P.decel = v0^2/(2*P.stop_distance); end
P.v0_ms = v0;
assignin('base','P',P);
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);
set_param(mdl,'SolverType','Variable-step','Solver','ode45','MaxStep','0.01','StopTime','120');

add = @(src,name,pos,varargin) add_block(src,[mdl '/' name],'Position',pos,varargin{:});
add('simulink/Sources/Constant','NegDecel',[30 40 90 70],'Value','-P.decel');
add('simulink/Continuous/Integrator','Int_v',[130 40 180 70],'InitialCondition','P.v0_ms', ...
    'LimitOutput','on','UpperSaturationLimit','inf','LowerSaturationLimit','0');
add('simulink/Continuous/Integrator','Int_s',[230 120 280 150],'InitialCondition','0');
add('simulink/Signal Routing/Mux','Mux_in',[330 40 335 100],'Inputs','2');
add('simulink/User-Defined Functions/Interpreted MATLAB Function','Regen',[380 50 480 90], ...
    'MATLABFcn','regenStepWrap','OutputDimensions','9');
add('simulink/Signal Routing/Demux','Dmx',[520 30 525 190],'Outputs','9');
add('simulink/Continuous/Integrator','Int_soc',[600 200 650 230],'InitialCondition','P.soc0', ...
    'LimitOutput','on','UpperSaturationLimit','P.soc_max','LowerSaturationLimit','0');
add('simulink/Continuous/Integrator','Int_Eelec',[600 260 650 290],'InitialCondition','0');
add('simulink/Math Operations/Product','Prod_fric',[560 320 590 350]);   % f_fric * v
add('simulink/Continuous/Integrator','Int_Efric',[640 320 690 350],'InitialCondition','0');
add('simulink/Signal Routing/Mux','Mux_log',[760 60 765 360],'Inputs','13');
add('simulink/Sinks/To Workspace','ToWs',[820 190 900 230],'VariableName','simout_regen','SaveFormat','Array');
add('simulink/Logic and Bit Operations/Compare To Constant','AtRest',[230 -10 290 20],'const','0.01','relop','<=');
add('simulink/Sinks/Stop Simulation','Stop',[340 -10 380 20]);

ln = @(a,b) add_line(mdl,a,b,'autorouting','on');
ln('NegDecel/1','Int_v/1');
ln('Int_v/1','Int_s/1');
ln('Int_v/1','Mux_in/1');
ln('Int_soc/1','Mux_in/2');
ln('Mux_in/1','Regen/1');
ln('Regen/1','Dmx/1');
ln('Dmx/9','Int_soc/1');
ln('Dmx/8','Int_Eelec/1');
ln('Dmx/4','Prod_fric/1'); ln('Int_v/1','Prod_fric/2'); ln('Prod_fric/1','Int_Efric/1');
ln('Int_v/1','AtRest/1'); ln('AtRest/1','Stop/1');
% log: v s soc Eelec Efric f_res f_req f_regen f_fric torque rpm p_mech p_elec
ln('Int_v/1','Mux_log/1'); ln('Int_s/1','Mux_log/2'); ln('Int_soc/1','Mux_log/3');
ln('Int_Eelec/1','Mux_log/4'); ln('Int_Efric/1','Mux_log/5');
for k = 1:8, ln(sprintf('Dmx/%d',k), sprintf('Mux_log/%d',5+k)); end
ln('Mux_log/1','ToWs/1');
save_system(mdl, fullfile(fileparts(mfilename('fullpath')), mdl));
fprintf('Built %s.slx. Run: out = sim(''%s''); y = out.simout_regen; t = out.tout;\n', mdl, mdl);
end
