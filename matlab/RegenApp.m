function RegenApp()
%REGENAPP Interactive regenerative-braking simulator (MATLAB uifigure, R2020b+).
%   Pick a car preset and motor, drag the sliders; all plots and KPIs update live
%   (pure-MATLAB engine regenSim). "Run Simulink model" builds/runs regen_braking.slx
%   with the same parameters. Hover any control to see its formula.
[cars, motors] = loadPreset();
motorNames = fieldnames(motors);
fig = uifigure('Name','EV Regenerative Braking Simulator','Position',[50 50 1350 760]);
gl = uigridlayout(fig,[1 2]); gl.ColumnWidth = {330,'1x'};
left = uigridlayout(gl,[17 2]); left.RowHeight = repmat({26},1,17); left.Scrollable = 'on';
right = uigridlayout(gl,[4 3]); right.RowHeight = {70,'1x','1x','1x'};

st.P = defaultParams(motors.(motorNames{1})); st.motors = motors; st.cars = cars;
st.motorNames = motorNames; st.topSpeed = 160;

carNames = [{'Custom vehicle'}; {cars.name}'];
ddCar   = ctrl(@uidropdown,1,'Car preset',{'Items',carNames},'Loads vehicle, battery and motor ratings (illustrative data)');
ddMotor = ctrl(@uidropdown,2,'Motor',{'Items',strrep(motorNames,'_',' ')},'PMSM / BLDC / ACIM / SRM / DC Series');
eMass   = ctrl(@uieditfield,3,'Mass (kg)',{'numeric','Value',1500},'F_req = m*a - F_res');
eSpeed  = ctrl(@uieditfield,4,'Speed (km/h)',{'numeric','Value',60},'v0 = v_kmh / 3.6');
sDecel  = ctrl(@uislider,5,'Decel (m/s^2)',{'Limits',[0.5 9],'Value',3},'t_stop = v0/a;  s = v0^2/(2a);  a = v0^2/(2d)');
eRad    = ctrl(@uieditfield,6,'Wheel radius (m)',{'numeric','Value',0.32},'omega_wheel = v / r');
eCd     = ctrl(@uieditfield,7,'Drag coeff. Cd',{'numeric','Value',0.30},'F_drag = 0.5*rho*Cd*A*v^2');
eArea   = ctrl(@uieditfield,8,'Frontal area (m^2)',{'numeric','Value',2.2},'');
eCrr    = ctrl(@uieditfield,9,'Rolling coeff.',{'numeric','Value',0.010},'F_roll = Crr*m*g*cos(theta)');
cbAuto  = ctrl(@uicheckbox,10,'Auto gear ratio',{'Value',true},'G = (omega_motor_max * r) / v_top');
eGear   = ctrl(@uieditfield,11,'Gear ratio',{'numeric','Value',9},'Manual gear ratio (ignored in auto mode)');
eDt     = ctrl(@uieditfield,12,'Drivetrain eff. (%)',{'numeric','Value',95},'P_motor = F_regen * v * eta_dt');
eBatt   = ctrl(@uieditfield,13,'Battery (kWh)',{'numeric','Value',40},'SOC += P_bat*eta_c*dt/(C*3.6e6)*100');
sSoc    = ctrl(@uislider,14,'Initial SOC (%)',{'Limits',[0 100],'Value',50},'');
eSocMax = ctrl(@uieditfield,15,'Max SOC (%)',{'numeric','Value',95},'Regen cut to zero at SOC_max');
eChg    = ctrl(@uieditfield,16,'Max charge (kW)',{'numeric','Value',60},'P_bat = min(P_mech*eta_m, P_chg_max(SOC))');
btn = uibutton(left,'Text','Run Simulink model','ButtonPushedFcn',@(~,~)runSimulink());
btn.Layout.Row = 17; btn.Layout.Column = [1 2];

kpi = uilabel(right,'Text','','FontSize',14,'WordWrap','on'); kpi.Layout.Row = 1; kpi.Layout.Column = [1 3];
ax = gobjects(1,6);
for k = 1:6
    ax(k) = uiaxes(right); ax(k).Layout.Row = 2 + floor((k-1)/3); ax(k).Layout.Column = mod(k-1,3)+1;
end
fig.UserData = st;
update();

    function h = ctrl(fcn,row,labelText,args,tip)
        lb = uilabel(left,'Text',labelText); lb.Layout.Row = row; lb.Layout.Column = 1; lb.Tooltip = tip;
        h = fcn(left,args{:}); h.Layout.Row = row; h.Layout.Column = 2; h.Tooltip = tip;
        h.ValueChangedFcn = @(~,~)onChange(h);
    end

    function onChange(h)
        if h == ddCar, applyCar(); end
        update();
    end

    function applyCar()
        k = find(strcmp(carNames, ddCar.Value)) - 1;
        if k < 1, return; end
        c = cars(k);
        eMass.Value = c.mass_kg; eRad.Value = c.wheel_radius_m; eCd.Value = c.drag_coefficient;
        eArea.Value = c.frontal_area_m2; eBatt.Value = c.battery_kwh; eChg.Value = c.max_charge_kw;
        ddMotor.Value = c.motor_type; fig.UserData.topSpeed = c.top_speed_kmh;
        fig.UserData.carMotor = struct('t',c.motor_max_torque_nm,'p',c.motor_max_power_kw);
    end

    function P = collect()
        mName = strrep(ddMotor.Value,' ','_');
        M = motors.(mName);
        k = find(strcmp(carNames, ddCar.Value)) - 1;
        if k >= 1 && isfield(fig.UserData,'carMotor')
            M.max_torque_nm = fig.UserData.carMotor.t; M.max_power_kw = fig.UserData.carMotor.p;
        end
        P = defaultParams(M);
        P.mass = eMass.Value; P.v0_kmh = eSpeed.Value; P.decel = sDecel.Value; P.wheel_radius = eRad.Value;
        P.cd = eCd.Value; P.area = eArea.Value; P.crr = eCrr.Value;
        P.drivetrain_eff = eDt.Value/100; P.battery_kwh = eBatt.Value; P.soc0 = sSoc.Value;
        P.soc_max = eSocMax.Value; P.max_charge_kw = eChg.Value;
        if cbAuto.Value
            P.gear_ratio = autoGearRatio(M.max_rpm, P.wheel_radius, fig.UserData.topSpeed);
            eGear.Value = P.gear_ratio;
        else
            P.gear_ratio = eGear.Value;
        end
    end

    function update()
        P = collect(); R = regenSim(P); S = R.summary; fig.UserData.last = P;
        kpi.Text = sprintf(['Recovered %.1f Wh  |  Recovery %.1f %%  |  Final SOC %.3f %%  |  Peak power %.1f kW  |  ' ...
            'Stop %.2f s / %.1f m  |  Peak torque %.0f Nm  |  Peak %.0f rpm  |  Gear %.2f'], ...
            S.recovered_kwh*1000, S.recovery_pct, S.final_soc, S.peak_motor_power_kw, S.stop_time_s, ...
            S.stop_distance_m, S.peak_torque_nm, S.peak_rpm, P.gear_ratio);
        if S.grip_exceeded, kpi.Text = [kpi.Text '   !! decel exceeds tyre grip']; end
        plotLine(ax(1), R.t, R.v_kmh, 'Speed (km/h)');
        plotLine(ax(2), R.t, R.torque, 'Motor torque (Nm)');
        plotLine(ax(3), R.t, R.rpm, 'Motor speed (rpm)');
        cla(ax(4)); hold(ax(4),'on'); plot(ax(4),R.t,R.p_mech/1000); plot(ax(4),R.t,R.p_elec/1000); plot(ax(4),R.t,R.p_fric/1000,'--');
        legend(ax(4),'Motor mech.','Battery chg','Friction'); title(ax(4),'Power (kW)'); grid(ax(4),'on');
        cla(ax(5)); hold(ax(5),'on'); plot(ax(5),R.t,R.f_req); plot(ax(5),R.t,R.f_regen); plot(ax(5),R.t,R.f_fric,'--');
        legend(ax(5),'Required','Regen','Friction'); title(ax(5),'Braking force (N)'); grid(ax(5),'on');
        plotLine(ax(6), R.t, R.soc, 'Battery SOC (%)');
    end

    function plotLine(a,x,y,ttl)
        cla(a); plot(a,x,y,'LineWidth',1.5); title(a,ttl); xlabel(a,'Time (s)'); grid(a,'on');
    end

    function runSimulink()
        P = fig.UserData.last;
        try
            build_regen_model(P);
            out = sim('regen_braking'); y = out.simout_regen; t = out.tout;
            figure('Name','Simulink result'); subplot(2,1,1); plot(t, y(:,1)*3.6); ylabel('km/h');
            subplot(2,1,2); plot(t, y(:,3)); ylabel('SOC (%)'); xlabel('Time (s)');
            fprintf('Simulink: recovered %.1f Wh\n', y(end,4)/3600);
        catch err
            uialert(fig, err.message, 'Simulink run failed');
        end
    end
end
