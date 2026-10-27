function report = build_and_verify_simulink_pid_saturation()
%BUILD_AND_VERIFY_SIMULINK_PID_SATURATION Build and verify stage-3 model.
% The model contains three parallel, independently logged experiments:
% G1 ideal actuator, G2 saturation without anti-windup, G3 saturation with
% the exact clamping rule used by pid_control_sat.m.

projectPath = fileparts(mfilename('fullpath'));
model = 'pid_saturation_simulink';
modelFile = fullfile(projectPath,[model '.slx']);
T = 30;
p = struct('m',1,'c',1,'k',4,'Kp',10,'Ki',15,'Kd',3,'r',1);
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
load_system('simulink');

if isfile(modelFile)
    load_system(model);
else
    if bdIsLoaded(model)
        close_system(model,0);
    end
    new_system(model);
    set_param(model,'StopTime',num2str(T),'Solver','ode45','RelTol','1e-8', ...
        'AbsTol','1e-10','MaxStep','0.005','ReturnWorkspaceOutputs','on');
    add_block('simulink/Sources/Step',[model '/Reference'],'Time','0', ...
        'Before','0','After','1','Position',[30 260 60 290]);
    for g = 1:3
        make_branch(model,g,140 + (g-1)*300, g==1, g==3);
    end
    set_param(model,'SimulationCommand','update');
    save_system(model,modelFile);
end

simOut = sim(model,'ReturnWorkspaceOutputs','on');
ref = cell(1,3);
p1 = p; p1.u_max = Inf; p1.aw_mode = 'none'; ref{1} = simulate_saturation(p1,T,[0;0;0],opts);
p2 = p; p2.u_max = 5; p2.aw_mode = 'none'; ref{2} = simulate_saturation(p2,T,[0;0;0],opts);
p3 = p; p3.u_max = 5; p3.aw_mode = 'clamp'; ref{3} = simulate_saturation(p3,T,[0;0;0],opts);

tq = linspace(0,T,30001).';
names = {'G1','G2','G3'};
report = struct('model',modelFile,'solver','ode45','stop_time',T);
for g = 1:3
    yS = simOut.get([names{g} '_y']); vS = simOut.get([names{g} '_v']);
    x3S = simOut.get([names{g} '_x3']); udS = simOut.get([names{g} '_u_des']);
    uaS = simOut.get([names{g} '_u_act']);
    vals = {yS,vS,x3S,udS,uaS};
    for j = 1:numel(vals)
        assert(isstruct(vals{j}) && all(isfinite(vals{j}.time)) && ...
            all(isfinite(vals{j}.signals.values)), 'Non-finite Simulink output.');
    end
    m = ref{g}; q = {m.y,m.x(:,2),m.x3,m.u_des,m.u_act};
    d = zeros(1,5);
    for j = 1:5
        sq = interp1(vals{j}.time(:),vals{j}.signals.values(:),tq,'pchip');
        mq = interp1(m.t,q{j},tq,'pchip');
        d(j) = max(abs(sq-mq));
    end
    report.(lower(names{g})) = struct('max_position_difference',d(1), ...
        'max_velocity_difference',d(2),'max_integral_difference',d(3), ...
        'max_desired_force_difference',d(4),'max_applied_force_difference',d(5), ...
        'matlab_metrics',m.metrics);
    assert(all(d(1:3) < 2e-3), ...
        'Position, velocity, or integral-state mismatch exceeds 2e-3.');
end

% Direct model-level checks of the anti-windup state and actuator behavior.
assert(report.g1.max_applied_force_difference < 2e-3);
assert(report.g2.max_integral_difference < 2e-3);
assert(report.g3.max_integral_difference < 2e-3);
for g = 2:3
    ua = simOut.get([names{g} '_u_act']);
    assert(all(abs(ua.signals.values) <= 5+1e-9),'Actuator limit exceeded.');
end
g3y = simOut.get('G3_y'); g3x3 = simOut.get('G3_x3');
g3ud = simOut.get('G3_u_des'); g3ua = simOut.get('G3_u_act');
g3e = 1 - g3y.signals.values(:);
g3push = (g3ud.signals.values(:)-g3ua.signals.values(:)).*g3e > 1e-10;
g3dt = diff(g3x3.time(:));
g3dx3 = diff(g3x3.signals.values(:))./g3dt;
g3frozen = g3push(1:end-1) & g3push(2:end);
assert(any(g3frozen),'G3 has no saturated-pushing samples in Simulink output.');
report.g3.clamping_max_abs_dx3 = max(abs(g3dx3(g3frozen)));
assert(report.g3.clamping_max_abs_dx3 < 1e-9, ...
    'G3 integral state was not frozen under the clamping condition.');
fprintf('Simulink stage-3 saturation verification passed.\n');
fprintf('  model: %s\n',modelFile);
for g = 1:3
    r = report.(lower(names{g}));
    fprintf('  %s max differences [y v x3 u_des u_act] = [%.3e %.3e %.3e %.3e %.3e]\n', ...
        names{g},r.max_position_difference,r.max_velocity_difference, ...
        r.max_integral_difference,r.max_desired_force_difference, ...
        r.max_applied_force_difference);
end
fprintf('  G3 clamping max|dx3| on pushing intervals = %.3e\n', ...
    report.g3.clamping_max_abs_dx3);
close_system(model,0);
end

function make_branch(model,g,x0,isIdeal,isClamp)
prefix = sprintf('%s/G%d_',model,g);
y = x0 + 170;
limit = '5';
if isIdeal, limit = 'Inf'; end
add_block('simulink/Math Operations/Sum',[prefix 'Error'],'Inputs','+-','Position',[x0 y x0+30 y+30]);
add_block('simulink/Math Operations/Gain',[prefix 'Kp'],'Gain','10','Position',[x0+70 y-50 x0+125 y-20]);
add_block('simulink/Math Operations/Gain',[prefix 'Ki'],'Gain','15','Position',[x0+70 y+20 x0+125 y+50]);
add_block('simulink/Math Operations/Gain',[prefix 'Kd'],'Gain','3','Position',[x0+70 y+90 x0+125 y+120]);
add_block('simulink/Math Operations/Sum',[prefix 'ControlSum'],'Inputs','++-','Position',[x0+160 y-5 x0+190 y+55]);
add_block('simulink/Discontinuities/Saturation',[prefix 'Actuator'], ...
    'UpperLimit',limit,'LowerLimit',['-' limit],'Position',[x0+220 y-5 x0+280 y+55]);
add_block('simulink/Math Operations/Sum',[prefix 'Gap'],'Inputs','+-','Position',[x0+305 y-70 x0+335 y-30]);
add_block('simulink/Math Operations/Product',[prefix 'PushProduct'],'Position',[x0+355 y-35 x0+390 y+5]);
add_block('simulink/Logic and Bit Operations/Relational Operator',[prefix 'PushTest'], ...
    'Operator','>','Position',[x0+410 y-35 x0+455 y+5]);
add_block('simulink/Sources/Constant',[prefix 'Zero'],'Value','0','Position',[x0+410 y+30 x0+455 y+60]);
add_block('simulink/Signal Routing/Switch',[prefix 'ClampSwitch'],'Threshold','0', ...
    'Criteria','u2 > Threshold','Position',[x0+480 y+45 x0+530 y+85]);
add_block('simulink/Continuous/Integrator',[prefix 'Integral'],'InitialCondition','0','Position',[x0+230 y+55 x0+260 y+85]);
add_block('simulink/Continuous/Integrator',[prefix 'Velocity'],'InitialCondition','0','Position',[x0+470 y+5 x0+500 y+35]);
add_block('simulink/Continuous/Integrator',[prefix 'Position'],'InitialCondition','0','Position',[x0+540 y+5 x0+570 y+35]);
add_block('simulink/Math Operations/Sum',[prefix 'PlantSum'],'Inputs','+--','Position',[x0+310 y+80 x0+340 y+130]);
add_block('simulink/Math Operations/Gain',[prefix 'Damping'],'Gain','1','Position',[x0+230 y+150 x0+290 y+180]);
add_block('simulink/Math Operations/Gain',[prefix 'Spring'],'Gain','4','Position',[x0+390 y+190 x0+450 y+220]);
add_block('simulink/Math Operations/Gain',[prefix 'MassInverse'],'Gain','1','Position',[x0+370 y+95 x0+430 y+125]);
outs = {'y','v','x3','u_des','u_act'};
for j=1:numel(outs)
    add_block('simulink/Sinks/To Workspace',[prefix outs{j}], ...
        'VariableName',sprintf('G%d_%s',g,outs{j}), ...
        'SaveFormat','Structure With Time','Position',[x0+610 y-80+j*45 x0+710 y-50+j*45]);
end
add_line(model,'Reference/1',[sprintf('G%d_Error',g) '/1']);
add_line(model,sprintf('G%d_Position/1',g),sprintf('G%d_Error/2',g));
add_line(model,sprintf('G%d_Error/1',g),sprintf('G%d_Kp/1',g));
add_line(model,sprintf('G%d_Integral/1',g),sprintf('G%d_Ki/1',g));
add_line(model,sprintf('G%d_Velocity/1',g),sprintf('G%d_Kd/1',g));
add_line(model,sprintf('G%d_Kp/1',g),sprintf('G%d_ControlSum/1',g));
add_line(model,sprintf('G%d_Ki/1',g),sprintf('G%d_ControlSum/2',g));
add_line(model,sprintf('G%d_Kd/1',g),sprintf('G%d_ControlSum/3',g));
add_line(model,sprintf('G%d_ControlSum/1',g),sprintf('G%d_Actuator/1',g));
add_line(model,sprintf('G%d_ControlSum/1',g),sprintf('G%d_Gap/1',g));
add_line(model,sprintf('G%d_Actuator/1',g),sprintf('G%d_Gap/2',g));
add_line(model,sprintf('G%d_Gap/1',g),sprintf('G%d_PushProduct/1',g));
add_line(model,sprintf('G%d_Error/1',g),sprintf('G%d_PushProduct/2',g));
add_line(model,sprintf('G%d_PushProduct/1',g),sprintf('G%d_PushTest/1',g));
add_line(model,sprintf('G%d_Zero/1',g),sprintf('G%d_PushTest/2',g));
    add_line(model,sprintf('G%d_Zero/1',g),sprintf('G%d_ClampSwitch/1',g));
add_line(model,sprintf('G%d_PushTest/1',g),sprintf('G%d_ClampSwitch/2',g));
    add_line(model,sprintf('G%d_Error/1',g),sprintf('G%d_ClampSwitch/3',g));
if isClamp
    add_line(model,sprintf('G%d_ClampSwitch/1',g),sprintf('G%d_Integral/1',g));
else
    add_line(model,sprintf('G%d_Error/1',g),sprintf('G%d_Integral/1',g));
end
add_line(model,sprintf('G%d_Actuator/1',g),sprintf('G%d_PlantSum/1',g));
add_line(model,sprintf('G%d_Velocity/1',g),sprintf('G%d_Damping/1',g));
add_line(model,sprintf('G%d_Damping/1',g),sprintf('G%d_PlantSum/2',g));
add_line(model,sprintf('G%d_Position/1',g),sprintf('G%d_Spring/1',g));
add_line(model,sprintf('G%d_Spring/1',g),sprintf('G%d_PlantSum/3',g));
add_line(model,sprintf('G%d_PlantSum/1',g),sprintf('G%d_MassInverse/1',g));
add_line(model,sprintf('G%d_MassInverse/1',g),sprintf('G%d_Velocity/1',g));
add_line(model,sprintf('G%d_Velocity/1',g),sprintf('G%d_Position/1',g));
add_line(model,sprintf('G%d_Position/1',g),sprintf('G%d_y/1',g));
add_line(model,sprintf('G%d_Velocity/1',g),sprintf('G%d_v/1',g));
add_line(model,sprintf('G%d_Integral/1',g),sprintf('G%d_x3/1',g));
add_line(model,sprintf('G%d_ControlSum/1',g),sprintf('G%d_u_des/1',g));
add_line(model,sprintf('G%d_Actuator/1',g),sprintf('G%d_u_act/1',g));
end
