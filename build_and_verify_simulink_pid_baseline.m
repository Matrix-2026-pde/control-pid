function report = build_and_verify_simulink_pid_baseline()
%BUILD_AND_VERIFY_SIMULINK_PID_BASELINE Build and verify PID-15 in Simulink.
projectPath = fileparts(mfilename('fullpath'));
model = 'pid_baseline_simulink';
modelFile = fullfile(projectPath,[model '.slx']);
p = struct('m',1,'c',1,'k',4,'Kp',10,'Ki',15,'Kd',3,'r',1);
T = 30;
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
load_system('simulink');
if isfile(modelFile)
    % Re-running this script verifies the existing independent artifact;
    % generation never overwrites it.
    load_system(model);
else
new_system(model); open_system(model);
set_param(model,'StopTime','30','Solver','ode45','RelTol','1e-8', ...
    'AbsTol','1e-10','ReturnWorkspaceOutputs','on');

% Reference, error, and explicit PID branches.
add_block('simulink/Sources/Step',[model '/Reference'],'Time','0', ...
    'Before','0','After','1','Position',[30 80 60 110]);
add_block('simulink/Math Operations/Sum',[model '/Error'],'Inputs','+-', ...
    'Position',[120 75 145 115]);
add_block('simulink/Math Operations/Gain',[model '/Kp'],'Gain','10', ...
    'Position',[190 20 250 50]);
add_block('simulink/Continuous/Integrator',[model '/Integral'], ...
    'InitialCondition','0','Position',[190 95 220 125]);
add_block('simulink/Math Operations/Gain',[model '/Ki'],'Gain','15', ...
    'Position',[260 95 320 125]);
add_block('simulink/Math Operations/Gain',[model '/Kd'],'Gain','3', ...
    'Position',[390 170 450 200]);
add_block('simulink/Math Operations/Sum',[model '/ControlSum'],'Inputs','++-', ...
    'Position',[500 65 530 125]);

% Plant: x''=(u-c*v-k*x)/m, v=integral(x''), x=integral(v).
add_block('simulink/Math Operations/Gain',[model '/Damping'],'Gain','1', ...
    'Position',[390 260 450 290]);
add_block('simulink/Math Operations/Gain',[model '/Spring'],'Gain','4', ...
    'Position',[390 330 450 360]);
add_block('simulink/Math Operations/Sum',[model '/PlantSum'],'Inputs','+--', ...
    'Position',[500 245 530 315]);
add_block('simulink/Math Operations/Gain',[model '/MassInverse'],'Gain','1', ...
    'Position',[560 260 620 290]);
add_block('simulink/Continuous/Integrator',[model '/Velocity'], ...
    'InitialCondition','0','Position',[660 255 690 285]);
add_block('simulink/Continuous/Integrator',[model '/Position'], ...
    'InitialCondition','0','Position',[730 255 760 285]);

% Structure With Time outputs make the comparison independent of display.
add_block('simulink/Sinks/To Workspace',[model '/y_out'],'VariableName','y_sim', ...
    'SaveFormat','Structure With Time','Position',[820 250 910 280]);
add_block('simulink/Sinks/To Workspace',[model '/v_out'],'VariableName','v_sim', ...
    'SaveFormat','Structure With Time','Position',[820 310 910 340]);
add_block('simulink/Sinks/To Workspace',[model '/u_out'],'VariableName','u_sim', ...
    'SaveFormat','Structure With Time','Position',[570 70 660 100]);

add_line(model,'Reference/1','Error/1'); add_line(model,'Position/1','Error/2');
add_line(model,'Error/1','Kp/1'); add_line(model,'Error/1','Integral/1');
add_line(model,'Integral/1','Ki/1'); add_line(model,'Kp/1','ControlSum/1');
add_line(model,'Ki/1','ControlSum/2'); add_line(model,'Velocity/1','Kd/1');
add_line(model,'Kd/1','ControlSum/3'); add_line(model,'ControlSum/1','u_out/1');
add_line(model,'ControlSum/1','PlantSum/1'); add_line(model,'Velocity/1','Damping/1');
add_line(model,'Damping/1','PlantSum/2'); add_line(model,'Position/1','Spring/1');
add_line(model,'Spring/1','PlantSum/3'); add_line(model,'PlantSum/1','MassInverse/1');
add_line(model,'MassInverse/1','Velocity/1'); add_line(model,'Velocity/1','Position/1');
add_line(model,'Position/1','y_out/1'); add_line(model,'Velocity/1','v_out/1');
set_param(model,'SimulationCommand','update'); save_system(model,modelFile);
end

simOut = sim(model,'ReturnWorkspaceOutputs','on');
ref = simulate_pid(p,T,[0;0;0],opts);
yS = simOut.get('y_sim'); vS = simOut.get('v_sim'); uS = simOut.get('u_sim');
assert(isstruct(yS) && isstruct(vS) && isstruct(uS), ...
    'Simulink outputs are not Structure With Time.');
assert(all(isfinite(yS.time)) && all(isfinite(yS.signals.values)), ...
    'Simulink position contains non-finite values.');
tq = linspace(0,T,30001).';
yM = interp1(ref.t,ref.y,tq,'pchip');
vM = interp1(ref.t,ref.x(:,2),tq,'pchip');
uM = interp1(ref.t,ref.u,tq,'pchip');
yQ = interp1(yS.time(:),yS.signals.values(:),tq,'pchip');
vQ = interp1(vS.time(:),vS.signals.values(:),tq,'pchip');
uQ = interp1(uS.time(:),uS.signals.values(:),tq,'pchip');
report = struct('model',modelFile,'solver','ode45','stop_time',T, ...
    'simulink_samples',numel(yS.time),'matlab_samples',numel(ref.t), ...
    'max_position_difference',max(abs(yQ-yM)), ...
    'max_velocity_difference',max(abs(vQ-vM)), ...
    'max_control_difference',max(abs(uQ-uM)), ...
    'matlab_steady_position',mean(ref.y(ref.t>=27)), ...
    'simulink_steady_position',mean(yQ(tq>=27)));
report.matlab_steady_error = abs(p.r-report.matlab_steady_position);
report.simulink_steady_error = abs(p.r-report.simulink_steady_position);
assert(report.max_position_difference < 2e-3,'Position mismatch exceeds 2e-3.');
assert(report.max_velocity_difference < 2e-3,'Velocity mismatch exceeds 2e-3.');
assert(report.max_control_difference < 2e-3,'Control mismatch exceeds 2e-3.');
assert(abs(report.simulink_steady_position-report.matlab_steady_position)<2e-4, ...
    'Steady position mismatch exceeds 2e-4.');
fprintf('Simulink PID-15 verification passed.\n');
fprintf('  model: %s\n',modelFile);
fprintf('  samples: Simulink %d, MATLAB %d\n',report.simulink_samples,report.matlab_samples);
fprintf('  max differences [position velocity control] = [%.3e %.3e %.3e]\n', ...
    report.max_position_difference,report.max_velocity_difference,report.max_control_difference);
fprintf('  steady position [MATLAB Simulink] = [%.12f %.12f]\n', ...
    report.matlab_steady_position,report.simulink_steady_position);
close_system(model,0);
end
