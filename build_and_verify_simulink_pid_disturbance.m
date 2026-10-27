function report = build_and_verify_simulink_pid_disturbance()
%BUILD_AND_VERIFY_SIMULINK_PID_DISTURBANCE Compare P, PD and PID-15.
root=fileparts(mfilename('fullpath')); model='pid_disturbance_simulink';
file=fullfile(root,[model '.slx']); load_system('simulink');
if isfile(file)
    load_system(model);
else
    new_system(model); open_system(model);
    set_param(model,'StopTime','30','Solver','ode45','RelTol','1e-8','AbsTol','1e-10','MaxStep','0.005','ReturnWorkspaceOutputs','on');
    add_block('simulink/Sources/Step',[model '/Reference'],'Time','0','Before','0','After','1','Position',[30 70 60 100]);
    add_block('simulink/Sources/Step',[model '/Disturbance'],'Time','10','Before','0','After','-4','Position',[30 430 60 460]);
    add_block('simulink/Math Operations/Sum',[model '/Error'],'Inputs','+-','Position',[120 65 145 105]);
    add_block('simulink/Math Operations/Gain',[model '/Kp'],'Gain','10','Position',[190 10 250 40]);
    add_block('simulink/Continuous/Integrator',[model '/Integral'],'InitialCondition','0','Position',[190 85 220 115]);
    add_block('simulink/Math Operations/Gain',[model '/Ki'],'Gain','15','Position',[260 85 320 115]);
    add_block('simulink/Math Operations/Gain',[model '/Kd'],'Gain','3','Position',[390 165 450 195]);
    add_block('simulink/Math Operations/Sum',[model '/ControlSum'],'Inputs','++-','Position',[500 55 530 115]);
    add_block('simulink/Math Operations/Gain',[model '/Damping'],'Gain','1','Position',[390 255 450 285]);
    add_block('simulink/Math Operations/Gain',[model '/Spring'],'Gain','4','Position',[390 325 450 355]);
    add_block('simulink/Math Operations/Sum',[model '/PlantSum'],'Inputs','+--+','Position',[500 240 530 320]);
    add_block('simulink/Math Operations/Gain',[model '/MassInverse'],'Gain','1','Position',[560 255 620 285]);
    add_block('simulink/Continuous/Integrator',[model '/Velocity'],'InitialCondition','0','Position',[660 250 690 280]);
    add_block('simulink/Continuous/Integrator',[model '/Position'],'InitialCondition','0','Position',[730 250 760 280]);
    add_block('simulink/Sinks/To Workspace',[model '/y_out'],'VariableName','y_sim','SaveFormat','Structure With Time','Position',[820 245 910 275]);
    add_block('simulink/Sinks/To Workspace',[model '/v_out'],'VariableName','v_sim','SaveFormat','Structure With Time','Position',[820 305 910 335]);
    add_block('simulink/Sinks/To Workspace',[model '/u_out'],'VariableName','u_sim','SaveFormat','Structure With Time','Position',[570 60 660 90]);
    add_block('simulink/Sinks/To Workspace',[model '/d_out'],'VariableName','d_sim','SaveFormat','Structure With Time','Position',[100 430 190 460]);
    add_line(model,'Reference/1','Error/1'); add_line(model,'Position/1','Error/2'); add_line(model,'Error/1','Kp/1'); add_line(model,'Error/1','Integral/1');
    add_line(model,'Integral/1','Ki/1'); add_line(model,'Kp/1','ControlSum/1'); add_line(model,'Ki/1','ControlSum/2'); add_line(model,'Velocity/1','Kd/1'); add_line(model,'Kd/1','ControlSum/3');
    add_line(model,'ControlSum/1','u_out/1'); add_line(model,'ControlSum/1','PlantSum/1'); add_line(model,'Velocity/1','Damping/1'); add_line(model,'Damping/1','PlantSum/2');
    add_line(model,'Position/1','Spring/1'); add_line(model,'Spring/1','PlantSum/3'); add_line(model,'Disturbance/1','PlantSum/4'); add_line(model,'Disturbance/1','d_out/1');
    add_line(model,'PlantSum/1','MassInverse/1'); add_line(model,'MassInverse/1','Velocity/1'); add_line(model,'Velocity/1','Position/1'); add_line(model,'Position/1','y_out/1'); add_line(model,'Velocity/1','v_out/1');
    set_param(model,'SimulationCommand','update'); save_system(model,file);
end
p0=struct('m',1,'c',1,'k',4,'r',1,'t_dist',10,'d0',-4); T=30; opts=odeset('RelTol',1e-8,'AbsTol',1e-10); tq=linspace(0,T,30001).';
names={'P','PD','PID-15'}; gains=[10 0 0;10 0 3;10 15 3];
report=repmat(struct('name','','max_position_difference',0,'max_velocity_difference',0,'max_control_difference',0,'pre_position_difference',0,'post_position_difference',0,'matlab_pre_position',0,'matlab_post_steady_position',0,'simulink_post_steady_position',0,'matlab_post_steady_error',0,'simulink_post_steady_error',0,'matlab_max_deviation',0,'matlab_max_excursion',0,'matlab_delta_u',0,'matlab_recovery_time',NaN),1,3);
for j=1:3
    set_param([model '/Kp'],'Gain',num2str(gains(j,1))); set_param([model '/Ki'],'Gain',num2str(gains(j,2))); set_param([model '/Kd'],'Gain',num2str(gains(j,3))); set_param(model,'SimulationCommand','update');
    so=sim(model,'ReturnWorkspaceOutputs','on'); ys=so.get('y_sim'); vs=so.get('v_sim'); us=so.get('u_sim'); ds=so.get('d_sim');
    p=p0; p.Kp=gains(j,1); p.Ki=gains(j,2); p.Kd=gains(j,3); ref=simulate_disturbance(p,T,[0;0;0],opts);
    yM=interp1(ref.t,ref.y,tq,'pchip'); vM=interp1(ref.t,ref.x(:,2),tq,'pchip'); uM=interp1(ref.t,ref.u,tq,'pchip'); yQ=interp1(ys.time,ys.signals.values,tq,'pchip'); vQ=interp1(vs.time,vs.signals.values,tq,'pchip'); uQ=interp1(us.time,us.signals.values,tq,'pchip');
    pre=tq<10; post=~pre; report(j).name=names{j}; report(j).max_position_difference=max(abs(yQ-yM)); report(j).max_velocity_difference=max(abs(vQ-vM)); report(j).max_control_difference=max(abs(uQ-uM)); report(j).pre_position_difference=max(abs(yQ(pre)-yM(pre))); report(j).post_position_difference=max(abs(yQ(post)-yM(post)));
    report(j).matlab_pre_position=ref.metrics.pre_value; report(j).matlab_post_steady_position=mean(yM(tq>=27)); report(j).simulink_post_steady_position=mean(yQ(tq>=27)); report(j).matlab_post_steady_error=abs(p.r-report(j).matlab_post_steady_position); report(j).simulink_post_steady_error=abs(p.r-report(j).simulink_post_steady_position); report(j).matlab_max_deviation=ref.metrics.max_deviation; report(j).matlab_max_excursion=ref.metrics.max_excursion; report(j).matlab_delta_u=ref.metrics.delta_u; report(j).matlab_recovery_time=ref.metrics.recovery_time;
    assert(all(isfinite(ds.signals.values))); assert(report(j).max_position_difference<2e-3); assert(report(j).max_velocity_difference<2e-3); assert(report(j).max_control_difference<2e-3);
    fprintf('%s: max [x v u]=[%.3e %.3e %.3e], pre/post diff=[%.3e %.3e], y_pre=%.6f, y_post=[%.9f %.9f], maxdev=%.6f, du=%.6f, recovery=%.6g\n',names{j},report(j).max_position_difference,report(j).max_velocity_difference,report(j).max_control_difference,report(j).pre_position_difference,report(j).post_position_difference,report(j).matlab_pre_position,report(j).matlab_post_steady_position,report(j).simulink_post_steady_position,report(j).matlab_max_deviation,report(j).matlab_delta_u,report(j).matlab_recovery_time);
end
close_system(model,0);
end
