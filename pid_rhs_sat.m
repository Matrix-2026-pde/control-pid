function dx = pid_rhs_sat(~, x, p)
%PID_RHS_SAT  Closed-loop RHS with actuator saturation and optional clamping.
%   State: x(1) = position, x(2) = velocity, x(3) = integral state.
%
%   Plant:      m*x'' + c*x' + k*x = u_act
%   Controller: u_des = Kp*(r-x1) + Ki*x3 - Kd*x2
%               u_act = sat(u_des)
%
%   The integral derivative dx3 is produced by pid_control_sat.m so the
%   anti-windup rule is identical to the one used for recording and plotting.
%   The unsaturated local poles are not valid while the actuator is at its
%   limit; they are reported only for reference.

[~, u_act, dx3] = pid_control_sat(x, p);

dx = [ ...
    x(2); ...
    (u_act - p.c*x(2) - p.k*x(1))/p.m; ...
    dx3 ];
end
