function [u_des, u_act, dx3, u_max] = pid_control_sat(x, p)
%PID_CONTROL_SAT  PID control law with actuator saturation and anti-windup.
%   [u_des, u_act, dx3, u_max] = PID_CONTROL_SAT(x, p)
%
%   x      state [position; velocity; integral state]
%   p      struct with m, c, k, Kp, Ki, Kd, r and optional
%          u_max    actuator limit (default Inf, i.e. ideal actuator)
%          aw_mode  'none' (default) or 'clamp'
%
%   u_des  control force requested by the PID law
%   u_act  force actually applied by the saturated actuator
%   dx3    derivative of the integral state, including the anti-windup rule
%   u_max  actuator limit actually used
%
%   This is the single implementation of the control law. pid_rhs_sat.m,
%   simulate_saturation.m, and the verification scripts all call it, so the
%   saturation and clamping rules cannot drift apart between integration and
%   post-processing.
%
%   Clamping rule: freeze the integrator when the actuator is saturated and
%   the error would push the requested force further into the same limit:
%       (u_des - u_act) * e > 0  =>  dx3 = 0
%   Otherwise integrate normally. When u_des == u_act the product is zero, so
%   an unsaturated controller always integrates.

e = p.r - x(1);
u_des = p.Kp*e + p.Ki*x(3) - p.Kd*x(2);

if isfield(p, 'u_max') && ~isempty(p.u_max)
    u_max = p.u_max;
else
    u_max = Inf;
end
if ~(isscalar(u_max) && u_max > 0)
    error('pid_control_sat:badLimit', 'p.u_max must be a positive scalar.');
end

u_act = min(u_max, max(-u_max, u_des));

if isfield(p, 'aw_mode') && ~isempty(p.aw_mode)
    aw_mode = lower(p.aw_mode);
else
    aw_mode = 'none';
end

switch aw_mode
    case 'none'
        dx3 = e;
    case 'clamp'
        if (u_des - u_act) * e > 0
            dx3 = 0;
        else
            dx3 = e;
        end
    otherwise
        error('pid_control_sat:badMode', ...
            'Unknown p.aw_mode ''%s''; use ''none'' or ''clamp''.', aw_mode);
end
end
