function out = simulate_saturation(p, T, x0, opts)
%SIMULATE_SATURATION  One closed-loop run with actuator saturation.
%   out = SIMULATE_SATURATION(p, T, x0, opts)
%   p must contain m, c, k, Kp, Ki, Kd, r and may contain u_max, aw_mode.
%   x0   initial state [x; xdot; integral] (default [0;0;0])
%   opts odeset options (default RelTol=1e-8, AbsTol=1e-10)
%
%   Returned fields: t, x, y, e, u_des, u_act, x3, u_max, metrics,
%   local_roots, local_routh_margin, local_zero.
%
%   u_des and u_act are recovered from the recorded states by the same
%   pid_control_sat.m used inside the ODE, so they are consistent with the
%   integrated trajectory.
%
%   local_roots describe the UNSATURATED linear closed loop only. They are
%   not valid while the actuator is at its limit and must not be used to
%   predict the saturated transient.

if nargin < 3 || isempty(x0)
    x0 = [0; 0; 0];
end
if nargin < 4 || isempty(opts)
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
end

[t, x] = ode45(@(tt, xx) pid_rhs_sat(tt, xx, p), [0 T], x0, opts);

n = numel(t);
u_des = zeros(n, 1);
u_act = zeros(n, 1);
u_max = NaN;
for k = 1:n
    [u_des(k), u_act(k), ~, u_max] = pid_control_sat(x(k, :).', p);
end

y = x(:, 1);
e = p.r - y;

out.t = t;
out.x = x;
out.y = y;
out.e = e;
out.u_des = u_des;
out.u_act = u_act;
out.x3 = x(:, 3);
out.u_max = u_max;
out.metrics = saturation_metrics(t, y, e, u_des, u_act, x(:, 3), p, u_max, [], x(:,2));

% Unsaturated linear model, for reference only.
if p.Ki > 0
    out.local_roots = roots([p.m, p.c + p.Kd, p.k + p.Kp, p.Ki]);
    out.local_routh_margin = (p.c + p.Kd)*(p.k + p.Kp) - p.m*p.Ki;
    out.local_zero = -p.Ki/p.Kp;
else
    out.local_roots = roots([p.m, p.c + p.Kd, p.k + p.Kp]);
    out.local_routh_margin = NaN;
    out.local_zero = NaN;
end
end
