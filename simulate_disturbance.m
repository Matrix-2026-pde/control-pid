function out = simulate_disturbance(p, T, x0, opts)
%SIMULATE_DISTURBANCE  Step reference, then a constant disturbance at t_dist.
%   out = SIMULATE_DISTURBANCE(p, T, x0, opts)
%   p must contain m, c, k, Kp, Ki, Kd, r, t_dist, d0.
%   T is the final time (T > p.t_dist).
%
%   The run is split into two ODE segments:
%     [0, t_dist]      d = 0, using the unmodified pid_rhs.m
%     [t_dist, T]      d = d0, using pid_rhs_dist.m
%   Splitting at t_dist avoids smearing the force step across a solver step
%   and makes the pre-disturbance trajectory identical to the baseline.
%
%   Returned fields: t, x, y, e, u, d, metrics, pre.
%   out.pre is the baseline segment produced by simulate_pid.m.

if nargin < 3 || isempty(x0)
    x0 = [0; 0; 0];
end
if nargin < 4 || isempty(opts)
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
end
assert(T > p.t_dist, 'T must be greater than p.t_dist');

% Pre-disturbance segment: exactly the original closed loop.
pre = simulate_pid(p, p.t_dist, x0, opts);

% Post-disturbance segment: start from the state reached at t_dist.
[t2, x2] = ode45(@(tt, xx) pid_rhs_dist(tt, xx, p), ...
    [p.t_dist T], pre.x(end, :).', opts);

t = [pre.t; t2(2:end)];
x = [pre.x; x2(2:end, :)];
y = x(:, 1);
e = p.r - y;
u = p.Kp*e + p.Ki*x(:, 3) - p.Kd*x(:, 2);
d = [zeros(size(pre.t)); p.d0 * ones(size(t2(2:end)))];

out.t = t;
out.x = x;
out.y = y;
out.e = e;
out.u = u;
out.d = d;
out.pre = pre;
out.metrics = disturbance_metrics(t, y, e, u, p);
end
