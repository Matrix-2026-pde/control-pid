function out = simulate_pid(p, T, x0, opts)
%SIMULATE_PID  Simulate one closed-loop PID run.
%   out = SIMULATE_PID(p, T, x0, opts)
%   p     struct with fields m, c, k, Kp, Ki, Kd, r
%   T     final time (s)
%   x0    initial state [x; xdot; integral of error] (default [0;0;0])
%   opts  odeset options (default RelTol=1e-8, AbsTol=1e-10)
%
%   Returned fields include t, x, y, e, u, metrics, and pole information.
%   For Ki = 0 the reduced second-order model is reported (wn, zeta,
%   reduced_roots). For Ki > 0 the system is third order: only the cubic
%   roots and the Routh margin are reported; no single damping ratio is
%   assigned.

if nargin < 3 || isempty(x0)
    x0 = [0; 0; 0];
end
if nargin < 4 || isempty(opts)
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
end

[t, x] = ode45(@(tt, xx) pid_rhs(tt, xx, p), [0 T], x0, opts);

y = x(:,1);
e = p.r - y;
u = p.Kp*e + p.Ki*x(:,3) - p.Kd*x(:,2);

out.t = t;
out.x = x;
out.y = y;
out.e = e;
out.u = u;
out.metrics = pid_metrics(t, y, p.r, [], x(:,2));
out.metrics.max_u = max(abs(u));

% Closed-loop zero for Ki > 0. The numerator of X(s)/R(s) is Kp*s + Ki,
% so the finite zero sits at -Ki/Kp. It shapes the third-order response and
% must be read together with the poles; it is not captured by a single zeta.
if p.Ki > 0
    out.zero = -p.Ki/p.Kp;
else
    out.zero = NaN;
end

% Full characteristic polynomial of the closed loop:
%   m*s^3 + (c+Kd)*s^2 + (k+Kp)*s + Ki = 0
out.roots = roots([p.m, p.c + p.Kd, p.k + p.Kp, p.Ki]);

if p.Ki == 0
    % Ki = 0: the cubic degenerates to s*(m*s^2 + (c+Kd)*s + (k+Kp)) = 0.
    % The s = 0 pole belongs to the unused integral state. Stability and
    % damping are read from the reduced quadratic, not from the cubic.
    out.reduced_roots = roots([p.m, p.c + p.Kd, p.k + p.Kp]);
    out.wn = sqrt((p.k + p.Kp)/p.m);
    out.zeta = (p.c + p.Kd)/(2*sqrt(p.m*(p.k + p.Kp)));
    out.routh_margin = NaN;
else
    % Ki > 0: genuinely third order. Do not collapse it to one zeta.
    out.reduced_roots = [];
    out.wn = NaN;
    out.zeta = NaN;
    out.routh_margin = (p.c + p.Kd)*(p.k + p.Kp) - p.m*p.Ki;
end
end
