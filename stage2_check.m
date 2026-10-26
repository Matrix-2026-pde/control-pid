% STAGE2_CHECK  Verify that the modular implementation matches stage 1.
% The same gain set is run twice: once with the inline RHS from stage 1 and
% once through simulate_pid. The two trajectories and metrics must agree.

p = struct('m', 1, 'c', 1, 'k', 4, 'Kp', 10, 'Ki', 15, 'Kd', 3, 'r', 1);
T = 30;
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

rhs_inline = @(t, x) [ ...
    x(2); ...
    (p.Kp*(p.r - x(1)) + p.Ki*x(3) - p.Kd*x(2) - p.c*x(2) - p.k*x(1))/p.m; ...
    p.r - x(1) ];
[t1, x1] = ode45(rhs_inline, [0 T], [0; 0; 0], opts);
y1 = x1(:,1);

out = simulate_pid(p, T, [0; 0; 0], opts);

tq = linspace(0, T, 30001).';
y1q = interp1(t1, y1, tq, 'pchip');
y2q = interp1(out.t, out.y, tq, 'pchip');
max_diff = max(abs(y1q - y2q));

fprintf('Stage 2 refactor check: max|y_inline - y_modular| = %.3e\n', max_diff);
assert(max_diff < 1e-7, 'modular implementation changed the trajectory');

s1 = pid_metrics(t1, y1, p.r, [], x1(:,2));
fprintf('  stage1 metrics: rise %.6f, overshoot %.6f, settle %.6f, ess %.3e\n', ...
    s1.rise_time, s1.overshoot_pct, s1.settling_time, s1.steady_error);
fprintf('  modular metrics: rise %.6f, overshoot %.6f, settle %.6f, ess %.3e\n', ...
    out.metrics.rise_time, out.metrics.overshoot_pct, ...
    out.metrics.settling_time, out.metrics.steady_error);

assert(abs(s1.rise_time - out.metrics.rise_time) < 1e-6, 'rise time changed');
assert(abs(s1.overshoot_pct - out.metrics.overshoot_pct) < 1e-6, 'overshoot changed');
assert(abs(s1.settling_time - out.metrics.settling_time) < 1e-6, 'settling time changed');
assert(abs(s1.steady_error - out.metrics.steady_error) < 1e-9, 'steady error changed');

fprintf('Stage 2 refactor check passed.\n');
