% VERIFY_SATURATION  Independent checks for the stage-3 saturation experiment.
%
%   1) control-law math rules (saturation, clamping, defaults, bad inputs)
%   2) G1 reproduces the stage-1 PID-15 baseline
%   3) G2/G3 stay within the actuator limit and actually saturate
%   4) the clamping invariant holds along the G3 trajectory
%   5) steady state, force balance, and integral state match theory
%   6) solver-tolerance convergence for the saturated/clamped RHS
%   7) performance ordering, with thresholds calibrated by the pilot run
%
% The directional thresholds below are project-level observations at
% m=1, c=1, k=4, r=1, PID-15, u_max=5 N. They are not general claims about
% every saturated system.

p0 = struct('m', 1, 'c', 1, 'k', 4, 'Kp', 10, 'Ki', 15, 'Kd', 3, 'r', 1);
T = 30;
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
u_lim = 5;
x3_expected = p0.k*p0.r/p0.Ki;

fprintf('=== verify_saturation ===\n');

%% 1) Control-law math rules
fprintf('\n1) Control-law math rules\n');

pm = p0; pm.u_max = u_lim; pm.aw_mode = 'none';
[ud, ua, dx3, um] = pid_control_sat([0; 0; 0], pm);
fprintf('   t=0 state: u_des=%.3f, u_act=%.3f, dx3=%.3f, u_max=%.3f\n', ...
    ud, ua, dx3, um);
assert(abs(ud - 10) < 1e-12, 'initial u_des should be Kp*r');
assert(abs(ua - u_lim) < 1e-12, 'initial u_act should be clamped to u_max');
assert(abs(dx3 - 1) < 1e-12, 'no-AW integral derivative should be e');
assert(um == u_lim, 'u_max output should match p.u_max');

pm.aw_mode = 'clamp';
[~, ~, dx3] = pid_control_sat([0; 0; 0], pm);
assert(dx3 == 0, 'clamping should freeze the integrator at t=0');
fprintf('   clamp, pushing into limit: dx3=%.3e\n', dx3);

[~, ~, dx3] = pid_control_sat([0.5; 0; 0], pm);
assert(abs(dx3 - 0.5) < 1e-12, 'u_des at the limit should still integrate');
fprintf('   clamp, u_des exactly at limit: dx3=%.3f\n', dx3);

[ud, ua, dx3] = pid_control_sat([1.2; 0; 0.5], pm);
fprintf('   clamp, saturated but e<0: u_des=%.3f, u_act=%.3f, dx3=%.3f\n', ...
    ud, ua, dx3);
assert(abs(ud - 5.5) < 1e-12 && abs(ua - 5) < 1e-12, ...
    'positive saturated test point is wrong');
assert(abs(dx3 - (-0.2)) < 1e-12, ...
    'clamping should integrate when the error helps unwind');

[ud, ua, dx3] = pid_control_sat([3; 0; 0], pm);
fprintf('   clamp, negative saturation, e<0: u_des=%.3f, u_act=%.3f, dx3=%.3f\n', ...
    ud, ua, dx3);
assert(ud < -u_lim && abs(ua + u_lim) < 1e-12, 'negative saturation is wrong');
assert(dx3 == 0, 'clamping should freeze under negative saturation too');

[ud, ua, dx3] = pid_control_sat([0; 0; -2], pm);
fprintf('   clamp, negative saturation, e>0: u_des=%.3f, u_act=%.3f, dx3=%.3f\n', ...
    ud, ua, dx3);
assert(ud < -u_lim && abs(ua + u_lim) < 1e-12, 'negative saturation is wrong');
assert(abs(dx3 - 1) < 1e-12, 'clamping should integrate when e helps unwind');

pn = p0; pn.aw_mode = 'none';
[ud, ua, dx3, um] = pid_control_sat([0; 0; 0], pn);
fprintf('   default u_max: um=%.3f, u_des=%.3f, u_act=%.3f, dx3=%.3f\n', ...
    um, ud, ua, dx3);
assert(isinf(um), 'default actuator limit should be Inf');
assert(abs(ua - ud) < 1e-12, 'ideal actuator should not change u_des');
assert(abs(dx3 - 1) < 1e-12, 'default integral law should be e');

bad = p0; bad.u_max = 0;
threw = false;
try
    pid_control_sat([0; 0; 0], bad);
catch ME
    threw = true;
    assert(strcmp(ME.identifier, 'pid_control_sat:badLimit'), ...
        'u_max=0 raised the wrong error');
end
assert(threw, 'u_max=0 should raise an error');

bad = p0; bad.u_max = u_lim; bad.aw_mode = 'bogus';
threw = false;
try
    pid_control_sat([0; 0; 0], bad);
catch ME
    threw = true;
    assert(strcmp(ME.identifier, 'pid_control_sat:badMode'), ...
        'unknown aw_mode raised the wrong error');
end
assert(threw, 'unknown aw_mode should raise an error');

% RHS must use the same rule as the helper.
x_test = [0.2; 0.4; 0.1];
dx_test = pid_rhs_sat(0, x_test, pm);
[~, ua_test, dx3_test] = pid_control_sat(x_test, pm);
assert(abs(dx_test(2) - (ua_test - p0.c*x_test(2) - p0.k*x_test(1))/p0.m) < 1e-12, ...
    'pid_rhs_sat acceleration does not use u_act');
assert(abs(dx_test(3) - dx3_test) < 1e-12, ...
    'pid_rhs_sat integral derivative differs from pid_control_sat');
fprintf('   pid_rhs_sat agrees with pid_control_sat at a test state\n');

%% 2) G1 baseline equivalence
fprintf('\n2) G1 baseline equivalence\n');

p1 = p0; p1.u_max = Inf; p1.aw_mode = 'none';
o1 = simulate_saturation(p1, T, [0; 0; 0], opts);
o1_base = simulate_pid(p1, T, [0; 0; 0], opts);

tq = linspace(0, T, 30001).';
y1 = interp1(o1.t, o1.y, tq, 'pchip');
yb = interp1(o1_base.t, o1_base.y, tq, 'pchip');
u1 = interp1(o1.t, o1.u_act, tq, 'pchip');
ub = interp1(o1_base.t, o1_base.u, tq, 'pchip');
dy = max(abs(y1 - yb));
du = max(abs(u1 - ub));
fprintf('   max|y-y_base|=%.3e, max|u_act-u_base|=%.3e\n', dy, du);
assert(dy < 1e-8, 'G1 trajectory differs from the stage-1 baseline');
assert(du < 1e-8, 'G1 control force differs from the stage-1 baseline');
assert(abs(o1.metrics.rise_time - o1_base.metrics.rise_time) < 1e-6, ...
    'G1 rise time differs from baseline');
assert(abs(o1.metrics.overshoot_pct - o1_base.metrics.overshoot_pct) < 1e-6, ...
    'G1 overshoot differs from baseline');
assert(abs(o1.metrics.settling_time - o1_base.metrics.settling_time) < 1e-6, ...
    'G1 settling time differs from baseline');
assert(abs(o1.metrics.steady_error - o1_base.metrics.steady_error) < 1e-9, ...
    'G1 steady error differs from baseline');
assert(~o1.metrics.sat_occurred, 'G1 should never saturate');
assert(abs(max(abs(o1.u_act)) - 10) < 1e-9, 'G1 peak force should be 10 N');

%% 3) G2/G3 bounds and saturation occurrence
fprintf('\n3) G2/G3 actuator bounds and saturation\n');

p2 = p0; p2.u_max = u_lim; p2.aw_mode = 'none';
p3 = p0; p3.u_max = u_lim; p3.aw_mode = 'clamp';
o2 = simulate_saturation(p2, T, [0; 0; 0], opts);
o3 = simulate_saturation(p3, T, [0; 0; 0], opts);

groups = {o1, o2, o3};
group_names = {'G1 ideal', 'G2 sat no AW', 'G3 sat clamp'};
for ii = 1:numel(groups)
    o = groups{ii};
    assert(all(isfinite(o.t)) && all(diff(o.t) > 0), ...
        'time vector is not finite and monotonic');
    assert(all(isfinite(o.y)) && all(isfinite(o.u_des)) && ...
        all(isfinite(o.u_act)) && all(isfinite(o.x3)), ...
        'trajectory contains non-finite values');
end

for ii = 2:3
    o = groups{ii};
    s = o.metrics;
    assert(all(abs(o.u_act) <= u_lim + 1e-9), ...
        'applied force exceeds the actuator limit');
    assert(s.sat_occurred, 'saturated group did not saturate');
    assert(s.max_u_des > u_lim + 1, 'desired force did not exceed the limit');
    assert(abs(s.max_u_act - u_lim) < 1e-6, ...
        'applied force never reached the limit');
    assert(abs(s.u_des_initial - 10) < 1e-9, ...
        'initial desired force should be Kp*r');
    assert(abs(s.u_act_initial - u_lim) < 1e-9, ...
        'initial applied force should be clamped');
    assert(s.max_u_des >= s.u_des_initial - 1e-12, ...
        'peak desired force cannot be below its initial value');
    assert(s.sat_duration > 0, 'saturation duration should be positive');
    assert(isfinite(s.first_desat_time) && s.first_desat_time > 0, ...
        'first desaturation time should be finite and positive');
    fprintf('   %-14s max|u_des|=%.6f, max|u_act|=%.6f, sat=%.4f s\n', ...
        group_names{ii}, s.max_u_des, s.max_u_act, s.sat_duration);
end
fprintf('   note: u_des(0) and max|u_des| are recorded separately; ' );
fprintf('both equal 10 N here, which is not assumed.\n');

%% 4) Clamping invariant along the G3 trajectory
fprintf('\n4) G3 clamping invariant\n');

mask = (o3.u_des - o3.u_act).*o3.e > 1e-12;
assert(any(mask), 'G3 has no saturated-pushing samples to check');
max_dx3_traj = 0;
for k = find(mask).'
    [~, ~, dx3_k] = pid_control_sat(o3.x(k, :).', p3);
    max_dx3_traj = max(max_dx3_traj, abs(dx3_k));
end
fprintf('   saturated-pushing samples=%d, max|dx3|=%.3e\n', ...
    nnz(mask), max_dx3_traj);
assert(max_dx3_traj < 1e-9, 'clamping did not freeze dx3 along G3');

% Project-specific observation, not a general invariant of clamping:
% with x3(0)=0 and this parameter set, the first desaturation happens
% before the integrator has been allowed to move.
fprintf('   x3 at first desaturation: G2=%.6f, G3=%.6f\n', ...
    o2.metrics.x3_at_first_desat, o3.metrics.x3_at_first_desat);
assert(o2.metrics.x3_at_first_desat > 0.1, ...
    'G2 should have accumulated integral state before desaturating');
assert(abs(o3.metrics.x3_at_first_desat) < 1e-6, ...
    'observed G3 x3 at first desaturation should be ~0 for this setup');
assert(o3.metrics.first_desat_time < o2.metrics.first_desat_time - 0.1, ...
    'clamping should leave saturation earlier than no AW');
assert(o3.metrics.max_x3 < o2.metrics.max_x3 - 0.05, ...
    'clamping should reduce the integral peak');

%% 5) Steady state, force balance, integral state
fprintf('\n5) Steady state and force balance\n');

for ii = 1:3
    s = groups{ii}.metrics;
    fprintf(['   %-14s y_ss=%.10f, x3_ss=%.10f (expected %.10f), ' ...
        'u_post=%.6f, balance=%.3e\n'], ...
        group_names{ii}, s.steady_value, s.x3_ss, x3_expected, ...
        s.u_act_post, s.force_balance);
    assert(abs(s.steady_value - p0.r) < 1e-3, ...
        'steady position does not reach the reference');
    assert(abs(s.force_balance) < 1e-2, ...
        'steady-state force balance does not close');
    assert(abs(s.x3_ss - x3_expected) < 1e-3, ...
        'steady integral state does not match k*r/Ki');
end

%% 6) Solver-tolerance convergence for the clamped RHS
fprintf('\n6) Solver-tolerance convergence (G3)\n');

o3_lo = simulate_saturation(p3, T, [0; 0; 0], ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-9));
o3_hi = simulate_saturation(p3, T, [0; 0; 0], ...
    odeset('RelTol', 1e-10, 'AbsTol', 1e-13));
y_lo = interp1(o3_lo.t, o3_lo.y, tq, 'pchip');
y_hi = interp1(o3_hi.t, o3_hi.y, tq, 'pchip');
tol_diff = max(abs(y_lo - y_hi));
fprintf('   max|y(loose)-y(tight)|=%.3e (threshold 1e-4)\n', tol_diff);
assert(tol_diff < 1e-4, 'saturated/clamped RHS did not converge with tolerance');

%% 7) Performance ordering
fprintf('\n7) Performance ordering\n');

s1 = o1.metrics; s2 = o2.metrics; s3 = o3.metrics;
fprintf('   %-14s %9s %11s %11s %10s %10s\n', ...
    'case', 'rise(s)', 'overshoot%', 'settle2%(s)', 'IAE', 'max x3');
for ii = 1:3
    s = groups{ii}.metrics;
    fprintf('   %-14s %9.4f %11.4f %11.4f %10.4f %10.4f\n', ...
        group_names{ii}, s.rise_time, s.overshoot_pct, ...
        s.settling_time, s.IAE, s.max_x3);
end

% G2 is worse than the ideal baseline.
assert(s2.overshoot_pct > s1.overshoot_pct + 5, ...
    'G2 overshoot should exceed G1 by a clear margin');
assert(s2.settling_time > s1.settling_time + 0.3, ...
    'G2 settling should be slower than G1 by a clear margin');
assert(s2.IAE > s1.IAE + 0.05, ...
    'G2 IAE should exceed G1 by a clear margin');

% G3 improves on G2.
assert(s3.overshoot_pct < s2.overshoot_pct - 5, ...
    'G3 overshoot should be clearly below G2');
assert(s3.settling_time < s2.settling_time - 0.3, ...
    'G3 settling should be clearly faster than G2');
assert(s3.IAE < s2.IAE - 0.05, ...
    'G3 IAE should be clearly below G2');

% Project-specific observation at this parameter set: clamping also
% undershoots the ideal baseline's overshoot, because it freezes the
% integrator while the actuator is limited. This is not a general claim.
assert(s3.overshoot_pct < s1.overshoot_pct - 5, ...
    'observed G3 overshoot should be below the ideal baseline');

% Trade-off: report, but do not assert, the small differences vs G1.
fprintf(['   trade-off vs G1 (reported, not asserted): ' ...
    'rise %+.4f s, settle %+.4f s, IAE %+.4f\n'], ...
    s3.rise_time - s1.rise_time, s3.settling_time - s1.settling_time, ...
    s3.IAE - s1.IAE);

fprintf('\nAll stage-3 verification checks passed.\n');
