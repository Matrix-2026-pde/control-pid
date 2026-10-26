% VERIFY_MODEL  Independent checks for the closed-loop simulation.
%   1) free response vs the analytic underdamped solution
%   2) P-only steady state vs k/(k+Kp)
%   3) PID steady state vs the reference
%   4) solver-tolerance convergence
%   5a) Ki = 0: second-order reduced model and its roots
%   5b) Ki > 0: third-order poles and Routh margin

m = 1; c = 1; k = 4;
wn = sqrt(k/m);
zeta = c/(2*sqrt(k*m));
wd = wn*sqrt(1 - zeta^2);

% 1) analytic free response
opts_free = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
rhs_free = @(t, x) [x(2); (-c*x(2) - k*x(1))/m];
[tf, xf] = ode45(rhs_free, [0 10], [1; 0], opts_free);
yf = xf(:,1);
yf_exact = exp(-zeta*wn*tf) .* (cos(wd*tf) + (zeta*wn/wd)*sin(wd*tf));
err_free = max(abs(yf - yf_exact));
fprintf('1) free response: max|error| = %.3e (threshold 1e-6)\n', err_free);
assert(err_free < 1e-6, 'free-response check failed');

% 2) P-only steady state
p = struct('m', m, 'c', c, 'k', k, 'Kp', 10, 'Ki', 0, 'Kd', 0, 'r', 1);
out_p = simulate_pid(p, 30, [0; 0; 0], odeset('RelTol', 1e-9, 'AbsTol', 1e-11));
y_ss_p = mean(out_p.y(out_p.t >= 27));
y_pred_p = p.Kp/(p.k + p.Kp);
fprintf('2) P steady state: simulated %.8f, predicted %.8f, diff %.3e\n', ...
    y_ss_p, y_pred_p, abs(y_ss_p - y_pred_p));
assert(abs(y_ss_p - y_pred_p) < 1e-3, 'P steady-state check failed');

% 3) PID steady state
p = struct('m', m, 'c', c, 'k', k, 'Kp', 10, 'Ki', 15, 'Kd', 3, 'r', 1);
out_pid = simulate_pid(p, 30, [0; 0; 0], odeset('RelTol', 1e-9, 'AbsTol', 1e-11));
y_ss_pid = mean(out_pid.y(out_pid.t >= 27));
fprintf('3) PID steady state: simulated %.8f, target %.8f, diff %.3e\n', ...
    y_ss_pid, p.r, abs(y_ss_pid - p.r));
assert(abs(y_ss_pid - p.r) < 1e-3, 'PID steady-state check failed');

% 4) solver-tolerance convergence
out_lo = simulate_pid(p, 30, [0; 0; 0], odeset('RelTol', 1e-6, 'AbsTol', 1e-9));
out_hi = simulate_pid(p, 30, [0; 0; 0], odeset('RelTol', 1e-10, 'AbsTol', 1e-13));
tq = linspace(0, 30, 30001).';
y_lo = interp1(out_lo.t, out_lo.y, tq, 'pchip');
y_hi = interp1(out_hi.t, out_hi.y, tq, 'pchip');
max_diff = max(abs(y_lo - y_hi));
fprintf('4) tolerance convergence: max|y(loose)-y(tight)| = %.3e (threshold 1e-4)\n', max_diff);
assert(max_diff < 1e-4, 'tolerance convergence check failed');

% 5a) Ki = 0: second-order reduced model. The second-order overshoot formula
% is valid in this degenerate case, so check P and PD against it.
for Kd_list = [0, 3]
    p0 = struct('m', m, 'c', c, 'k', k, 'Kp', 10, 'Ki', 0, 'Kd', Kd_list, 'r', 1);
    out0 = simulate_pid(p0, 30, [0; 0; 0], odeset('RelTol', 1e-9, 'AbsTol', 1e-11));
    zeta0 = out0.zeta;
    Mp_formula = 100 * exp(-pi*zeta0/sqrt(1 - zeta0^2));
    Mp_measured = out0.metrics.overshoot_pct;
    fprintf('5a) Ki=0, Kd=%g: wn=%.4f rad/s, zeta=%.4f\n', Kd_list, out0.wn, zeta0);
    fprintf('    second-order formula Mp=%.3f%%, measured=%.3f%%, diff=%.3f pp\n', ...
        Mp_formula, Mp_measured, abs(Mp_formula - Mp_measured));
    assert(max(real(out0.reduced_roots)) < 0, 'Ki=0 reduced model is not stable');
    assert(zeta0 > 0 && zeta0 < 1, 'Ki=0 reduced model is not underdamped');
    assert(abs(Mp_formula - Mp_measured) < 1.0, ...
        'Ki=0 measured overshoot does not match the second-order formula');
end

% 5b) Ki > 0: third-order poles and Routh margin
for Ki_list = [5, 15, 30]
    p = struct('m', m, 'c', c, 'k', k, 'Kp', 10, 'Ki', Ki_list, 'Kd', 3, 'r', 1);
    out = simulate_pid(p, 1, [0; 0; 0]);
    fprintf('5b) Ki=%2d: max Re(root)=%+.6f, Routh margin=%.4f\n', ...
        Ki_list, max(real(out.roots)), out.routh_margin);
    assert(max(real(out.roots)) < 0, 'third-order poles are not stable');
    assert(out.routh_margin > 0, 'Routh margin is not positive');
end

fprintf('All verification checks passed.\n');
