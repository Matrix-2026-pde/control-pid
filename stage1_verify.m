% STAGE1_VERIFY  Independent checks of the ODE setup used in stage 1.
% 1) free response (u = 0) against the analytic underdamped solution
% 2) PID closed-loop steady state against the reference

m = 1; c = 1; k = 4;
wn = sqrt(k/m);
zeta = c/(2*sqrt(k*m));
wd = wn*sqrt(1 - zeta^2);

opts_free = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
rhs_free = @(t, x) [x(2); (-c*x(2) - k*x(1))/m];
[tf, xf] = ode45(rhs_free, [0 10], [1; 0], opts_free);
yf = xf(:,1);
yf_exact = exp(-zeta*wn*tf) .* (cos(wd*tf) + (zeta*wn/wd)*sin(wd*tf));
err_free = max(abs(yf - yf_exact));
fprintf('Stage 1 verify (1): free response max|error| = %.3e\n', err_free);
assert(err_free < 1e-6, 'free-response check failed');

Kp = 10; Ki = 15; Kd = 3; r = 1;
opts_pid = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
rhs_pid = @(t, x) [ ...
    x(2); ...
    (Kp*(r - x(1)) + Ki*x(3) - Kd*x(2) - c*x(2) - k*x(1))/m; ...
    r - x(1) ];
[tp, xp] = ode45(rhs_pid, [0 30], [0; 0; 0], opts_pid);
yp = xp(:,1);
y_ss = mean(yp(tp >= 27));
fprintf('Stage 1 verify (2): PID steady state mean y(t>27) = %.8f, target %.8f, diff %.3e\n', ...
    y_ss, r, abs(y_ss - r));
assert(abs(y_ss - r) < 1e-3, 'PID steady-state check failed');

fprintf('Stage 1 verification passed.\n');
