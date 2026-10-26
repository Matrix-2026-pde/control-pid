% STAGE1_PID_SIM  One closed-loop PID simulation (minimal runnable version).
% Plant:       m*x'' + c*x' + k*x = u
% Controller:  u = Kp*(r-x) + Ki*integral(r-x) - Kd*xdot
% State:       [x; xdot; integral of error]
%
% This file intentionally keeps everything in one script. Stage 2 factors the
% same equations into pid_rhs.m / simulate_pid.m / pid_metrics.m.

m = 1; c = 1; k = 4;
Kp = 10; Ki = 15; Kd = 3;
r = 1; T = 30;
x0 = [0; 0; 0];
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

rhs = @(t, x) [ ...
    x(2); ...
    (Kp*(r - x(1)) + Ki*x(3) - Kd*x(2) - c*x(2) - k*x(1))/m; ...
    r - x(1) ];

[t, x] = ode45(rhs, [0 T], x0, opts);
y = x(:,1);
u = Kp*(r - y) + Ki*x(:,3) - Kd*x(:,2);
e = r - y;

y_ss = mean(y(t >= 0.9*T));
yn = y / y_ss;
tr = local_rise_time(t, yn);
Mp = 100 * max(0, max(yn) - 1);
ts = local_settling_time(t, yn, 0.02);
ess = abs(r - y_ss);
umax = max(abs(u));

fprintf('Stage 1: one closed-loop run (PID-15)\n');
fprintf('  m=%.3g, c=%.3g, k=%.3g, Kp=%.3g, Ki=%.3g, Kd=%.3g\n', m, c, k, Kp, Ki, Kd);
fprintf('  final y      = %.8f m\n', y(end));
fprintf('  steady error = %.3e m\n', ess);
fprintf('  rise time    = %.6f s\n', tr);
fprintf('  overshoot    = %.4f %%\n', Mp);
fprintf('  settling 2%% = %.6f s\n', ts);
fprintf('  max |u|      = %.4f N\n', umax);
fprintf('  samples      = %d\n', numel(t));

figName = 'Stage 1: one PID run';
old = findobj('Type', 'figure', 'Name', figName);
if ~isempty(old)
    close(old);
end
figure('Name', figName);
subplot(3,1,1);
plot(t, y, 'LineWidth', 1.5); hold on;
yline(r, 'k--');
ylabel('x (m)'); title('Position'); grid on;
subplot(3,1,2);
plot(t, e, 'LineWidth', 1.5);
ylabel('e (m)'); title('Tracking error'); grid on;
subplot(3,1,3);
plot(t, u, 'LineWidth', 1.5);
ylabel('u (N)'); xlabel('t (s)'); title('Control force'); grid on;

function tr = local_rise_time(t, yn)
% First 10%-to-90% crossing of the normalized response.
i10 = find(yn >= 0.1, 1, 'first');
i90 = find(yn >= 0.9, 1, 'first');
if isempty(i10) || isempty(i90)
    tr = NaN;
else
    tr = t(i90) - t(i10);
end
end

function ts = local_settling_time(t, yn, band)
% Last time the normalized response leaves the +-band around 1.
outside = find(abs(yn - 1) > band, 1, 'last');
if isempty(outside)
    ts = t(1);
elseif outside == numel(t)
    ts = NaN;
else
    ts = t(outside + 1);
end
end
