% RUN_DISTURBANCE  Step reference followed by a persistent external force.
% Plant: m*x'' + c*x' + k*x = u + d(t)
%   d(t) = 0        for 0 <= t < 10 s
%   d(t) = -4 N     for t >= 10 s
%
% P, PD, and PID-15 use the same plant, reference, disturbance, and initial
% condition. PID-15 is the representative PID case from the baseline (it had
% the shortest settling time there). The original six-case baseline in
% run_comparison.m is not modified.
%
% Metrics:
%   pre_error   |r - y(t=10^-)| (last sample before the disturbance)
%   post_ess    |r - mean(y over the last 10% of the record)|
%   max_dev     max |y - r| for t >= 10 (distance from the target)
%   max_exc     max |y - y(10^-)| for t >= 10 (excursion caused by the
%               disturbance, excluding the pre-existing P/PD steady error)
%   recovery    time after t=10 until |e| stays inside 2% of |r|; NaN if never
%   u_pre       u at t = 10^-; u_post = mean u over the last 10%
%   delta_u     u_post - u_pre
%
% "recovery" is measured from the disturbance onset at t=10; it is NOT the
% original step settling time. P/PD have a steady offset of 0.571 m, so their
% recovery is undefined (NaN), not merely slow.

p0 = struct('m', 1, 'c', 1, 'k', 4, 'r', 1, 't_dist', 10, 'd0', -4);
T = 30;
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

names = {'P', 'PD', 'PID-15'};
gains = [ ...
    10,  0, 0; ...
    10,  0, 3; ...
    10, 15, 3];

n = size(gains, 1);
results = repmat(struct('name', '', 'Kp', 0, 'Ki', 0, 'Kd', 0, ...
    'out', [], 'base_diff', NaN), 1, n);

for ii = 1:n
    p = p0;
    p.Kp = gains(ii, 1);
    p.Ki = gains(ii, 2);
    p.Kd = gains(ii, 3);

    results(ii).name = names{ii};
    results(ii).Kp = p.Kp;
    results(ii).Ki = p.Ki;
    results(ii).Kd = p.Kd;
    results(ii).out = simulate_disturbance(p, T, [0; 0; 0], opts);

    % Independent baseline over the same horizon, used only to check the
    % pre-disturbance segment.
    base = simulate_pid(p, T, [0; 0; 0], opts);
    tq = linspace(0, p.t_dist - 1e-3, 10001).';
    yq = interp1(results(ii).out.t, results(ii).out.y, tq, 'pchip');
    yq_base = interp1(base.t, base.y, tq, 'pchip');
    results(ii).base_diff = max(abs(yq - yq_base));
end

fprintf('\n=== Disturbance experiment: d(t)=0 for t<10, d0=-4 N for t>=10 ===\n');
fprintf('Plant m=%.1f, c=%.1f, k=%.1f; reference r=%.1f; T=%.0f s\n', ...
    p0.m, p0.c, p0.k, p0.r, T);
fprintf('\n%-8s %10s %10s %9s %9s %9s %8s %8s %8s %8s %10s\n', ...
    'case', 'pre_err', 'post_ess', 'max_dev', 'max_exc', 'recovery', ...
    'u_pre', 'u_post', 'delta_u', 'max|u|', 'pre_diff');
for ii = 1:n
    s = results(ii).out.metrics;
    if isnan(s.recovery_time)
        rec_str = 'never';
    else
        rec_str = sprintf('%.4f', s.recovery_time);
    end
    fprintf('%-8s %10.2e %10.2e %9.4f %9.4f %9s %8.3f %8.3f %8.3f %8.3f %10.2e\n', ...
        results(ii).name, s.pre_error, s.post_steady_error, s.max_deviation, ...
        s.max_excursion, rec_str, s.u_pre, s.u_post, s.delta_u, ...
        s.max_u_post, results(ii).base_diff);
end

fprintf('\nNote: recovery is timed from the disturbance at t=10, not from t=0.\n');
fprintf('      P/PD cannot recover to the 2%% band: their post-disturbance\n');
fprintf('      steady error is 0.5714 m, so recovery is reported as "never".\n');

fprintf('\nSteady-state force balance (u_post + d0 - k*y_post):\n');
for ii = 1:n
    s = results(ii).out.metrics;
    fprintf('  %-8s %+.3e N\n', results(ii).name, ...
        s.u_post + p0.d0 - p0.k * s.post_steady_value);
end

figName = 'Disturbance response';
old = findobj('Type', 'figure', 'Name', figName);
if ~isempty(old)
    close(old);
end
figure('Name', figName);
h = gobjects(n, 1);

subplot(4,1,1); hold on; grid on;
for ii = 1:n
    h(ii) = plot(results(ii).out.t, results(ii).out.y, 'LineWidth', 1.2);
end
yline(p0.r, 'k--');
xline(p0.t_dist, 'k:');
ylabel('x (m)'); title('Position');
legend(h, names, 'Location', 'southeast');

subplot(4,1,2); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.e, 'LineWidth', 1.2);
end
yline(0, 'k--'); xline(p0.t_dist, 'k:');
ylabel('e (m)'); title('Tracking error');

subplot(4,1,3); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.u, 'LineWidth', 1.2);
end
xline(p0.t_dist, 'k:');
ylabel('u (N)'); title('Control force');

subplot(4,1,4); hold on; grid on;
plot(results(1).out.t, results(1).out.d, 'k', 'LineWidth', 1.5);
xline(p0.t_dist, 'k:');
ylabel('d (N)'); xlabel('t (s)'); title('External disturbance');
