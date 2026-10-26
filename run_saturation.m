% RUN_SATURATION  Three-way actuator-saturation experiment (stage 3).
% Plant: m*x'' + c*x' + k*x = u_act, d(t) = 0.
% Controller: PID-15 (Kp=10, Ki=15, Kd=3).
%
% Groups:
%   G1  ideal actuator              u_max = Inf, aw = 'none'
%   G2  saturation, no anti-windup  u_max = 5 N, aw = 'none'
%   G3  saturation + clamping       u_max = 5 N, aw = 'clamp'
%
% All groups share the plant, reference, initial state, horizon, and solver
% tolerances. The only differences are the actuator limit and the integral
% law. u_max = 5 N is chosen because the steady holding force is k*r = 4 N
% (25% margin) and the unconstrained initial command is Kp*r = 10 N, so
% saturation necessarily occurs at t = 0 for G2/G3.
%
% u_des(0) is the initial command. max|u_des| is the peak over the whole run;
% the two are reported separately and need not be equal.

p0 = struct('m', 1, 'c', 1, 'k', 4, 'Kp', 10, 'Ki', 15, 'Kd', 3, 'r', 1);
T = 30;
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

names = {'G1 ideal', 'G2 sat no AW', 'G3 sat clamp'};
u_max_list = [Inf, 5, 5];
aw_list = {'none', 'none', 'clamp'};

n = numel(names);
results = repmat(struct('name', '', 'u_max', 0, 'aw_mode', '', 'out', []), 1, n);

for ii = 1:n
    p = p0;
    p.u_max = u_max_list(ii);
    p.aw_mode = aw_list{ii};
    results(ii).name = names{ii};
    results(ii).u_max = p.u_max;
    results(ii).aw_mode = p.aw_mode;
    results(ii).out = simulate_saturation(p, T, [0; 0; 0], opts);
end

u_lim = u_max_list(2);

fprintf('\n=== Stage 3: actuator saturation and integral windup ===\n');
fprintf('Plant m=%.3g, c=%.3g, k=%.3g; PID-15 (Kp=%.3g, Ki=%.3g, Kd=%.3g)\n', ...
    p0.m, p0.c, p0.k, p0.Kp, p0.Ki, p0.Kd);
fprintf('r=%.3g m; T=%.3g s; d(t)=0\n', p0.r, T);
fprintf('u_max = %.3g N (G2/G3); steady holding force k*r = %.3g N; ', ...
    u_lim, p0.k*p0.r);
fprintf('initial command Kp*r = %.3g N\n', p0.Kp*p0.r);

fprintf('\nPerformance (overshoot/settling are relative to each case''s own y_ss):\n');
fprintf('%-14s %9s %11s %11s %10s %10s\n', ...
    'case', 'rise(s)', 'overshoot%', 'settle2%(s)', 'ess(m)', 'y_ss');
for ii = 1:n
    s = results(ii).out.metrics;
    fprintf('%-14s %9.4f %11.4f %11.4f %10.2e %10.6f\n', ...
        results(ii).name, s.rise_time, s.overshoot_pct, ...
        s.settling_time, s.steady_error, s.steady_value);
end

fprintf('\nControl force and saturation:\n');
fprintf('%-14s %8s %10s %12s %12s %10s %11s %12s\n', ...
    'case', 'u_max', 'u_des(0)', 'max|u_des|', 'max|u_act|', ...
    'max_gap', 'sat_dur(s)', 'first_desat');
for ii = 1:n
    s = results(ii).out.metrics;
    if isnan(s.first_desat_time)
        fd = 'never';
    else
        fd = sprintf('%.4f', s.first_desat_time);
    end
    fprintf('%-14s %8.3g %10.4f %12.4f %12.4f %10.4f %11.4f %12s\n', ...
        results(ii).name, s.u_max, s.u_des_initial, s.max_u_des, ...
        s.max_u_act, s.max_gap, s.sat_duration, fd);
end

fprintf('\nIntegral state and tracking cost:\n');
fprintf('%-14s %10s %10s %14s %12s %12s %12s\n', ...
    'case', 'x3(0)', 'max x3', 'x3@first_desat', 'x3_ss', 'IAE', 'effort');
for ii = 1:n
    s = results(ii).out.metrics;
    fprintf('%-14s %10.4f %10.4f %14.4f %12.4f %12.4f %12.4f\n', ...
        results(ii).name, s.x3_initial, s.max_x3, s.x3_at_first_desat, ...
        s.x3_ss, s.IAE, s.control_effort);
end
fprintf('\nExpected steady integral state k*r/Ki = %.6f\n', p0.k*p0.r/p0.Ki);

fprintf('\nNote: u_des(0) is the initial command; max|u_des| is the peak over the\n');
fprintf('      whole run. They are recorded separately and need not be equal.\n');
fprintf('Note: local closed-loop poles describe the unsaturated region only.\n');
for ii = 1:n
    out = results(ii).out;
    fprintf('  %-14s local poles:', results(ii).name);
    for jj = 1:numel(out.local_roots)
        fprintf(' %8.4f%+8.4fi', real(out.local_roots(jj)), ...
            imag(out.local_roots(jj)));
    end
    fprintf('  Routh margin = %.4f\n', out.local_routh_margin);
end

figName = 'Stage 3: saturation and anti-windup';
old = findobj('Type', 'figure', 'Name', figName);
if ~isempty(old)
    close(old);
end
figure('Name', figName);
colors = lines(n);

subplot(4,1,1); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.y, 'LineWidth', 1.3, ...
        'Color', colors(ii,:));
end
yline(p0.r, 'k--');
ylabel('x (m)'); title('Position');
legend(names, 'Location', 'southeast');

subplot(4,1,2); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.e, 'LineWidth', 1.3, ...
        'Color', colors(ii,:));
end
yline(0, 'k--');
ylabel('e (m)'); title('Tracking error');

subplot(4,1,3); hold on; grid on;
h = gobjects(n,1);
for ii = 1:n
    h(ii) = plot(results(ii).out.t, results(ii).out.u_act, ...
        'LineWidth', 1.4, 'Color', colors(ii,:));
    plot(results(ii).out.t, results(ii).out.u_des, '--', ...
        'LineWidth', 1.0, 'Color', colors(ii,:), 'HandleVisibility', 'off');
end
yline(u_lim, 'k:');
yline(-u_lim, 'k:');
ylabel('u (N)');
title('Control force (solid = applied, dashed = desired, dotted = \pmu_{max})');
legend(h, names, 'Location', 'southeast');

subplot(4,1,4); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.x3, 'LineWidth', 1.3, ...
        'Color', colors(ii,:));
end
yline(p0.k*p0.r/p0.Ki, 'k--');
ylabel('x_3'); xlabel('t (s)');
title('Integral state (dashed = k*r/Ki)');
