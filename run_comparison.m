% RUN_COMPARISON  Compare six PID gain sets on the mass-spring-damper plant.
% All runs share the same plant, reference, horizon, and solver tolerances.
%
% Design of the comparison:
%   1 -> 2  isolates the effect of Kd (P vs PD)
%   1 -> 3  isolates the effect of Ki (P vs PI)
%   4 -> 5 -> 6  fixes Kp and Kd, increases Ki only
%
% Ki = 0 cases use the second-order reduced model (wn, zeta). Ki > 0 cases are
% third order: their poles and Routh margin are reported, and no single
% damping ratio is assigned.

p0 = struct('m', 1, 'c', 1, 'k', 4, 'r', 1);
T = 30;
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

names = {'P', 'PD', 'PI', 'PID-5', 'PID-15', 'PID-30'};
gains = [ ...
    10,  0, 0; ...
    10,  0, 3; ...
    10,  5, 0; ...
    10,  5, 3; ...
    10, 15, 3; ...
    10, 30, 3];

n = size(gains, 1);
results = repmat(struct('name', '', 'Kp', 0, 'Ki', 0, 'Kd', 0, 'out', []), 1, n);

for ii = 1:n
    p = p0;
    p.Kp = gains(ii,1);
    p.Ki = gains(ii,2);
    p.Kd = gains(ii,3);
    results(ii).name = names{ii};
    results(ii).Kp = p.Kp;
    results(ii).Ki = p.Ki;
    results(ii).Kd = p.Kd;
    results(ii).out = simulate_pid(p, T, [0; 0; 0], opts);
end

fprintf('\n%-8s %5s %5s %5s %10s %12s %12s %12s %10s %10s\n', ...
    'case', 'Kp', 'Ki', 'Kd', 'rise(s)', 'overshoot%', 'settle2%(s)', 'ess(m)', 'max|u|(N)', 'Routh');
for ii = 1:n
    s = results(ii).out.metrics;
    routh = results(ii).out.routh_margin;
    if isnan(routh)
        routh_str = 'n/a';
    else
        routh_str = sprintf('%.3f', routh);
    end
    fprintf('%-8s %5g %5g %5g %10.4f %12.3f %12.4f %12.2e %10.3f %10s\n', ...
        results(ii).name, results(ii).Kp, results(ii).Ki, results(ii).Kd, ...
        s.rise_time, s.overshoot_pct, s.settling_time, s.steady_error, ...
        s.max_u, routh_str);
end

fprintf('\nNote: overshoot%% and settle2%% are relative to each case''s own steady value;\n');
fprintf('      ess is relative to the reference r. For P/PD the final value is 0.7143, not 1.\n');

fprintf('\nPoles and interpretation:\n');
for ii = 1:n
    out = results(ii).out;
    if results(ii).Ki == 0
        fprintf('  %-8s reduced roots:', results(ii).name);
        rr = out.reduced_roots(:);
        for jj = 1:numel(rr)
            fprintf(' %8.4f%+8.4fi', real(rr(jj)), imag(rr(jj)));
        end
        fprintf('  (unused integrator pole s=0 omitted)\n');
        fprintf('           Ki=0: second-order reduced model, wn=%.4f rad/s, zeta=%.4f\n', ...
            out.wn, out.zeta);
    else
        fprintf('  %-8s cubic roots:', results(ii).name);
        rr = out.roots(:);
        for jj = 1:numel(rr)
            fprintf(' %8.4f%+8.4fi', real(rr(jj)), imag(rr(jj)));
        end
        fprintf('\n');
        fprintf('           Ki>0: third order; zero at s=%.4f; no single zeta. Routh margin = %.4f\n', ...
            out.zero, out.routh_margin);
    end
end

figName = 'PID comparison';
old = findobj('Type', 'figure', 'Name', figName);
if ~isempty(old)
    close(old);
end
figure('Name', figName);
subplot(3,1,1); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.y, 'LineWidth', 1.2);
end
yline(p0.r, 'k--');
ylabel('x (m)'); title('Position'); legend(names, 'Location', 'southeast');
subplot(3,1,2); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.e, 'LineWidth', 1.2);
end
yline(0, 'k--');
ylabel('e (m)'); title('Tracking error');
subplot(3,1,3); hold on; grid on;
for ii = 1:n
    plot(results(ii).out.t, results(ii).out.u, 'LineWidth', 1.2);
end
ylabel('u (N)'); xlabel('t (s)'); title('Control force');
