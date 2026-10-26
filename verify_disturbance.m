% VERIFY_DISTURBANCE  Independent checks for the disturbance experiment.
%   1) pre-disturbance trajectory matches the baseline over [0, t_dist)
%   2) d(t) = 0 before t_dist and d0 after
%   3) P/PD post-disturbance steady state matches (Kp*r + d0)/(k+Kp)
%   4) PID post-disturbance steady state matches r
%   5) control-force balance u_post + d0 - k*y_post ~ 0
%   6) P/PD recovery is NaN; PID recovery is finite
%   7) PID Routh margin is positive

p0 = struct('m', 1, 'c', 1, 'k', 4, 'r', 1, 't_dist', 10, 'd0', -4);
T = 30;
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

names = {'P', 'PD', 'PID-15'};
gains = [10, 0, 0; 10, 0, 3; 10, 15, 3];

for ii = 1:size(gains, 1)
    p = p0;
    p.Kp = gains(ii, 1);
    p.Ki = gains(ii, 2);
    p.Kd = gains(ii, 3);
    out = simulate_disturbance(p, T, [0; 0; 0], opts);
    base = simulate_pid(p, T, [0; 0; 0], opts);
    s = out.metrics;

    % 1) pre-disturbance trajectory
    tq = linspace(0, p.t_dist - 1e-3, 10001).';
    yq = interp1(out.t, out.y, tq, 'pchip');
    yq_base = interp1(base.t, base.y, tq, 'pchip');
    pre_diff = max(abs(yq - yq_base));
    fprintf('1) %-8s pre-disturbance max|y-y_base| = %.3e (threshold 1e-6)\n', ...
        names{ii}, pre_diff);
    assert(pre_diff < 1e-6, 'pre-disturbance trajectory differs from baseline');

    % 2) disturbance is zero before and d0 after
    pre_d = out.d(out.t < p.t_dist - 1e-9);
    post_d = out.d(out.t >= p.t_dist + 1e-9);
    assert(all(pre_d == 0), 'disturbance is nonzero before t_dist');
    assert(all(abs(post_d - p.d0) < 1e-12), 'disturbance is not d0 after t_dist');

    % 3/4) steady state and recovery
    if p.Ki == 0
        y_pred = (p.Kp*p.r + p.d0)/(p.k + p.Kp);
        fprintf('2) %-8s post y_ss = %.8f, formula = %.8f, diff = %.3e\n', ...
            names{ii}, s.post_steady_value, y_pred, ...
            abs(s.post_steady_value - y_pred));
        assert(abs(s.post_steady_value - y_pred) < 1e-3, ...
            'P/PD post-disturbance steady state does not match formula');
        assert(isnan(s.recovery_time), 'P/PD should not recover to the band');
    else
        fprintf('3) %-8s post y_ss = %.8f, target = %.8f, diff = %.3e\n', ...
            names{ii}, s.post_steady_value, p.r, ...
            abs(s.post_steady_value - p.r));
        assert(abs(s.post_steady_value - p.r) < 1e-3, ...
            'PID post-disturbance steady state does not match r');
        assert(isfinite(s.recovery_time) && s.recovery_time >= 0, ...
            'PID recovery time is not finite');
        routh = (p.c + p.Kd)*(p.k + p.Kp) - p.m*p.Ki;
        fprintf('4) %-8s Routh margin = %.1f (> 0)\n', names{ii}, routh);
        assert(routh > 0, 'PID Routh margin is not positive');
    end

    % 5) quasi-static force balance
    balance = s.u_post + p.d0 - p.k * s.post_steady_value;
    fprintf('5) %-8s u_post + d0 - k*y_ss = %+.3e N (threshold 1e-2)\n', ...
        names{ii}, balance);
    assert(abs(balance) < 1e-2, 'steady-state force balance does not close');
end

fprintf('All disturbance verification checks passed.\n');
