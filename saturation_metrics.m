function s = saturation_metrics(t, y, e, u_des, u_act, x3, p, u_max, band, dydt)
%SATURATION_METRICS  Step and windup metrics for one saturated run.
%
%   Step metrics (rise, overshoot, settling) reuse pid_metrics.m. They are
%   normalized to the case's own final value y_ss; for this experiment all
%   three groups should reach y_ss ~= r, and verify_saturation.m checks that.
%   Validity flags, reasons, and convergence diagnostics are forwarded from
%   pid_metrics; they describe position only, not the full controller state.
%
%   s.u_des_initial      u_des at t = 0 (the initial proportional kick)
%   s.max_u_des          max |u_des| over the whole run (NOT the initial value)
%   s.max_u_act          max |u_act| over the whole run
%   s.max_gap            max(u_des - u_act), the positive saturation gap
%   s.sat_occurred       true if |u_des| exceeded u_max at any sample
%   s.sat_duration       linear-interpolant time with |u_des| > u_max+1e-9
%   s.first_desat_time   first interpolated exit from that saturation band
%                        (NaN if it never does)
%   s.x3_at_first_desat  integral state at that first exit (NaN if never)
%   s.max_x3             peak integral state
%   s.x3_ss              mean integral state over the last 10% of the record
%   s.x3_ss_expected     k*r/Ki, the integral state required at steady state
%   s.IAE, s.ISE         integral absolute / squared error
%   s.control_effort     integral of u_act^2
%   s.force_balance      u_act_post - k*y_ss (should be ~0 at steady state)
%   Optional dydt forwards position velocity for Hermite step metrics.
%   Force and integral-state interpolation remains linear across switching.

if nargin < 9 || isempty(band)
    band = 0.02;
end
if nargin < 10
    dydt = [];
end

t = t(:); y = y(:); e = e(:);
u_des = u_des(:); u_act = u_act(:); x3 = x3(:);

tail = t >= 0.9*t(end);
s = pid_metrics(t, y, p.r, band, dydt);
y_ss = s.steady_value;
s.y_ss_relative_error = abs(y_ss - p.r)/max(abs(p.r), eps);

s.u_max = u_max;
s.u_des_initial = u_des(1);
s.u_act_initial = u_act(1);
s.max_u_des = max(abs(u_des));
s.max_u_act = max(abs(u_act));
s.max_u_des_positive = max(u_des);
s.max_u_act_positive = max(u_act);
s.max_gap = max(u_des - u_act);
s.max_abs_gap = max(abs(u_des - u_act));

[s.sat_duration, s.first_desat_time, s.sat_episodes] = ...
    saturation_times(t, u_des, u_max + 1e-9);
s.saturation_time_method = 'linear';
s.sat_occurred = any(abs(u_des) > u_max + 1e-9);
s.sat_fraction = s.sat_duration/(t(end) - t(1));

s.x3_at_first_desat = NaN;
s.u_des_at_first_desat = NaN;
if isfinite(s.first_desat_time)
    s.x3_at_first_desat = interp1(t, x3, s.first_desat_time);
    s.u_des_at_first_desat = interp1(t, u_des, s.first_desat_time);
end

s.x3_initial = x3(1);
s.max_x3 = max(x3);
s.min_x3 = min(x3);
s.x3_ss = mean(x3(tail));
if p.Ki > 0
    s.x3_ss_expected = p.k*p.r/p.Ki;
    s.x3_ss_error = abs(s.x3_ss - s.x3_ss_expected);
else
    s.x3_ss_expected = NaN;
    s.x3_ss_error = NaN;
end

s.IAE = trapz(t, abs(e));
s.ISE = trapz(t, e.^2);
s.control_effort = trapz(t, u_act.^2);
s.u_act_post = mean(u_act(tail));
s.force_balance = s.u_act_post - p.k*y_ss;
end

function [duration, first_exit, episodes] = saturation_times(t, u, limit)
% Integrate above +limit and below -limit separately, including an interval
% that crosses BOTH limits. Keep the existing 1e-9 force tolerance.
duration = 0;
first_exit = NaN;
episodes = double(abs(u(1)) > limit);
h = diff(t);
for sign_u = [-1 1]
    z = sign_u*u;
    mask = z > limit;
    starts = mask(1:end-1); stops = mask(2:end);
    widths = h.*double(starts & stops);
    cross = find(starts ~= stops);
    fraction = (limit - z(cross))./(z(cross + 1) - z(cross));
    widths(cross) = h(cross).*( ...
        starts(cross).*fraction + stops(cross).*(1 - fraction));
    duration = duration + sum(widths);
    episodes = episodes + nnz(~starts & stops);
    exits = cross(starts(cross));
    fractions_exit = fraction(starts(cross));
    if ~isempty(exits)
        candidate = t(exits(1)) + (t(exits(1)+1) - t(exits(1)))*fractions_exit(1);
        if isnan(first_exit) || candidate < first_exit
            first_exit = candidate;
        end
    end
end
end
