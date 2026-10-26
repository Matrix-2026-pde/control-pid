function s = pid_metrics(t, y, r, band, dydt)
%PID_METRICS  Step-response metrics for one closed-loop simulation.
%
%   Rise time, overshoot, and settling time are measured relative to the
%   case's OWN estimated steady value y_ss, not relative to r. This matters for
%   P/PD, whose final value is below r: their transient shape is still well
%   defined, while the offset is reported separately as steady_error.
%
%   s.rise_time      first 10%-to-90% crossing of y_ss (s)
%   s.overshoot_pct  max(0, max(y/y_ss) - 1) * 100 on the interpolant
%   s.settling_time  final entry into the closed +-band around y_ss (s)
%   s.steady_value   last-10%-of-time sample mean (estimate, even if invalid)
%   s.steady_error   |r - steady_value| (also an estimate)
%   s.steady_state_valid/reason   sampled tail supports the estimate, or why not
%   s.settling_time_valid/reason  observed settling is supported, or why not
%   s.convergence    window bounds, sample counts, deviation, drift, budget
%   s.peak_time      first maximum of the normalized interpolated response
%   s.time_method    'linear' or 'cubic_hermite' (optional dydt supplied)
%   Crossings and extrema use the interpolant; band and validity are unchanged.
%   dydt must be the derivative of y at the same t, in y units per second.
%   An empty band retains the default 0.02. Legacy calls need no new input.
%
%   The last 20% is a guard window. Its maximum deviation from y_ss and
%   fitted linear drift over that window must EACH be <= band*abs(y_ss)/2.
%   This splits the specified error-band radius equally between observed
%   variation and trend. Each 10% half needs >=3 samples (two intervals).
%   No fixed duration in seconds or assumption y_ss=r is used.
%
%   A valid flag means stationarity at the requested band resolution in this
%   sampled record, not proof of asymptotic convergence or future settling.
%   If unconfirmed, all three normalized transient metrics are NaN. A zero
%   equilibrium can be stationary but has undefined relative step metrics.

if nargin < 4 || isempty(band)
    band = 0.02;
end
if nargin < 5
    dydt = [];
end

validateattributes(r, {'numeric'}, {'real', 'finite', 'scalar'}, mfilename, 'r');
validateattributes(band, {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, 'band');

s.steady_value = NaN;
s.steady_error = NaN;
s.rise_time = NaN;
s.overshoot_pct = NaN;
s.settling_time = NaN;
s.peak_time = NaN;
s.time_method = 'linear';
if ~isempty(dydt)
    validateattributes(dydt, {'numeric'}, {'real', 'finite', 'vector', ...
        'numel', numel(y)}, mfilename, 'dydt');
    dydt = dydt(:);
    s.time_method = 'cubic_hermite';
end
s.steady_state_valid = false;
s.steady_state_reason = 'invalid_record';
s.settling_time_valid = false;
s.settling_time_reason = 'invalid_record';
s.convergence = struct('window_start', NaN, 'tail_start', NaN, ...
    'deviation', NaN, 'drift', NaN, 'budget', NaN, ...
    'tail_samples', 0, 'preceding_samples', 0);

if ~isnumeric(t) || ~isnumeric(y) || ~isreal(t) || ~isreal(y) || ...
        ~isvector(t) || ~isvector(y) || isempty(t) || numel(t) ~= numel(y)
    return;
end
t = t(:);
y = y(:);
if any(~isfinite(t)) || any(~isfinite(y)) || any(diff(t) <= 0)
    return;
end

duration = t(end) - t(1);
tail_start = t(1) + 0.9*duration;
guard_start = t(1) + 0.8*duration;
tail = t >= tail_start;
guard = t >= guard_start;
y_ss = mean(y(tail));

s.steady_value = y_ss;
s.steady_error = abs(r - y_ss);
s.convergence.window_start = guard_start;
s.convergence.tail_start = tail_start;
s.convergence.tail_samples = nnz(tail);
s.convergence.preceding_samples = nnz(guard & ~tail);
s.convergence.budget = band*abs(y_ss)/2;
s.convergence.deviation = max(abs(y(guard) - y_ss));

if duration <= 0 || s.convergence.tail_samples < 3 || s.convergence.preceding_samples < 3
    s.steady_state_reason = 'insufficient_tail_samples';
    s.settling_time_reason = s.steady_state_reason;
    return;
end

% Normalize time so the fitted slope is a change in y over the entire guard
% duration, independent of time units and of the time origin. Center y too.
tau = (t(guard) - guard_start)/(t(end) - guard_start);
fit = [tau, ones(size(tau))] \ (y(guard) - y_ss);
s.convergence.drift = abs(fit(1));
variation_ok = s.convergence.deviation <= s.convergence.budget;
trend_ok = s.convergence.drift <= s.convergence.budget;
if ~variation_ok && ~trend_ok
    s.steady_state_reason = 'tail_variation_and_trend';
elseif ~variation_ok
    s.steady_state_reason = 'tail_variation';
elseif ~trend_ok
    s.steady_state_reason = 'tail_trend';
else
    s.steady_state_valid = true;
    s.steady_state_reason = 'tail_within_budget';
end
if ~s.steady_state_valid
    s.settling_time_reason = s.steady_state_reason;
    return;
end
if abs(y_ss) <= 1e-12*max(abs(y))
    s.settling_time_reason = 'zero_steady_value';
    return;
end

times = response_time_metrics(t, y/y_ss, band, dydt/y_ss);
s.rise_time = times.rise_time;
s.overshoot_pct = times.overshoot_pct;
s.peak_time = times.peak_time;
s.settling_time = times.settling_time;
if isnan(s.settling_time)
    s.settling_time_reason = 'record_ends_outside_band';
    return;
end
s.settling_time_valid = true;
s.settling_time_reason = 'observed_settling';
end
