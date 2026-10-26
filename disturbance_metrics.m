function s = disturbance_metrics(t, y, e, u, p, band)
%DISTURBANCE_METRICS  Metrics for the step-reference + disturbance experiment.
%
%   The record starts at t = 0 with the unit-step reference. A constant
%   disturbance switches on at t = p.t_dist. Recovery time is measured from
%   p.t_dist, NOT from t = 0; it is a different quantity from the original
%   step settling time.
%
%   s.pre_value          y at t_dist (last sample before the disturbance)
%   s.pre_error          |r - pre_value|
%   s.post_steady_value  mean y over the last 10% of the record
%   s.post_steady_error  |r - post_steady_value|
%   s.max_deviation      max |y - r| for t >= t_dist
%   s.max_excursion      max |y - pre_value| for t >= t_dist (the excursion
%                        caused by the disturbance, excluding the pre-existing
%                        P/PD steady error)
%   s.recovery_time      time after t_dist of the first sample following the
%                        last exit from the +-band around r; NaN if it never
%                        re-enters or is still outside at the final sample
%   s.u_pre              u at the last sample before t_dist
%   s.u_post             mean u over the last 10% of the record
%   s.delta_u            u_post - u_pre
%   s.max_u_post         max |u| for t >= t_dist

if nargin < 6
    band = 0.02;
end

t = t(:);
y = y(:);
e = e(:);
u = u(:);

post = t >= p.t_dist;
tail = t >= 0.9 * t(end);
i_pre = find(t <= p.t_dist, 1, 'last');

s.pre_value = y(i_pre);
s.pre_error = abs(p.r - s.pre_value);
s.post_steady_value = mean(y(tail));
s.post_steady_error = abs(p.r - s.post_steady_value);
s.max_deviation = max(abs(y(post) - p.r));
s.max_excursion = max(abs(y(post) - s.pre_value));

idx_post = find(post);
outside = find(abs(e(post)) > band * abs(p.r), 1, 'last');
if isempty(outside)
    s.recovery_time = 0;
elseif outside == numel(idx_post)
    s.recovery_time = NaN;
else
    s.recovery_time = t(idx_post(outside + 1)) - p.t_dist;
end

s.u_pre = u(i_pre);
s.u_post = mean(u(tail));
s.delta_u = s.u_post - s.u_pre;
s.max_u_post = max(abs(u(post)));
end
