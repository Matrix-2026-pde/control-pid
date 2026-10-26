function s = response_time_metrics(t, yn, band, vn)
%RESPONSE_TIME_METRICS  Crossings/extrema of a normalized numerical curve.
%   Internal metric helper: t is strictly increasing, yn is normalized by
%   the caller's accepted steady estimate. Optional vn is dy/dt divided by
%   that SAME estimate. This helper does not establish steady-state validity.
%   Without vn, use linear segments; with vn, use cubic Hermite segments.
%   All interior extrema split segments into monotone pieces, so two band
%   crossings within one original sample interval are not silently omitted.
%   Results describe this interpolant over the finite record, not the exact
%   ODE solution, hidden dynamics absent from the samples, or future behavior.

t = t(:); yn = yn(:);
h = diff(t);
n = numel(h);
if nargin < 4 || isempty(vn)
    co = [zeros(n, 2), diff(yn), yn(1:end-1)];
    s.time_method = 'linear';
else
    vn = vn(:);
    % Polynomial in local u=(time-t_i)/h_i, avoiding powers of small h.
    d = diff(yn);
    co = [-2*d + h.*(vn(1:end-1) + vn(2:end)), ...
        3*d - h.*(2*vn(1:end-1) + vn(2:end)), ...
        h.*vn(1:end-1), yn(1:end-1)];
    s.time_method = 'cubic_hermite';
end

[ext_i, ext_u] = stationary_points(co);
ext_t = t(ext_i) + h(ext_i).*ext_u;
ext_y = polynomial(co(ext_i,:), ext_u);
[knots, order] = sort([t; ext_t]);
values = [yn; ext_y];
values = values(order);
% Interval owning the piece to the RIGHT of each knot.
owners = [(1:n)'; n; ext_i];
owners = owners(order);

t10 = first_crossing(0.1, knots, values, owners, t, h, co);
t90 = first_crossing(0.9, knots, values, owners, t, h, co);
s.rise_time = t90 - t10;
[peak, peak_idx] = max(values); % first maximizer, including endpoints
s.peak_time = knots(peak_idx);
s.overshoot_pct = 100*max(0, peak - 1);

outside = find(abs(values - 1) > band, 1, 'last');
s.settling_time = NaN;
if isempty(outside)
    s.settling_time = t(1);
elseif outside < numel(knots)
    level = 1 + sign(values(outside) - 1)*band;
    s.settling_time = crossing(level, outside, knots, owners, t, h, co);
end
end

function q = first_crossing(level, knots, values, owners, t, h, co)
idx = find(values >= level, 1, 'first');
if isempty(idx)
    q = NaN;
elseif idx == 1
    q = knots(1);
else
    q = crossing(level, idx - 1, knots, owners, t, h, co);
end
end

function q = crossing(level, idx, knots, owners, t, h, co)
i = owners(idx);
left = (knots(idx) - t(i))/h(i);
right = (knots(idx + 1) - t(i))/h(i);
c = co(i,:);
% Values at original samples are used verbatim. Snap a root at a bracket
% endpoint only within polynomial evaluation roundoff, not a physical band.
tol = 16*eps(max([1, abs(c), abs(level)]));
if abs(polynomial(c, left) - level) <= tol
    u = left;
elseif abs(polynomial(c, right) - level) <= tol
    u = right;
else
    u = fzero(@(z) polynomial(c, z) - level, [left right]);
end
q = t(i) + h(i)*u;
end

function y = polynomial(c, u)
y = ((c(:,1).*u + c(:,2)).*u + c(:,3)).*u + c(:,4);
end

function [idx, u] = stationary_points(co)
% Stable quadratic roots of p'(u). Linear/constant derivatives are handled
% separately; keep only real roots strictly inside their sample interval.
a = 3*co(:,1); b = 2*co(:,2); c = co(:,3);
disc = b.^2 - 4*a.*c;
r = NaN(numel(a), 2);
quadratic = a ~= 0 & disc >= 0;
sgn = ones(size(b));
sgn(b < 0) = -1;
q = -0.5*(b(quadratic) + sgn(quadratic).*sqrt(disc(quadratic)));
r(quadratic,1) = q./a(quadratic);
r(quadratic,2) = c(quadratic)./q;
linear = a == 0 & b ~= 0;
r(linear,1) = -c(linear)./b(linear);
[idx, column] = find(r > 0 & r < 1);
idx = idx(:); column = column(:);
u = r(sub2ind(size(r), idx, column));
u = u(:);
% A repeated derivative root should appear only once in the knot list.
keep = column == 1 | r(idx,1) ~= u;
idx = idx(keep); u = u(keep);
end
