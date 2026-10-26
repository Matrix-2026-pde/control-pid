function dx = pid_rhs_dist(~, x, p)
%PID_RHS_DIST  Closed-loop PID + plant with a constant external force.
%   State: x(1) = position, x(2) = velocity, x(3) = integral of error.
%   p is a struct with fields m, c, k, Kp, Ki, Kd, r, d0.
%
%   Plant:      m*x'' + c*x' + k*x = u + d0
%   Controller: u = Kp*(r-x) + Ki*x(3) - Kd*x(2)
%
%   This RHS is used only for the post-disturbance segment [t_dist, T].
%   The pre-disturbance segment uses pid_rhs.m with d = 0, so the baseline
%   trajectory is reproduced exactly before the disturbance is applied.

e = p.r - x(1);
u = p.Kp*e + p.Ki*x(3) - p.Kd*x(2);

dx = [ ...
    x(2); ...
    (u + p.d0 - p.c*x(2) - p.k*x(1))/p.m; ...
    e ];
end
