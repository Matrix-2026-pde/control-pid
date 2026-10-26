function dx = pid_rhs(~, x, p)
%PID_RHS  Closed-loop PID + mass-spring-damper right-hand side.
%   State: x(1) = position, x(2) = velocity, x(3) = integral of error.
%   p is a struct with fields m, c, k, Kp, Ki, Kd, r.
%
%   Plant:      m*x'' + c*x' + k*x = u
%   Controller: u = Kp*(r-x) + Ki*x(3) - Kd*x(2)
%
%   The derivative term acts on the measured velocity (-Kd*x(2)) rather than
%   on the error, so a step in r does not create a derivative impulse.

e = p.r - x(1);
u = p.Kp*e + p.Ki*x(3) - p.Kd*x(2);

dx = [ ...
    x(2); ...
    (u - p.c*x(2) - p.k*x(1))/p.m; ...
    e ];
end
