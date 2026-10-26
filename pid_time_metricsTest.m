classdef pid_time_metricsTest < matlab.unittest.TestCase
    % Independent analytic references and explicitly synthetic trajectories.
    properties (TestParameter)
        refine = struct('one', 1, 'four', 4, 'twenty', 20)
        validRefine = struct('four', 4, 'twenty', 20)
        actuator = struct('limited', 'none', 'clamped', 'clamp')
    end

    methods (Test)
        function baselineMatchesMatrixExponential(testCase, validRefine)
            out = simulate_pid(parameters(), 30, [], options(validRefine));
            expected = linear_reference();
            testCase.verifyTrue(out.metrics.settling_time_valid);
            testCase.verifyEqual(metric_vector(out.metrics), expected, 'AbsTol', 1e-6);
            testCase.verifyEqual(out.metrics.time_method, 'cubic_hermite');
        end

        function rawInterpolantMatchesIndependentReference(testCase, refine)
            % Diagnostic only: this helper does not certify steady state.
            out = simulate_pid(parameters(), 30, [], options(refine));
            s = response_time_metrics(out.t, out.y, 0.02, out.x(:,2));
            testCase.verifyEqual(metric_vector(s), linear_reference(), 'AbsTol', 1e-6);
        end

        function refineOneDoesNotBypassValidity(testCase)
            out = simulate_pid(parameters(), 30, [], options(1));
            testCase.verifyFalse(out.metrics.steady_state_valid);
            testCase.verifyEqual(out.metrics.steady_state_reason, 'insufficient_tail_samples');
            testCase.verifyTrue(all(isnan(metric_vector(out.metrics))));
        end

        function saturatedRunMatchesPiecewiseExponential(testCase, actuator)
            p = parameters();
            p.u_max = 5;
            p.aw_mode = actuator;
            out = simulate_saturation(p, 30);
            [expected, exit_time] = saturated_reference(actuator);
            testCase.verifyTrue(out.metrics.settling_time_valid);
            testCase.verifyEqual(metric_vector(out.metrics), expected, 'AbsTol', 2e-6);
            testCase.verifyEqual(out.metrics.first_desat_time, exit_time, 'AbsTol', 1e-6);
            testCase.verifyEqual(out.metrics.sat_duration, exit_time, 'AbsTol', 1e-6);
        end

        function linearCrossingsLieBetweenSamples(testCase)
            t = (0:100).';
            y = min(t/2, 1);
            s = pid_metrics(t, y, 1);
            testCase.verifyEqual(s.rise_time, 1.6, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.settling_time, 1.96, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.peak_time, 2, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.time_method, 'linear');
        end

        function hermiteReproducesSmoothCubic(testCase)
            t = [0; 2; (3:100).'];
            y = [0; ones(numel(t)-1, 1)];
            s = pid_metrics(t, y, 1, [], zeros(size(t)));
            % y=3u^2-2u^3 on [0,2]; inverse from the triple-angle identity.
            inverse = @(level) 0.5 - sin(asin(1 - 2*level)/3);
            testCase.verifyEqual(s.rise_time, 2*(inverse(.9)-inverse(.1)), 'AbsTol', 1e-12);
            testCase.verifyEqual(s.settling_time, 2*inverse(.98), 'AbsTol', 1e-12);
            testCase.verifyEqual(s.peak_time, 2, 'AbsTol', 1e-12);
        end

        function settlingUsesLastOfManyCrossings(testCase)
            t = (0:100).';
            y = ones(size(t));
            y(1:6) = [0; 1; 1.1; 1; .9; 1];
            s = pid_metrics(t, y, 1);
            testCase.verifyTrue(s.settling_time_valid);
            testCase.verifyEqual(s.settling_time, 4.8, 'AbsTol', 1e-12);
        end

        function laterDepartureOverridesEarlyEntry(testCase)
            t = (0:100).';
            y = ones(size(t));
            y(71) = 1.1;
            s = pid_metrics(t, y, 1);
            testCase.verifyEqual(s.settling_time, 70.8, 'AbsTol', 1e-12);
        end

        function finalOutsideRecordCannotReportSettling(testCase)
            t = (0:100).';
            y = ones(size(t));
            y(end) = 1.1;
            s = pid_metrics(t, y, 1);
            testCase.verifyFalse(s.settling_time_valid);
            testCase.verifyTrue(isnan(s.settling_time));
        end

        function interiorExtremumRevealsHiddenBandExit(testCase)
            % Both samples are inside: p(u)=1+u-u^2 exceeds the upper band.
            s = response_time_metrics([0;1], [1;1], .02, [1;-1]);
            testCase.verifyEqual(s.settling_time, (1+sqrt(.92))/2, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.peak_time, .5, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.overshoot_pct, 25, 'AbsTol', 1e-12);
        end

        function boundaryTouchIsNotAnExit(testCase)
            % p(u)=1+.5u-.5u^2 touches 1.125, with no strict excursion.
            s = response_time_metrics([0;1], [1;1], .125, [.5;-.5]);
            testCase.verifyEqual(s.settling_time, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.peak_time, .5, 'AbsTol', 1e-12);
        end

        function stationaryDerivativeWithDoubleRoot(testCase)
            % p(u)=(u-.5)^3+.5 has a double stationary root at .5.
            s = response_time_metrics([0;1], [.375;.625], .02, [.75;.75]);
            testCase.verifyTrue(isnan(s.settling_time));
            testCase.verifyEqual(s.peak_time, 1, 'AbsTol', 1e-12);
        end

        function offsetAndNegativeSteadyValuesKeepOwnNormalization(testCase)
            t = (0:100).';
            y = -.7*min(t/2, 1);
            s = pid_metrics(t, y, 1);
            testCase.verifyTrue(s.steady_state_valid);
            testCase.verifyEqual(s.steady_error, 1.7, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.rise_time, 1.6, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.settling_time, 1.96, 'AbsTol', 1e-12);
        end

        function saturationCrossesBothLimitsInOneInterval(testCase)
            t = (0:100).';
            u = zeros(size(t));
            u(1:2) = [2;-2];
            p = parameters();
            s = saturation_metrics(t, ones(size(t)), zeros(size(t)), ...
                u, max(-1, min(1, u)), t, p, 1);
            limit = 1 + 1e-9;
            exit_time = (2-limit)/4;
            expected_duration = 2*exit_time + (2-limit)/2;
            testCase.verifyEqual(s.first_desat_time, exit_time, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.sat_duration, expected_duration, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.x3_at_first_desat, exit_time, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.u_des_at_first_desat, limit, 'AbsTol', 1e-12);
            testCase.verifyEqual(s.sat_episodes, 2);
        end

        function invalidVelocityIsRejected(testCase)
            t = (0:100).';
            testCase.verifyError(@()pid_metrics(t, ones(size(t)), 1, [], [0;NaN]), ...
                'MATLAB:pid_metrics:expectedFinite');
        end
    end
end

function p = parameters()
p = struct('m',1,'c',1,'k',4,'Kp',10,'Ki',15,'Kd',3,'r',1);
end

function opts = options(refine)
opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'Refine',refine);
end

function v = metric_vector(s)
v = [s.rise_time, s.settling_time, s.peak_time];
end

function v = linear_reference()
% Independent augmented matrix exponential, no ODE or metric implementation.
A = [0 1 0 0; -14 -4 15 10; -1 0 0 1; 0 0 0 0];
state = @(t) expm(A*t)*[0;0;0;1];
y = @(t) [1 0 0 0]*state(t);
velocity = @(t) [0 1 0 0]*state(t);
v = [fzero(@(t)y(t)-.9,[.3 .8])-fzero(@(t)y(t)-.1,[0 .3]), ...
    fzero(@(t)y(t)-.98,[2.5 2.8]), fzero(velocity,[.7 1.5])];
check_reference_tail(A, 0, [0;0;0;1], y, velocity, v);
end

function [v, exit_time] = saturated_reference(mode)
% Independent piecewise affine system: one positive saturation episode,
% then the unsaturated closed loop. No call to pid_rhs or pid_metrics.
A = [0 1 0 0; -14 -4 15 10; -1 0 0 1; 0 0 0 0];
B = [0 1 0 0; -4 -1 0 5; -1 0 0 1; 0 0 0 0];
if strcmp(mode, 'clamp')
    B(3,:) = 0;
end
w0 = [0;0;0;1];
td = fzero(@(t)[-10 -3 15 10]*expm(B*t)*w0-5,[0 1]);
wd = expm(B*td)*w0;
state = @(t) piecewise_state(t,A,B,td,wd,w0);
y = @(t) [1 0 0 0]*state(t);
velocity = @(t) [0 1 0 0]*state(t);
settle_bracket = [3.4 4];
settle_level = 1.02;
if strcmp(mode, 'clamp')
    settle_bracket = [2.5 3];
    settle_level = .98;
end
v = [fzero(@(t)y(t)-.9,[.5 1])-fzero(@(t)y(t)-.1,[0 .3]), ...
    fzero(@(t)y(t)-settle_level,settle_bracket), fzero(velocity,[1 1.5])];
% Physical switching is at 5; the reported metric retains 5+1e-9.
exit_time = fzero(@(t)[-10 -3 15 10]*expm(B*t)*w0-(5+1e-9),[0 td]);
check_reference_tail(A, td, wd, y, velocity, v);
% Independently check the one-saturation-episode assumption on a fine grid.
post_times = linspace(td+1e-6, 10, 1001);
post_u = arrayfun(@(t)[-10 -3 15 10]*state(t), post_times);
assert(all(abs(post_u) < 5), 'Reference requires another saturation segment.');
end

function w = piecewise_state(t,A,B,td,wd,w0)
if t <= td
    w = expm(B*t)*w0;
else
    w = expm(A*(t-td))*wd;
end
end

function check_reference_tail(A, td, wd, y, velocity, metrics)
% Locate later extrema independently using the exponential's derivative.
% Brackets are 0.01 s, far below the ~2 s oscillation period; no pid metric
% code is used. A modal envelope bounds ALL later times beyond 10 s.
grid = linspace(metrics(2)+1e-5, 10, 1001);
v = arrayfun(velocity, grid);
idx = find(v(1:end-1).*v(2:end) < 0);
extrema = arrayfun(@(i)fzero(velocity, grid([i i+1])), idx);
assert(all(abs(arrayfun(y, extrema)-1) < .02), 'A later extremum leaves the band.');
[V,D] = eig(A(1:3,1:3));
amplitude = V\(wd(1:3)-[1;0;4/15]);
envelope = sum(abs(V(1,:).'.*amplitude).*exp(real(diag(D))*(10-td)));
assert(envelope < .02, 'Late-time envelope is too large.');
end
