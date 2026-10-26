classdef pid_metricsTest < matlab.unittest.TestCase
    % Real simulations and independent synthetic records for metric validity.
    properties (TestParameter)
        gain = struct('P', [10 0 0], 'PD', [10 0 3], 'PI', [10 5 0], ...
            'PID5', [10 5 3], 'PID15', [10 15 3], 'PID30', [10 30 3])
        actuator = struct('ideal', {{Inf, 'none'}}, ...
            'limited', {{5, 'none'}}, 'clamped', {{5, 'clamp'}})
        disturbanceGain = struct('P', [10 0 0], 'PD', [10 0 3], 'PID15', [10 15 3])
        deviationCase = struct('atBudget', [0.0625 1], 'aboveBudget', [0.0634765625 0])
        trendCase = struct('belowBudget', [0.0028125 1], 'aboveBudget', [0.0034375 0])
        sparseRecord = struct('onePoint', [0 1], 'fewTailPoints', [0 1; 8 1; 9 1; 10 1])
    end

    methods (Test)
        function baselineSupportsSteadyEstimate(testCase, gain)
            p = parameters(gain);
            out = simulate_pid(p, 30);
            expected = p.r;
            expected = expected - (gain(2) == 0)*p.k*p.r/(p.k + p.Kp);
            testCase.verifyTrue(out.metrics.steady_state_valid);
            testCase.verifyTrue(out.metrics.settling_time_valid);
            testCase.verifyEqual(out.metrics.steady_value, expected, 'AbsTol', 1e-3);
            testCase.verifyTrue(isfinite(out.metrics.rise_time));
            testCase.verifyTrue(isfinite(out.metrics.overshoot_pct));
            testCase.verifyClass(out.metrics.steady_state_valid, 'logical');
            testCase.verifyClass(out.metrics.settling_time_reason, 'char');
        end

        function shortRunCannotReportSettling(testCase)
            out = simulate_pid(parameters([10 15 3]), 1);
            testCase.verifyTrue(isnan(out.metrics.settling_time));
        end

        function shortRunExplainsUnconfirmedEstimate(testCase)
            out = simulate_pid(parameters([10 15 3]), 1);
            s = out.metrics;
            testCase.verifyFalse(s.steady_state_valid);
            testCase.verifyFalse(s.settling_time_valid);
            testCase.verifyNotEmpty(s.steady_state_reason);
            testCase.verifyEqual(s.settling_time_reason, s.steady_state_reason);
            testCase.verifyTrue(isfinite(s.steady_value));
            testCase.verifyEqual(s.steady_error, abs(1 - s.steady_value), 'AbsTol', 1e-14);
            testCase.verifyTrue(isnan(s.rise_time) && isnan(s.overshoot_pct));
        end

        function sustainedOscillationIsNotSteady(testCase)
            t = (0:0.01:10).';
            s = pid_metrics(t, 1 + 0.1*cos(2*pi*t), 1);
            testCase.verifyFalse(s.steady_state_valid);
            testCase.verifyEqual(s.steady_state_reason, 'tail_variation');
            testCase.verifyTrue(isnan(s.settling_time));
        end

        function smallButPersistentTrendIsNotSteady(testCase)
            t = (0:0.01:10).';
            s = pid_metrics(t, 1 + 0.006*(t - 9.5), 1);
            testCase.verifyLessThan(s.convergence.deviation, s.convergence.budget);
            testCase.verifyFalse(s.steady_state_valid);
            testCase.verifyEqual(s.steady_state_reason, 'tail_trend');
        end

        function steadyOffsetDoesNotImplyNonconvergence(testCase)
            t = (0:0.1:10).';
            s = pid_metrics(t, 0.7*ones(size(t)), 1);
            testCase.verifyTrue(s.steady_state_valid);
            testCase.verifyTrue(s.settling_time_valid);
            testCase.verifyEqual(s.steady_value, 0.7, 'AbsTol', 1e-14);
            testCase.verifyEqual(s.steady_error, 0.3, 'AbsTol', 1e-14);
        end

        function rippleBelowResolutionIsExplicitlyAccepted(testCase)
            t = (0:0.01:10).';
            y = 1 + 0.002*cos(2*pi*t);
            loose = pid_metrics(t, y, 1, 0.02);
            tight = pid_metrics(t, y, 1, 0.001);
            testCase.verifyTrue(loose.steady_state_valid);
            testCase.verifyFalse(tight.steady_state_valid);
        end

        function deviationBudgetBoundary(testCase, deviationCase)
            [t, y] = symmetricPulses(deviationCase(1));
            s = pid_metrics(t, y, 1, 0.125);
            testCase.verifyEqual(s.steady_state_valid, logical(deviationCase(2)));
            testCase.verifyEqual(s.convergence.deviation, deviationCase(1), 'AbsTol', 1e-14);
        end

        function trendBudgetBoundary(testCase, trendCase)
            t = (0:100).';
            s = pid_metrics(t, 1 + trendCase(1)*(t - 95), 1, 0.125);
            testCase.verifyEqual(s.steady_state_valid, logical(trendCase(2)));
            testCase.verifyEqual(s.convergence.drift, 20*trendCase(1), 'AbsTol', 1e-14);
        end

        function sparseTailCannotConfirmSteadyState(testCase, sparseRecord)
            s = pid_metrics(sparseRecord(:,1), sparseRecord(:,2), 1);
            testCase.verifyFalse(s.steady_state_valid);
            testCase.verifyEqual(s.steady_state_reason, 'insufficient_tail_samples');
            testCase.verifyTrue(isnan(s.settling_time));
        end

        function zeroEquilibriumHasUndefinedRelativeMetrics(testCase)
            t = (0:0.1:10).';
            s = pid_metrics(t, zeros(size(t)), 0);
            testCase.verifyTrue(s.steady_state_valid);
            testCase.verifyFalse(s.settling_time_valid);
            testCase.verifyEqual(s.settling_time_reason, 'zero_steady_value');
            testCase.verifyTrue(isnan(s.settling_time));
        end

        function validityDoesNotDependOnTimeUnitsOrReference(testCase)
            t = (0:0.01:10).';
            y = 1 + 0.006*(t - 9.5);
            a = pid_metrics(t, y, 1);
            b = pid_metrics(100*t + 17, -7*y, 100);
            testCase.verifyEqual(a.steady_state_reason, b.steady_state_reason);
            testCase.verifyEqual(b.convergence.drift, 7*a.convergence.drift, 'AbsTol', 1e-12);
        end

        function saturationForwardsValidity(testCase, actuator)
            p = parameters([10 15 3]);
            p.u_max = actuator{1};
            p.aw_mode = actuator{2};
            out = simulate_saturation(p, 30);
            direct = pid_metrics(out.t, out.y, p.r);
            testCase.verifyTrue(out.metrics.steady_state_valid);
            testCase.verifyTrue(out.metrics.settling_time_valid);
            testCase.verifyEqual(out.metrics.convergence, direct.convergence);
            testCase.verifyEqual(out.metrics.steady_state_reason, direct.steady_state_reason);
        end

        function shortSaturationCannotHideInvalidity(testCase)
            p = parameters([10 15 3]);
            p.u_max = 5;
            p.aw_mode = 'clamp';
            out = simulate_saturation(p, 1);
            testCase.verifyFalse(out.metrics.steady_state_valid);
            testCase.verifyFalse(out.metrics.settling_time_valid);
            testCase.verifyTrue(isnan(out.metrics.settling_time));
        end

        function unreachableReferenceCanHaveSteadyPosition(testCase)
            p = parameters([10 15 3]);
            p.u_max = 3;
            p.aw_mode = 'clamp';
            out = simulate_saturation(p, 30);
            testCase.verifyTrue(out.metrics.steady_state_valid);
            testCase.verifyTrue(out.metrics.settling_time_valid);
            testCase.verifyEqual(out.metrics.steady_value, p.u_max/p.k, 'AbsTol', 1e-4);
            testCase.verifyGreaterThan(out.metrics.steady_error, 0.2);
        end

        function disturbanceCallerRetainsItsInterface(testCase, disturbanceGain)
            p = parameters(disturbanceGain);
            p.t_dist = 10;
            p.d0 = -4;
            out = simulate_disturbance(p, 30);
            direct = pid_metrics(out.pre.t, out.pre.y, p.r);
            testCase.verifyEqual(out.pre.metrics.steady_state_valid, direct.steady_state_valid);
            testCase.verifyTrue(isfinite(out.metrics.post_steady_error));
            testCase.verifyTrue(isfield(out.metrics, 'recovery_time'));
        end
    end
end

function p = parameters(gain)
p = struct('m', 1, 'c', 1, 'k', 4, 'Kp', gain(1), 'Ki', gain(2), 'Kd', gain(3), 'r', 1);
end

function [t, y] = symmetricPulses(amplitude)
% Zero mean and zero linear trend in the final two windows, independently
% isolating the fluctuation budget. All times/values are exactly representable.
t = (0:100).';
y = ones(size(t));
y(ismember(t, [91 99])) = 1 + amplitude;
y(ismember(t, [93 97])) = 1 - amplitude;
end
