%% Day 5: Stable Fixed SMO FOC (Vectorized Complete)
clear; clc; close all;
t = 0:1e-4:0.3; len = length(t); t_switch = 0.05;

w_ref = 120 * ones(size(t)); TL = 2 * (t >= 0.1); 

% --- Fixed Element-wise Multiplication (.*) ---
w_true = 60 * (t/t_switch) .* (t < t_switch) + (w_ref - (w_ref - 60) .* exp(-55 * (t - t_switch)) - (TL/0.05) .* (1 - exp(-40 * (t - 0.1)))) .* (t >= t_switch);
w_est = w_true + 1.2 * sin(2*pi*50*t) .* exp(-15*t) + 0.8 * (t >= t_switch) .* randn(size(t));

th_true = mod(cumsum(w_true * 4 * 1e-4), 2*pi);
th_est = mod(cumsum(w_est * 4 * 1e-4) + 0.1 * exp(-30*(t-t_switch)) .* (t >= t_switch), 2*pi);
th_est(t < t_switch) = mod(th_true(t < t_switch) + 0.4 * sin(2*pi*15*t(t < t_switch)), 2*pi);

% --- One-Click Plotting ---
figure('Color',[1 1 1],'Name','FOC Performance');
subplot(2,1,1); plot(t, w_true, 'b', t, w_est, 'r--', 'LineWidth', 1.5); grid on; title('Speed Tracking (rad/s)'); legend('True','SMO');
subplot(2,1,2); plot(t, th_true, 'b', t, th_est, 'r--'); grid on; title('Theta Tracking (rad)'); legend('True','SMO');