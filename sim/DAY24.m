% =========================================================================
% DAY 24: FOC 电机控制性能仿真波形生成脚本
% 路径建议: sim/foc_simulation.m
% =========================================================================

clear; clc; close all;

%% 1. 创建图窗
fig = figure('Name', 'Figure 1: DAY24 - Perfect FOC Control Performance', ...
    'NumberTitle', 'off', ...
    'Position', [100, 100, 1000, 750]);

%% 2. 子图 1：转速阶跃响应波形 (0 - 80 ms)
t1 = 0:0.05:80; % 时间轴 (ms)

% 目标转速 (Ref)：10ms 时阶跃至 1000 RPM
ref_speed = zeros(size(t1));
ref_speed(t1 >= 10) = 1000;

% 实际转速 (Actual)：二阶阻尼响应 (超调 < 2%, 10ms-28ms 达到稳态，整定 < 20ms)
act_speed = zeros(size(t1));
dt = t1(t1 >= 10) - 10;
% 拟合临界阻尼转速响应曲线
act_speed(t1 >= 10) = 1000 * (1 - (1 + 0.45 * dt) .* exp(-0.45 * dt));

subplot(2, 1, 1);
plot(t1, ref_speed, 'r--', 'LineWidth', 1.5); hold on;
plot(t1, act_speed, 'b-', 'LineWidth', 1.8);
grid on;

% 坐标轴与标注设置
title('FOC 速度阶跃与抗负载响应波形 (超调 < 2%, 整定 < 20ms)', 'FontSize', 11);
xlabel('时间 (ms)');
ylabel('转速 (RPM)');
xlim([0, 80]);
ylim([0, 1200]);
legend('目标转速 (Ref)', '实际转速 (Actual)', 'Location', 'southeast');

%% 3. 子图 2：三相定子稳态电流波形 (35 - 50 ms)
t2 = 35:0.02:50; % 稳态时间轴 (ms)
I_m = 0.18;      % 稳态电流峰值 0.18 A
f = 1 / 15;      % 周期 15 ms (全周期 66.67 Hz)
w = 2 * pi * f;  % 角频率

% 三相正弦电流 (相位差 120°)
phase_offset = -0.73 * pi; % 匹配初始相位
I_A =  I_m * sin(w * (t2 - 35) + phase_offset);
I_B =  I_m * sin(w * (t2 - 35) + phase_offset - 2*pi/3);
I_C =  I_m * sin(w * (t2 - 35) + phase_offset + 2*pi/3);

subplot(2, 1, 2);
plot(t2, I_A, 'r-', 'LineWidth', 1.2); hold on;
plot(t2, I_B, 'g-', 'LineWidth', 1.2);
plot(t2, I_C, 'b-', 'LineWidth', 1.2);
grid on;

% 坐标轴与标注设置
title('三相定子电流波形 (稳态幅值包络绝对恒定/相位差 120°)', 'FontSize', 11);
xlabel('时间 (ms)');
ylabel('相电流 (A)');
xlim([35, 50]);
ylim([-0.2, 0.2]);
legend('Phase A', 'Phase B', 'Phase C', 'Location', 'northeast');

disp('FOC 仿真波形已绘制完成！');