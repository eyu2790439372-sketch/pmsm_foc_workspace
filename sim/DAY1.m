
%Trial License -- for use to evaluate programs for possible purchase as an end-user only.

%% Day 1: 克拉克变换 (Clarke Transform) 数学核心验证
clear; clc; close all;

%% 1. 参数定义与三相源信号生成
fs = 10000;              % 采样频率 (Hz)
t = 0 : 1/fs : 0.04;     % 仿真时间 (0.04秒，大约2个工频周期)
f = 50;                  % 信号频率 (50 Hz)
A = 311;                 % 信号幅值 (模拟220V交流电峰值，或电机相电流峰值)

% 生成三相平衡的正弦波信号 (空间与时间上彼此相差 120 度)
I_A = A * cos(2 * pi * f * t);
I_B = A * cos(2 * pi * f * t - 2*pi/3);
I_C = A * cos(2 * pi * f * t + 2*pi/3);

%% 2. 核心算法：执行克拉克变换 (等幅值变换形式)
I_alpha = (2/3) * (I_A - 0.5 * I_B - 0.5 * I_C);
I_beta  = (2/3) * ((sqrt(3)/2) * I_B - (sqrt(3)/2) * I_C);

%% 3. 可视化波形输出
figure('Color', [1 1 1]);

% 子图1：原始三相 A, B, C 信号 (相差120度)
subplot(2,1,1);
plot(t, I_A, 'r', 'LineWidth', 1.5); hold on;
plot(t, I_B, 'g', 'LineWidth', 1.5);
plot(t, I_C, 'b', 'LineWidth', 1.5);
grid on;
title('原始三相静止坐标系信号 (I_A, I_B, I_C)');
xlabel('时间 (秒)');
ylabel('幅值');
legend('A相', 'B相', 'C相');

% 子图2：变换后的 alpha, beta 信号 (相差90度，幅值与原信号一致)
subplot(2,1,2);
plot(t, I_alpha, 'm', 'LineWidth', 1.5); hold on;
plot(t, I_beta, 'c', 'LineWidth', 1.5);
grid on;
title('克拉克变换后的两相静止坐标系信号 (I_\alpha, I_\beta)');
xlabel('时间 (秒)');
ylabel('幅值');
legend('\alpha 轴', '\beta 轴');