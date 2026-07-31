%% Sensorless FOC 独立开发日志 - Day 1 & Day 2 联合验证
clear; clc; close all;

%% 1. 参数定义与三相源信号生成
fs = 10000;              % 采样频率 (Hz)
t = 0 : 1/fs : 0.04;     % 仿真时间 (0.04秒)
f = 50;                  % 电机基频 (50 Hz)
A = 311;                 % 电流幅值 (A)

% 生成三相平衡的交流相电流
I_A = A * cos(2 * pi * f * t);
I_B = A * cos(2 * pi * f * t - 2*pi/3);
I_C = A * cos(2 * pi * f * t + 2*pi/3);

%% 2. Day 1 核心：克拉克变换 (Clarke Transform) - 三相静止 -> 两相静止
I_alpha = (2/3) * (I_A - 0.5 * I_B - 0.5 * I_C);
I_beta  = (2/3) * ((sqrt(3)/2) * I_B - (sqrt(3)/2) * I_C);

%% 3. Day 2 核心：帕克变换 (Park Transform) - 两相静止 -> 两相旋转 (直流化)
% 模拟电机转子的旋转电角度 theta (假设转子速度与电网频率同步，即 theta = omega * t)
theta = 2 * pi * f * t; 

% 执行帕克变换矩阵，注意 I_q 公式里的负号
I_d =  I_alpha .* cos(theta) + I_beta .* sin(theta);
I_q = -I_alpha .* sin(theta) + I_beta .* cos(theta);

%% 4. Day 2 拓展：反帕克变换 (Inverse Park Transform) - 还原验证
% 模拟场景：假设经过 PID 控制器后，输出的直流控制电压为 Vd_ref = 50V, Vq_ref = 150V
Vd_ref = 50  * ones(size(t)); 
Vq_ref = 150 * ones(size(t));

% 执行反帕克变换，将旋转的直流控制量还原为两相静止的交流电压
V_alpha = Vd_ref .* cos(theta) - Vq_ref .* sin(theta);
V_beta  = Vd_ref .* sin(theta) + Vq_ref .* cos(theta);

%% 5. 可视化波形输出
figure('Color', [1 1 1], 'Name', 'FOC 坐标变换全家桶验证');

% 子图1：克拉克变换后的两相静止交变信号（Day 1 成果）
subplot(3,1,1);
plot(t, I_alpha, 'm', 'LineWidth', 1.5); hold on;
plot(t, I_beta, 'c', 'LineWidth', 1.5);
grid on; 
title('【Day 1 成果】两相静止坐标系信号 (I_\alpha, I_\beta) - 交流正弦波');
ylabel('幅值'); legend('\alpha 轴', '\beta 轴');

% 子图2：帕克变换后的旋转坐标系信号（Day 2 核心：见证奇迹的时刻）
subplot(3,1,2);
plot(t, I_d, 'r', 'LineWidth', 2); hold on;
plot(t, I_q, 'b', 'LineWidth', 2);
grid on; 
title('【Day 2 核心】帕克变换后的旋转坐标系信号 (I_d, I_q) - 成功直流化！');
ylim([-50, 400]); ylabel('幅值'); legend('d 轴 (直轴-励磁)', 'q 轴 (交轴-转矩)');

% 子图3：反帕克变换后的还原波形（Day 2 闭环准备）
subplot(3,1,3);
plot(t, V_alpha, 'g', 'LineWidth', 1.5); hold on;
plot(t, V_beta, 'b', 'LineWidth', 1.5);
grid on; 
title('【Day 2 拓展】反帕克变换输出的控制电压 (V_\alpha, V_\beta) - 重新交变');
xlabel('时间 (秒)'); ylabel('电压 (V)'); legend('V_\alpha', 'V_\beta');