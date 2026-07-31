%% Sensorless FOC 独立开发日志 - Day 3 SVPWM 核心验证
clear; clc; close all;

%% 1. 参数定义与前置旋转信号模拟 (模拟 Day 2 逆 Park 输出)
fs = 10000;              % 采样频率 (Hz)
t = 0 : 1/fs : 0.02;     % 仿真时间 (0.02秒，1个50Hz周期)
f = 50;                  % 电机旋转工频 (50 Hz)
U_dc = 24;               % 母线电压 (模拟24V供电板)
T_s = 1 / fs;            % PWM 周期

% 直接生成标准的旋转电压矢量 V_alpha 和 V_beta (作为 SVPWM 的输入)
V_max = U_dc / sqrt(3);  % 有效利用率下的最大线性基波幅值
V_alpha = V_max * cos(2 * pi * f * t);
V_beta  = V_max * sin(2 * pi * f * t);

%% 2. Day 3 核心：SVPWM 算法实现
len = length(t);
Ta = zeros(1, len); Tb = zeros(1, len); Tc = zeros(1, len);
Sector_Log = zeros(1, len);

for i = 1:len
    va = V_alpha(i);
    vb = V_beta(i);

    % Step 2.1: 扇区判定
    u1 = vb;
    u2 = (sqrt(3)/2)*va - 0.5*vb;
    u3 = -(sqrt(3)/2)*va - 0.5*vb;

    a_sign = u1 > 0;
    b_sign = u2 > 0;
    c_sign = u3 > 0;
    N = 4*c_sign + 2*b_sign + a_sign;

    switch N
        case 3; sector = 1;
        case 1; sector = 2;
        case 5; sector = 3;
        case 4; sector = 4;
        case 6; sector = 5;
        case 2; sector = 6;
        otherwise; sector = 1;
    end
    Sector_Log(i) = sector;

    % Step 2.2: 根据扇区计算有效矢量时间 X, Y, Z
    X = (sqrt(3)*T_s/U_dc) * vb;
    Y = (sqrt(3)*T_s/U_dc) * ((sqrt(3)/2)*va + 0.5*vb);
    Z = (sqrt(3)*T_s/U_dc) * (-(sqrt(3)/2)*va + 0.5*vb);

    switch sector
        case 1; T1 = -Z; T2 = X;
        case 2; T1 = Y;  T2 = Z;
        case 3; T1 = X;  T2 = -Y;
        case 4; T1 = Z;  T2 = -X;
        case 5; T1 = -Y; T2 = -Z;
        case 6; T1 = -X; T2 = Y;
    end

    % 过调制处理（时间限制）
    if (T1 + T2) > T_s
        sum_T = T1 + T2;
        T1 = T1 * T_s / sum_T;
        T2 = T2 * T_s / sum_T;
    end

    % Step 2.3: 计算三相七段式比较执行时间点
    Ta_tmp = (T_s - T1 - T2) / 4;
    Tb_tmp = Ta_tmp + T1 / 2;
    Tc_tmp = Tb_tmp + T2 / 2;

    % 依据扇区分配给具体的 A, B, C 三相定时器
    switch sector
        case 1; Ta(i) = Ta_tmp; Tb(i) = Tb_tmp; Tc(i) = Tc_tmp;
        case 2; Ta(i) = Tb_tmp; Tb(i) = Ta_tmp; Tc(i) = Tc_tmp;
        case 3; Ta(i) = Tc_tmp; Tb(i) = Ta_tmp; Tc(i) = Tb_tmp;
        case 4; Ta(i) = Tc_tmp; Tb(i) = Tb_tmp; Tc(i) = Ta_tmp;
        case 5; Ta(i) = Tb_tmp; Tb(i) = Tc_tmp; Tc(i) = Ta_tmp;
        case 6; Ta(i) = Ta_tmp; Tb(i) = Tc_tmp; Tc(i) = Tb_tmp;
    end
end

%% 3. 可视化波形输出
figure('Color', [1 1 1], 'Name', 'Day 3 SVPWM 核心波形验证');

% 子图1：输入的旋转控制电压空间矢量
subplot(2,1,1);
plot(t, V_alpha, 'm', 'LineWidth', 1.5); hold on;
plot(t, V_beta, 'c', 'LineWidth', 1.5);
grid on; title('SVPWM 输入源：两相静止控制电压 (V_\alpha, V_\beta)');
ylabel('电压 (V)'); legend('V_\alpha', 'V_\beta');

% 子图2：最终输出给 MCU 定时器计数的调制波（呈现完美的马鞍波！）
subplot(2,1,2);
plot(t, Ta, 'r', 'LineWidth', 1.5); hold on;
plot(t, Tb, 'g', 'LineWidth', 1.5);
plot(t, Tc, 'b', 'LineWidth', 1.5);
grid on; title('SVPWM 最终输出：MCU三相定时器切换占空比时间（经典马鞍波）');
xlabel('时间 (秒)'); ylabel('时间 (秒)'); legend('A相占空比', 'B相占空比', 'C相占空比');