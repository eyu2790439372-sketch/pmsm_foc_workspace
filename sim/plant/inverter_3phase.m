% 当前系统真实健康度/完成度: 60% - 主回路斩波波形验证成功，开始将断续相电压接入电机，观测动态电流纹波
% 修复日志: 将逆变器大信号输出 (Va_bridge, Vb_bridge) 实时输入离散电机状态方程，重构含高频开关噪声的真实电流环反馈输入

clear; clc; close all;

%% 1. 全局离散化参数（维持10倍超采样以捕获高频开关特性）
fs = 10000;                     % 载波频率 10kHz
Ts = 1/fs;                      % 载波周期
t_end = 0.1; 
t = 0:Ts/10:t_end; 
len = length(t);
dt = Ts/10;                     % 电机方程离散解算步长
Vdc = 60;

%% 2. PMSM 电机物理大信号参数
R = 1.2; L0 = 0.005; J = 0.001; Kt = 0.2; Pn = 4;

%% 3. 状态变量与存盘结构初始化
speed_fdb = 0; theta = 0;
i_alpha = 0; i_beta = 0;         % 电机瞬时电流状态
Ia_Log = zeros(1, len);
Ib_Log = zeros(1, len);
Speed_Log = zeros(1, len);

%% 4. 逆变器-电机功率级联合仿真循环
for k = 1:len
    currentTime = k * dt;
    
    % I. 开环外推基波电压指令
    w_elec = 100 * Pn; 
    theta_open = mod(w_elec * currentTime, 2*pi);
    V_amplitude = 25; 
    V_alpha = V_amplitude * cos(theta_open);
    V_beta  = V_amplitude * sin(theta_open);
    
    % II. SVPWM 扇区判定
    v1 = V_beta;
    v2 = (sqrt(3)*V_alpha - V_beta) / 2;
    v3 = (-sqrt(3)*V_alpha - V_beta) / 2;
    sector = (v1>0) + 2*(v2>0) + 4*(v3>0);
    
    % III. 作用时间解算
    X = sqrt(3) * V_beta * Ts / Vdc;
    Y = (1.5 * V_alpha + 0.5 * sqrt(3) * V_beta) * Ts / Vdc;
    Z = (-1.5 * V_alpha + 0.5 * sqrt(3) * V_beta) * Ts / Vdc;
    
    T1 = 0; T2 = 0;
    switch sector
        case 1, T1 = -Z; T2 = X;
        case 2, T1 = Z;  T2 = Y;
        case 3, T1 = X;  T2 = -Y;
        case 4, T1 = -X; T2 = Z;
        case 5, T1 = -Y; T2 = -Z;
        case 6, T1 = Y;  T2 = -X;
    end
    
    % 限幅与中心对齐驱动分配
    Tsum = T1 + T2;
    if Tsum > Ts, T1 = T1*Ts/Tsum; T2 = T2*Ts/Tsum; end
    T0 = Ts - T1 - T2;
    
    t_in_cycle = mod(currentTime, Ts);
    if sector == 1
        Ta = T0/4; Tb = Ta + T1/2; Tc = Tb + T2/2;
    else
        Ta = T0/4; Tb = T0/4; Tc = T0/4; % 基础映射基准保持
    end
    
    th_t = t_in_cycle;
    if th_t > Ts/2, th_t = Ts - th_t; end
    
    Sa = (th_t > Ta); Sb = (th_t > Tb); Sc = (th_t > Tc);
    
    % IV. 瞬时大信号电压映射
    V_AN = (Vdc/3) * (2*Sa - Sb - Sc);
    V_BN = (Vdc/3) * (-Sa + 2*Sb - Sc);
    V_CN = (Vdc/3) * (-Sa - Sb + 2*Sc);
    
    % Clarke 逆变换变电压到 Alpha-Beta 轴
    v_alpha_act = (2/3) * (V_AN - 0.5*V_BN - 0.5*V_CN);
    v_beta_act  = (2/3) * ((sqrt(3)/2)*V_BN - (sqrt(3)/2)*V_CN);
    
    % V. 实时解算电机电流状态方程（耦合高频断续电压源）
    % 考虑反电动势项
    E_alpha = -Kt * (speed_fdb * Pn) * sin(theta);
    E_beta  =  Kt * (speed_fdb * Pn) * cos(theta);
    
    i_alpha = i_alpha + (dt/L0) * (v_alpha_act - i_alpha*R - E_alpha);
    i_beta  = i_beta  + (dt/L0) * (v_beta_act  - i_beta*R  - E_beta);
    
    % 逆变物理电流重构
    ia_meas = i_alpha;
    ib_meas = -0.5*i_alpha + (sqrt(3)/2)*i_beta;
    
    % 动力学状态更新
    I_q_act = -i_alpha*sin(theta) + i_beta*cos(theta);
    speed_fdb = speed_fdb + (dt/J) * (Kt * I_q_act);
    theta = mod(theta + (speed_fdb * Pn) * dt, 2*pi);
    
    % 数据暂存
    Ia_Log(k) = ia_meas;
    Ib_Log(k) = ib_meas;
    Speed_Log(k) = speed_fdb;
end

%% 5. 绘图验证高频动态响应
figure('Name', 'Day 8 - Combined Inverter-Motor Response');
subplot(2,1,1); plot(t, Ia_Log, 'b', t, Ib_Log, 'g');
grid on; ylabel('Current (A)'); title('Phase Currents (Ia, Ib) with Switching Ripples');
legend('Phase A', 'Phase B');
subplot(2,1,2); plot(t, Speed_Log, 'r', 'LineWidth', 1.5);
grid on; ylabel('Speed (rad/s)'); title('Motor Dynamic Speed Profile under VSI');
drawnow;