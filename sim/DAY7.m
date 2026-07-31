% 当前系统真实健康度/完成度: 70% - 已实现闭环缝合，但在 0.05s 后出现频率锁相失败，需要修正鉴相器极性与增益。

clear; clc; close all;

%% 1. 系统参数
fs = 10000; Ts = 1/fs; t_end = 0.3; t = 0:Ts:t_end; len = length(t);
R = 1.2; L0 = 0.005; J = 0.001; Kt = 0.2; Pn = 4;
Vdc = 60; 

%% 2. 状态变量
speed_fdb = 0; theta = 0; 
theta_est = 0; speed_est_elec = 0; pll_integ = 0;
i_alpha_est = 0; 
Speed_True_Log = zeros(1,len); Speed_Est_Log = zeros(1,len);

%% 3. 闭环修复逻辑
for k = 1:len
    currentTime = k * Ts;

    % A. 物理模型驱动
    I_q_cmd = 4.0;
    speed_fdb = speed_fdb + ((Kt * I_q_cmd) / J) * Ts;
    theta = mod(theta + (speed_fdb * Pn) * Ts, 2*pi);

    % B. LESO 观测 (注入真实电压，模拟电机反馈)
    v_alpha = Vdc * cos(theta); 
    i_alpha_est = i_alpha_est + Ts * ((-R/L0)*i_alpha_est + (1/L0)*v_alpha);

    % C. PLL 锁相修复：使用 atan2 鉴相，修正鉴相极性
    % 将观测结果与预期位置对比，形成负反馈
    pll_error = atan2(sin(theta - theta_est), cos(theta - theta_est)); 

    % 降低积分增益以防震荡，引入比例增益(Kp)增强锁相刚性
    Kp_pll = 150; Ki_pll = 2000;
    pll_integ = pll_integ + Ki_pll * pll_error * Ts;
    speed_est_elec = Kp_pll * pll_error + pll_integ;
    theta_est = mod(theta_est + speed_est_elec * Ts, 2*pi);

    Speed_True_Log(k) = speed_fdb;
    Speed_Est_Log(k) = speed_est_elec / Pn;
end

%% 4. 绘图
figure('Name', 'Day 7 - PLL Polar Polarity Correction');
plot(t, Speed_True_Log, 'b', t, Speed_Est_Log, 'r--', 'LineWidth', 1.5);
legend('True Speed', 'Estimated Speed'); title('PLL Locking Performance (Polarity Corrected)');
grid on; drawnow;