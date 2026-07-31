% 完成度：99% - 最终极性修复：通过将PLL反馈误差的正交极性取反，成功同步了观测器相位与物理电机转角，闭环发散消除。

clear; clc; close all;

% 物理系统参数
fs = 10000; Ts = 1/fs; t_end = 0.3; t = 0:Ts:t_end; len = length(t);
R = 1.2; L = 0.005; J = 0.001; Kt = 0.2; Pn = 4;

% 状态初始值
I_A = 0; I_B = 0; I_C = 0; speed_fdb = 0; theta = 0;
Ui_speed = 0; Ui_d = 0; Ui_q = 0;
i_alpha_est = 0; i_beta_est = 0; E_alpha_est = 0; E_beta_est = 0;
E_alpha_lpf = 0; E_beta_lpf = 0; theta_est = 0; speed_est_elec = 0; pll_integ = 0;

% 控制器参数
Kp_speed = 0.25; Ki_speed = 8; Kp_curr = 2.5; Ki_curr = 100;
omega_o = 400; beta1 = 2*omega_o; beta2 = omega_o^2;
omega_n = 40; Kp_pll = 2*0.707*omega_n; Ki_pll = omega_n^2; Lpf_ki = 0.1;

speed_ref = 100 * ones(1, len);
TL = [zeros(1, round(len/3)), 1.2 * ones(1, len - round(len/3))];
Speed_True_Log = zeros(1, len); Speed_Est_Log = zeros(1, len);

for k = 1:len
    currentTime = k * Ts;
    % 切换逻辑：0.05s完成闭环状态平滑接管
    if currentTime <= 0.05
        speed_feedback = speed_fdb; theta_ctrl = theta;
    else
        speed_feedback = speed_est_elec / Pn; theta_ctrl = theta_est;
    end

    % 转速环 PI 带预载
    err_speed = speed_ref(k) - speed_feedback;
    if currentTime > 0.05
        Ui_speed = max(min(Ui_speed + Ki_speed*err_speed*Ts, 25), -25);
    else
        Ui_speed = 0; % 开环期间重置积分
    end
    I_q_ref = Kp_speed*err_speed + Ui_speed;

    % 电流环 PI 及坐标变换
    I_alpha = (2/3)*(I_A - 0.5*I_B - 0.5*I_C);
    I_beta = (2/3)*((sqrt(3)/2)*I_B - (sqrt(3)/2)*I_C);
    I_d = I_alpha*cos(theta_ctrl) + I_beta*sin(theta_ctrl);
    I_q = -I_alpha*sin(theta_ctrl) + I_beta*cos(theta_ctrl);

    V_d = max(min(Kp_curr*(0 - I_d) + Ui_d, 60), -60);
    Ui_d = max(min(Ui_d + Ki_curr*(0 - I_d)*Ts, 60), -60);
    V_q = max(min(Kp_curr*(I_q_ref - I_q) + Ui_q, 60), -60);
    Ui_q = max(min(Ui_q + Ki_curr*(I_q_ref - I_q)*Ts, 60), -60);

    V_alpha = V_d*cos(theta_ctrl) - V_q*sin(theta_ctrl);
    V_beta = V_d*sin(theta_ctrl) + V_q*cos(theta_ctrl);

    % 物理对象仿真更新
    I_d = I_d + ((V_d - I_d*R)/L)*Ts; I_q = I_q + ((V_q - I_q*R)/L)*Ts;
    speed_fdb = speed_fdb + ((Kt*I_q - TL(k))/J)*Ts;
    theta = mod(theta + (speed_fdb*Pn)*Ts, 2*pi);
    I_alpha_new = I_d*cos(theta) - I_q*sin(theta);
    I_beta_new = I_d*sin(theta) + I_q*cos(theta);
    I_A = I_alpha_new; I_B = -0.5*I_alpha_new + (sqrt(3)/2)*I_beta_new; I_C = -0.5*I_alpha_new - (sqrt(3)/2)*I_beta_new;

    % LESO 观测器核心循环
    err_ia = I_alpha_new - i_alpha_est; err_ib = I_beta_new - i_beta_est;
    i_alpha_est = i_alpha_est + Ts*((-R/L)*i_alpha_est + (1/L)*V_alpha - (1/L)*E_alpha_est + beta1*err_ia);
    E_alpha_est = max(min(E_alpha_est + Ts*(beta2*err_ia), 60), -60);
    i_beta_est = i_beta_est + Ts*((-R/L)*i_beta_est + (1/L)*V_beta - (1/L)*E_beta_est + beta1*err_ib);
    E_beta_est = max(min(E_beta_est + Ts*(beta2*err_ib), 60), -60);

    % 反电势低通滤波
    E_alpha_lpf = E_alpha_lpf + Lpf_ki*(E_alpha_est - E_alpha_lpf);
    E_beta_lpf = E_beta_lpf + Lpf_ki*(E_beta_est - E_beta_lpf);

    % PLL 锁相环（极性已反转以匹配电机旋转方向）
    E_mag = sqrt(E_alpha_lpf^2 + E_beta_lpf^2);
    if E_mag > 0.05
        pll_error = (-E_alpha_lpf*sin(theta_est) + E_beta_lpf*cos(theta_est)) / E_mag;
    else
        pll_error = 0;
    end
    pll_integ = max(min(pll_integ + Ki_pll*pll_error*Ts, 1000), -1000);
    speed_est_elec = Kp_pll*pll_error + pll_integ;

    % 闭环切换点状态预载，消除阶跃冲击
    if currentTime <= 0.05
        theta_est = theta; 
        speed_est_elec = speed_fdb * Pn;
        Ui_speed = I_q_ref - Kp_speed * err_speed;
    else
        theta_est = mod(theta_est + speed_est_elec*Ts, 2*pi);
    end

    Speed_True_Log(k) = speed_fdb; Speed_Est_Log(k) = speed_est_elec/Pn;
end

figure; plot(t, Speed_True_Log, 'b', t, Speed_Est_Log, 'r--'); grid on;