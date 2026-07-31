% =======================================================================
% 【DAY14 终极连续平滑无超调版代码】
% 1. 依托 DAY12 物理大信号与平滑切轨架构。
% 2. 摒弃硬阈值判断，采用连续平滑函数动态压缩积分上限，根除超调与阶跃冲击。
% =======================================================================

clear; clc; close all;

%% 1. 离散仿真参数
fs = 10000; Ts = 1/fs; t_end = 0.2; dt = Ts/10; t = 0:dt:t_end; len = length(t);
Vdc = 120;  

%% 2. PMSM 电机物理大信号参数（完美对齐 DAY12）
R = 1.2; Ld = 0.005; Lq = 0.005; J = 0.001; Kt = 0.2; Pn = 4; Psif = Kt/Pn;
B = 0.002;

%% 3. 控制器与无感双闭环参数精整（温和阻尼设计）
zeta = 0.90; omega_n = 45;                     
Kp_speed = (2 * zeta * omega_n * J) / Kt;
Ki_speed = (omega_n^2 * J) / Kt;          
Kp_curr = 3.5;    Ki_curr = 150;    

% SMO 观测器参数：坚固的低速抗噪能力
l_gain = 35.0;     
epsilon = 0.40;    

% 二阶锁相环 PLL 参数
omega_pll = 250;
Kp_pll = 2 * 0.707 * omega_pll;
Ki_pll = omega_pll^2;

% 电机真实状态物理量初始化
speed_fdb = 0; theta_real = 0;
i_alpha = 0; i_beta = 0;         
err_speed_int = 0; err_id_int = 0; err_iq_int = 0;
V_d_cmd = 0; V_q_cmd = 0;

% SMO & PLL 状态量初始化
i_alpha_est = 0; i_beta_est = 0;
e_alpha_est = 0; e_beta_est = 0;
theta_est = 0; omega_e_est = 0; err_pll_int = 0;

% I/F 强启参数与平滑切轨时间定义
t_start_switch = 0.04;  
t_end_switch   = 0.06;  
omega_if_acc = 1800;    
theta_if = 0;         
omega_if = 0;         

switch_executed = false;
theta_ctrl = 0;
k_smooth = 0;          

%% 4. 全矩阵预分配数据日志
Real_Theta_Log   = zeros(1, len); 
Est_Theta_Log    = zeros(1, len);
Active_Theta_Log = zeros(1, len);
Speed_Ref_Log    = zeros(1, len); 
Speed_Fdb_Log    = zeros(1, len);
Id_Log           = zeros(1, len); 
Iq_Log           = zeros(1, len);
E_Alpha_Log      = zeros(1, len); 

%% 5. FOC 核心循环
for k = 1:len
    currentTime = k * dt;
    t_in_cycle = mod(currentTime, Ts);
    
    speed_ref = 80; 
    Tl = 0.3; 
    
    % I. 电机连续物理层演轨
    V_alpha = V_d_cmd * cos(theta_real) - V_q_cmd * sin(theta_real);
    V_beta  = V_d_cmd * sin(theta_real) + V_q_cmd * cos(theta_real);
    
    i_d_real =  i_alpha * cos(theta_real) + i_beta * sin(theta_real);
    i_q_real = -i_alpha * sin(theta_real) + i_beta * cos(theta_real);
    
    omega_e = speed_fdb * Pn;
    di_d = (1/Ld) * (V_d_cmd - R*i_d_real + omega_e*Lq*i_q_real);
    di_q = (1/Lq) * (V_q_cmd - R*i_q_real - omega_e*Ld*i_d_real - omega_e*Psif);
    
    i_d_real = i_d_real + di_d * dt;
    i_q_real = i_q_real + di_q * dt;
    
    i_alpha = i_d_real * cos(theta_real) - i_q_real * sin(theta_real);
    i_beta  = i_d_real * sin(theta_real) + i_q_real * cos(theta_real);
    
    Te = Pn * Kt * i_q_real;
    speed_fdb = speed_fdb + (dt/J) * (Te - Tl - B*speed_fdb);
    theta_real = mod(theta_real + omega_e * dt, 2*pi);
    
    % I/F 强制角度生成
    if currentTime < t_end_switch
        omega_if = omega_if + omega_if_acc * dt;
        theta_if = mod(theta_if + omega_if * dt, 2*pi);
    end
    
    % --- II. 控制层与解算层 ---
    if abs(t_in_cycle - Ts/2) < dt/2
        
        % 1. SMO 滑模电磁状态观测
        di_alpha_est = (1/Ld) * (V_alpha - R*i_alpha_est - e_alpha_est);
        di_beta_est  = (1/Ld) * (V_beta  - R*i_beta_est  - e_beta_est);
        
        i_alpha_est = i_alpha_est + di_alpha_est * Ts;
        i_beta_est  = i_beta_est  + di_beta_est * Ts;
        
        err_i_alpha = i_alpha_est - i_alpha;
        err_i_beta  = i_beta_est  - i_beta;
        
        e_alpha_raw = l_gain * tanh(err_i_alpha / epsilon);
        e_beta_raw  = l_gain * tanh(err_i_beta / epsilon);
        
        % 自适应低通滤波器
        speed_abs = abs(omega_e_est);
        m_filter = 0.0012 + (0.0038 - 0.0012) * exp(-speed_abs / 40.0);
        
        e_alpha_est = e_alpha_est + (Ts / m_filter) * (e_alpha_raw - e_alpha_est);
        e_beta_est  = e_beta_est  + (Ts / m_filter) * (e_beta_raw  - e_beta_est);
        
        % 二阶锁相环闭环解算
        err_theta = -e_alpha_est * cos(theta_est) - e_beta_est * sin(theta_est);
        err_pll_int = err_pll_int + err_theta * Ts;
        omega_e_est = Kp_pll * err_theta + Ki_pll * err_pll_int;
        theta_est = mod(theta_est + omega_e_est * Ts, 2*pi);
        
        % I/F 到 SMO 平滑切轨状态机
        if currentTime < t_start_switch
            k_smooth = 0;
            theta_ctrl = theta_if;
            theta_est = theta_if; 
            omega_e_est = omega_if;
            err_pll_int = omega_if / Ki_pll; 
        elseif currentTime >= t_start_switch && currentTime < t_end_switch
            k_smooth = (currentTime - t_start_switch) / (t_end_switch - t_start_switch);
            diff_theta = atan2(sin(theta_est - theta_if), cos(theta_est - theta_if));
            theta_ctrl = mod(theta_if + k_smooth * diff_theta, 2*pi);
        else
            k_smooth = 1;
            if ~switch_executed
                I_q_IF_active = 4.0; 
                err_speed_int = (I_q_IF_active - Kp_speed * (speed_ref - (omega_e_est/Pn))) / Ki_speed;
                switch_executed = true;
            end
            theta_ctrl = theta_est; 
        end
        
        % FOC 核心 Park 变换
        i_d_fdb =  i_alpha * cos(theta_ctrl) + i_beta * sin(theta_ctrl);
        i_q_fdb = -i_alpha * sin(theta_ctrl) + i_beta * cos(theta_ctrl);
        
        % 速度外环控制策略（完全平滑连续的动态限幅）
        if currentTime < t_start_switch
            I_q_cmd = 4.0; 
            omega_feedforward = omega_if;
        else
            speed_est_fdb = omega_e_est / Pn;
            err_speed = speed_ref - speed_est_fdb;
            
            % 标准积分累加
            err_speed_int = err_speed_int + err_speed * Ts;
            
            % 【核心修正】：采用连续 Sigmoid 映射平滑缩减积分上限，杜绝任何阶跃冲击
            smooth_weight = 1.0 / (1.0 + exp(0.15 * (speed_est_fdb - 65)));
            I_q_int_max = 2.0 + 2.5 * smooth_weight; % 从 4.5 平滑过渡到 2.0
            
            if err_speed_int > I_q_int_max
                err_speed_int = I_q_int_max;
            elseif err_speed_int < -I_q_int_max
                err_speed_int = -I_q_int_max;
            end
            
            I_q_feedforward = (Tl + B * speed_ref) / (Pn * Kt);
            I_q_cmd = Kp_speed * err_speed + Ki_speed * err_speed_int + I_q_feedforward;
            
            % 总输出限制
            I_q_max = 7.0;
            if I_q_cmd > I_q_max, I_q_cmd = I_q_max; elseif I_q_cmd < -I_q_max, I_q_cmd = -I_q_max; end
            
            omega_feedforward = omega_e_est;
        end
        
        % 电流内环解算
        err_id = 0 - i_d_fdb;
        err_iq = I_q_cmd - i_q_fdb;
        
        err_id_int = err_id_int + err_id * Ts;
        err_iq_int = err_iq_int + err_iq * Ts;
        
        V_d_PI = Kp_curr * err_id + Ki_curr * err_id_int;
        V_q_PI = Kp_curr * err_iq + Ki_curr * err_iq_int;
        
        V_d_unlim = V_d_PI - omega_feedforward * Lq * i_q_fdb;
        V_q_unlim = V_q_PI + omega_feedforward * Ld * i_d_fdb;
        
        V_limit = Vdc / sqrt(3); 
        if abs(V_d_unlim) > V_limit, V_d_cmd = sign(V_d_unlim) * V_limit; else, V_d_cmd = V_d_unlim; end
        V_max_q = sqrt(max(0, V_limit^2 - V_d_cmd^2));
        if abs(V_q_unlim) > V_max_q, V_q_cmd = sign(V_q_unlim) * V_max_q; else, V_q_cmd = V_q_unlim; end
    end
    
    % 数据日志记录
    Real_Theta_Log(k)   = theta_real;
    Est_Theta_Log(k)    = theta_est;
    Active_Theta_Log(k) = theta_ctrl; 
    Speed_Ref_Log(k)    = speed_ref;
    Speed_Fdb_Log(k)    = speed_fdb;
    Id_Log(k)           = i_d_real; 
    Iq_Log(k)           = i_q_real;
    E_Alpha_Log(k)      = e_alpha_est;
end

%% 6. 绘图验证
figure('Name', 'Day 14 - Fixed Zero-Speed Smooth Transition Matrix');
subplot(3,1,1); plot(t, Real_Theta_Log, 'b', 'LineWidth', 1.5); hold on;
plot(t, Est_Theta_Log, 'r--', 'LineWidth', 1.5);
xline(t_start_switch, 'k:', 'LineWidth', 1.5, 'Label', 'Switch Start');
xline(t_end_switch, 'k:', 'LineWidth', 1.5, 'Label', 'Switch End');
grid on; ylabel('Angle (rad)'); title('1. Position Tracking Matrix (Smooth Weighted Transition)');
legend('Real Theta', 'Estimated Theta', 'Location', 'southeast');

subplot(3,1,2); plot(t, Speed_Ref_Log, 'r--', 'LineWidth', 1.5); hold on;
plot(t, Speed_Fdb_Log, 'b', 'LineWidth', 1.5);
xline(t_start_switch, 'k:', 'LineWidth', 1.5);
xline(t_end_switch, 'k:', 'LineWidth', 1.5);
grid on; ylabel('Speed (rad/s)'); title('2. Speed Loop Dynamic Response & Trajectory Follow');
legend('Ref Speed', 'Fdb Speed', 'Location', 'southeast');

subplot(3,1,3); plot(t, E_Alpha_Log, 'g', 'LineWidth', 1.5); hold on;
xline(t_start_switch, 'k:', 'LineWidth', 1.5);
xline(t_end_switch, 'k:', 'LineWidth', 1.5);
grid on; ylabel('EMF (V)'); title('3. Estimated Back-EMF (Suppressed Chattering via Adaptive Smooth LPF)');
xlabel('Time (s)');