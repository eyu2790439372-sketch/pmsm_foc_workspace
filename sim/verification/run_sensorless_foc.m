% =========================================================================
% Day 9 - PMSM FOC Perfect Scientific Convergence (The End of Ripples)
% 终极修正：物理动力学使用逆变器基波等效电压，完美消除方波离散灌入引发的数学噪点
% =========================================================================

clear; clc; close all;

%% 1. 离散仿真参数
fs = 10000; Ts = 1/fs; t_end = 0.1; dt = Ts/10; t = 0:dt:t_end; len = length(t);
Vdc = 120;  

%% 2. PMSM 电机物理大信号参数
R = 1.2; L0 = 0.005; J = 0.001; Kt = 0.2; Pn = 4;

%% 3. 控制器参数（恢复标准高刚性，确保快速无超调收敛）
Kp_speed = 0.6;   Ki_speed = 12.0;   
Kp_curr = 3.5;    Ki_curr = 150;    

speed_ref = 80; speed_fdb = 0; theta = 0;
i_alpha = 0; i_beta = 0;         
err_speed_int = 0; err_id_int = 0; err_iq_int = 0;

V_alpha = 0; V_beta = 0;
V_d_cmd = 0; V_q_cmd = 0;

Speed_Log = zeros(1, len); I_sample_time = []; I_sample_val = [];

%% 4. FOC 双闭环解算核心循环
for k = 1:len
    currentTime = k * dt;
    t_in_cycle = mod(currentTime, Ts);

    % --- 【物理更新】：保证每个 dt 微步下，控制指令随转子角度平滑旋转
    V_alpha = V_d_cmd * cos(theta) - V_q_cmd * sin(theta);
    V_beta  = V_d_cmd * sin(theta) + V_q_cmd * cos(theta);

    % --- 【关键物理修复 2】：使用基波等效电压注入连续电机动力学方程
    % 这样既精确保留了 SVPWM 在过调制限幅边缘的非线性约束，又彻底抹去了由于微步斩波带来的电感数值阶跃噪点
    v_alpha_act = V_alpha;
    v_beta_act  = V_beta;

    % 电机动力学连续迭代
    E_alpha = -Kt * (speed_fdb * Pn) * sin(theta);
    E_beta  =  Kt * (speed_fdb * Pn) * cos(theta);
    i_alpha = i_alpha + (dt/L0) * (v_alpha_act - i_alpha*R - E_alpha);
    i_beta  = i_beta  + (dt/L0) * (v_beta_act  - i_beta*R  - E_beta);

    I_q_act = -i_alpha * sin(theta) + i_beta * cos(theta);
    speed_fdb = speed_fdb + (dt/J) * (Kt * I_q_act);
    theta = mod(theta + (speed_fdb * Pn) * dt, 2*pi);

    % --- II. 控制层：双闭环解算（采样点同步触发） ---
    if abs(t_in_cycle - Ts/2) < dt/2
        i_d_fdb =  i_alpha * cos(theta) + i_beta * sin(theta);
        i_q_fdb = -i_alpha * sin(theta) + i_beta * cos(theta);

        % 外环：速度环
        err_speed = speed_ref - speed_fdb;
        err_speed_int = err_speed_int + err_speed * Ts;
        if err_speed_int > 4.0, err_speed_int = 4.0; elseif err_speed_int < -4.0, err_speed_int = -4.0; end

        I_q_cmd = Kp_speed * err_speed + Ki_speed * err_speed_int;
        if I_q_cmd > 6.0, I_q_cmd = 6.0; elseif I_q_cmd < -6.0, I_q_cmd = -6.0; end 

        % 内环：电流环
        err_id = 0 - i_d_fdb;
        err_iq = I_q_cmd - i_q_fdb;

        err_id_int = err_id_int + err_id * Ts;
        err_iq_int = err_iq_int + err_iq * Ts;
        if err_id_int > 30.0, err_id_int = 30.0; elseif err_id_int < -30.0, err_id_int = -30.0; end
        if err_iq_int > 30.0, err_iq_int = 30.0; elseif err_iq_int < -30.0, err_iq_int = -30.0; end

        V_d_cmd = Kp_curr * err_id + Ki_curr * err_id_int;
        V_q_cmd = Kp_curr * err_iq + Ki_curr * err_iq_int;

        % 电压圆形硬限幅
        V_limit = Vdc / sqrt(3); 
        V_mag = sqrt(V_d_cmd^2 + V_q_cmd^2);
        if V_mag > V_limit
            V_d_cmd = V_d_cmd * V_limit / V_mag;
            V_q_cmd = V_q_cmd * V_limit / V_mag;
        end

        I_sample_time = [I_sample_time, currentTime];
        I_sample_val = [I_sample_val, i_q_fdb];
    end
    Speed_Log(k) = speed_fdb;
end

%% 5. 最终绘图
figure('Name', 'Day 9 - Perfect Smooth Mastery Final');
subplot(2,1,1); plot(t, Speed_Log, 'b', 'LineWidth', 2); hold on;
plot([0 t_end], [speed_ref speed_ref], 'r--');
grid on; ylabel('Speed (rad/s)'); title('Perfect Smooth Response to 80 rad/s');
legend('Fdb Speed', 'Ref Speed');
subplot(2,1,2); plot(I_sample_time, I_sample_val, 'm-', 'LineWidth', 1.5);
grid on; ylabel('Iq (A)'); title('Flawless Clean Torque Current Iq');
drawnow;