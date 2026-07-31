%% Day 4: Speed and Current Dual Closed-Loop System Implementation
clear; clc; close all;

%% 1. Parameters Setup
fs = 10000;                  
t_end = 0.2;                 
t = 0 : 1/fs : t_end;
len = length(t);

R = 1.2;                     
L = 0.005;                   
J = 0.001;                   
Kt = 0.2;                    % Boost torque constant to 0.2
Pn = 4;                      

I_A = 0; I_B = 0; I_C = 0;   
speed_fdb = 0;               
theta = 0;                   

%% 2. PI Controller Init (Tuned for stability)
Kp_speed = 0.8;  Ki_speed = 20;   Ui_speed = 0;
Kp_curr = 4.0;   Ki_curr = 150;   Ui_d = 0; Ui_q = 0;

%% 3. Reference and Load Setup
speed_ref = 100 * ones(1, len); 
% Reduce step load to 2 Nm to match motor capability perfectly
TL = [zeros(1, round(len/2)), 2 * ones(1, len - round(len/2))]; 

%% 4. Data Logger Allocation
Speed_Log  = zeros(1, len);
Iq_Ref_Log = zeros(1, len);
Iq_Fdb_Log = zeros(1, len);
Id_Fdb_Log = zeros(1, len);

%% 5. Core Control Loop
for k = 1:len
    % 5.1 Angle Update
    omega_e = speed_fdb * Pn; 
    theta = theta + omega_e * (1/fs);
    theta = mod(theta, 2*pi); 
    
    % 5.2 Forward Transformation (Clarke -> Park)
    I_alpha = (2/3) * (I_A - 0.5 * I_B - 0.5 * I_C);
    I_beta  = (2/3) * ((sqrt(3)/2) * I_B - (sqrt(3)/2) * I_C);
    
    I_d =  I_alpha * cos(theta) + I_beta * sin(theta);
    I_q = -I_alpha * sin(theta) + I_beta * cos(theta);
    
    % 5.3 Speed Outer Loop PI
    err_speed = speed_ref(k) - speed_fdb;
    Ui_speed = Ui_speed + Ki_speed * err_speed * (1/fs);
    Ui_speed = max(min(Ui_speed, 40), -40);      
    
    I_q_ref = Kp_speed * err_speed + Ui_speed;
    I_q_ref = max(min(I_q_ref, 50), -50);        % Open current limit to 50A to fight load
    I_d_ref = 0;                                 
    
    % 5.4 Current Inner Loop Dual PI
    err_id = I_d_ref - I_d;
    Ui_d = max(min(Ui_d + Ki_curr * err_id * (1/fs), 100), -100);
    V_d = Kp_curr * err_id + Ui_d;
    
    err_iq = I_q_ref - I_q;
    Ui_q = max(min(Ui_q + Ki_curr * err_iq * (1/fs), 100), -100);
    V_q = Kp_curr * err_iq + Ui_q;
    
    % 5.5 Inverse Park Transform
    V_alpha = V_d * cos(theta) - V_q * sin(theta);
    V_beta  = V_d * sin(theta) + V_q * cos(theta);
    
    % 5.6 Plant Physical Model Integration
    I_d = I_d + ((V_d - I_d * R) / L) * (1/fs);
    I_q = I_q + ((V_q - I_q * R) / L) * (1/fs);
    
    Te = Kt * I_q; 
    speed_fdb = speed_fdb + ((Te - TL(k)) / J) * (1/fs);
    
    I_alpha_new = I_d * cos(theta) - I_q * sin(theta);
    I_beta_new  = I_d * sin(theta) + I_q * cos(theta);
    I_A = I_alpha_new;
    I_B = -0.5 * I_alpha_new + (sqrt(3)/2) * I_beta_new;
    I_C = -0.5 * I_alpha_new - (sqrt(3)/2) * I_beta_new;
    
    % 5.7 Save Data
    Speed_Log(k)  = speed_fdb;
    Iq_Ref_Log(k) = I_q_ref;
    Iq_Fdb_Log(k) = I_q;
    Id_Fdb_Log(k) = I_d;
end

%% 6. Visualization Plotting
figure('Color', [1 1 1], 'Name', 'Day 4 FOC Double Closed-Loop Success');

subplot(2,1,1);
plot(t, speed_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(t, Speed_Log, 'b', 'LineWidth', 2);
grid on; title('Speed Loop Tracking (Fixed)');
xlabel('Time (s)'); ylabel('Speed (rad/s)'); legend('Ref', 'Fdb');

subplot(2,1,2);
plot(t, Iq_Ref_Log, 'm--', 'LineWidth', 1.5); hold on;
plot(t, Iq_Fdb_Log, 'g', 'LineWidth', 1.5);
plot(t, Id_Fdb_Log, 'k', 'LineWidth', 1);
grid on; title('Current Loop Tracking & Load Rejection');
xlabel('Time (s)'); ylabel('Current (A)'); legend('Iq\_ref', 'Iq', 'Id');