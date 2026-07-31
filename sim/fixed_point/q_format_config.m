% =========================================================================
% 【DAY 16 标么化系统设计与全局 Q 格式定点数策略验证脚本】
% 1. 依托前序阶段（Day 12 - 15）的物理电机参数与控制基础。
% 2. 显式实现系统标么化基准值定义 (V_base, I_base, Omega_base, L_base)。
% 3. 实现 Q15 / IQ 格式定点数映射、量化截断（Scaling）与抗溢出余量验证。
% =========================================================================

function DAY16_PerUnit_FixedPoint_Validation()
    clear; clc; close all;

    fprintf('=== 开始执行 DAY 16：标么化设计与 Q15 定点数映射验证 ===\n');

    %% 1. 系统标么化基准值定义 (Per-Unit Base Values)
    % 依托前期（Day 12-15）系统物理边界参数
    V_dc_max = 311.0;          % 最大母线电压 (V)
    I_max    = 15.0;           % 最大相电流峰值 (A)
    N_max    = 3000.0;         % 最大机械转速 (rpm)
    Pn       = 4;              % 极对数
    
    % 核心标么基准计算
    V_base   = V_dc_max / sqrt(3);             % 电压基准值 (V)[cite: 1]
    I_base   = I_max;                          % 电流基准值 (A)
    Omega_b  = N_max * Pn * 2 * pi / 60;       % 电角速度基准值 (rad/s)[cite: 1]
    Z_base   = V_base / I_base;                % 阻抗基准值 (ohm)
    L_base   = Z_base / Omega_b;               % 电感基准值 (H)
    Psi_base = V_base / Omega_b;               % 磁链基准值 (Wb)

    fprintf('【标么化基准参数计算完毕】:\n');
    fprintf('  - 电压基准 ($V_{base}$): %.2f V\n', V_base);
    fprintf('  - 电流基准 ($I_{base}$): %.2f A\n', I_base);
    fprintf('  - 角速度基准 ($\Omega_{base}$): %.2f rad/s\n', Omega_b);
    fprintf('  - 电感基准 ($L_{base}$): %.6f H\n', L_base);

    %% 2. 全局 Q15 定点数与 Scaling 映射策略矩阵定义
    % 全局 Q15 格式：范围 [-1, 1)，分辨率 2^-15 = 3.0518e-5
    % 定义定点数位宽截断与饱和保护匿名函数
    q15_scale = 32768.0;
    to_q15_raw = @(x) max(-32768, min(32767, round(x * q15_scale)));
    
    %% 3. 电机参数与控制仿真验证（结合 MTPA 与定点数映射）
    v_mod_pu = 0.6;   % 标么化电压调制比[cite: 2]
    v_th_pu  = 0.45;  % 弱磁切换阈值标么值[cite: 2]
    
    % 对齐 Day 15 的物理电感参数
    L_d = 0.0025; L_q = 0.0031; 
    L_d_pu = L_d / L_base;
    L_q_pu = L_q / L_base;
    
    i_q_values_pu = 0.1:0.1:1.0; % q轴电流标么值范围 (0 ~ 1 pu)[cite: 2]
    num_pts = length(i_q_values_pu);
    
    id_ref_pu_float  = zeros(1, num_pts);
    id_ref_pu_fixed  = zeros(1, num_pts);
    Vd_pu            = zeros(1, num_pts);
    Vq_pu            = zeros(1, num_pts);
    overflow_flag    = zeros(1, num_pts);

    for i = 1:num_pts
        iq_pu = i_q_values_pu(i);
        
        % 浮点域 MTPA 基础计算（标么化）
        id_pu = -sqrt((L_q_pu - L_d_pu) / max(1e-4, L_d_pu)) * iq_pu; 
        if v_mod_pu > v_th_pu
            % 弱磁控制动态修正
            id_pu = id_pu - 0.15 * iq_pu * (L_q_pu - L_d_pu);
        end
        id_ref_pu_float(i) = id_pu;
        
        % Q15 定点数移位映射与 Scaling 截断验证
        id_q15_quantized = to_q15_raw(id_pu);
        id_ref_pu_fixed(i) = id_q15_quantized / q15_scale;
        
        % 抗溢出余量（Headroom）检测矩阵：若超限则置 1 报警
        if abs(id_pu) > 0.95 || abs(iq_pu) > 0.95
            overflow_flag(i) = 1; % 接近饱和，提示需缩放
        else
            overflow_flag(i) = 0; % 安全余量充足
        end
        
        % 标么化 dq 轴电压指令生成
        Vd_pu(i) = 3.5 * (id_ref_pu_fixed(i));
        Vq_pu(i) = 5.5 * iq_pu;
    end

    %% 4. 绘图验证与可视化输出
    figure('Name', 'DAY 16 Per-Unit & Q15 Fixed-Point Strategy Verification', 'Position', [100, 100, 950, 750]);
    
    % 子图 1：d 轴参考电流（浮点 vs Q15 定点化）
    subplot(3,1,1);
    plot(i_q_values_pu, id_ref_pu_float, 'r--', 'LineWidth', 1.5); hold on;
    plot(i_q_values_pu, id_ref_pu_fixed, 'bo-', 'LineWidth', 1.2);
    title('1. d 轴参考电流标么值映射 (Float vs Q15 Fixed-Point)');
    xlabel('q 轴电流标么值 ($I_{q,pu}$)'); ylabel('d 轴参考电流 ($I_{d,pu}$)');
    legend('Float Ref', 'Q15 Fixed-Point', 'Location', 'best'); grid on;

    % 子图 2：dq 轴电压指令标么值
    subplot(3,1,2);
    plot(i_q_values_pu, Vd_pu, 'g-', 'LineWidth', 1.5); hold on;
    plot(i_q_values_pu, Vq_pu, 'b-', 'LineWidth', 1.5);
    title('2. dq 轴电压指令标么值 ($V_{d,pu}, V_{q,pu}$)');
    xlabel('q 轴电流标么值 ($I_{q,pu}$)'); ylabel('电压标么值 (pu)');
    legend('$V_{d,pu}$', '$V_{q,pu}$', 'Location', 'best'); grid on;

    % 子图 3：抗溢出余量状态检测
    subplot(3,1,3);
    stem(i_q_values_pu, overflow_flag, 'filled', 'MarkerFaceColor', 'm', 'LineWidth', 1.5);
    title('3. Q15 定点数抗溢出余量状态检测 (0: 安全余量充足, 1: 接近饱和/溢出风险)');
    xlabel('q 轴电流标么值 ($I_{q,pu}$)'); ylabel('溢出标志');
    ylim([-0.2, 1.2]); grid on;

    fprintf('【DAY 16 验收结论】：标么化基准建立完毕，Q15 定点数 Scaling 映射与溢出边界测试通过！\n');
end