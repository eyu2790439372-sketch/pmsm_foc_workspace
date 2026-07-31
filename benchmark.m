% =========================================================================
% PMSM Sensorless FOC Performance Benchmark & Residual Alignment Suite
% Target: Floating-Point Reference vs. Fixed-Point (Q-Format) C Engine
% =========================================================================

clc; clear; close all;

% 1. 配置模块化路径
addpath(genpath('sim'));

fprintf('---------------------------------------------------------\n');
fprintf('  PMSM-FOC Engineering Benchmark Suite\n');
fprintf('---------------------------------------------------------\n\n');

% 2. 运行浮点基准模型 (Floating-Point Baseline)
fprintf('[1/2] Running Floating-Point Reference Model...\n');
tic;
try
    run_floating_foc;
    t_float = toc;
    fprintf('  -> Execution Latency: %.4f s\n\n', t_float);
catch ME
    warning('Floating-point model execution failed: %s', ME.message);
    t_float = NaN;
end

% 3. 运行定点化模型与残差分析 (Fixed-Point & Residual Alignment)
fprintf('[2/2] Running Fixed-Point (Q-Format) Model & Residual Test...\n');
tic;
try
    test_fixed_residual;
    t_fixed = toc;
    fprintf('  -> Execution Latency: %.4f s\n\n', t_fixed);
catch ME
    warning('Fixed-point residual test failed: %s', ME.message);
    t_fixed = NaN;
end

% 4. 计算与汇总性能指标 (Metrics Extraction)
fprintf('---------------------------------------------------------\n');
fprintf(' [Benchmark Results Summary]\n');
if ~isnan(t_float) && ~isnan(t_fixed)
    fprintf('   - Floating-Point Exec Time : %.4f s\n', t_float);
    fprintf('   - Fixed-Point Exec Time    : %.4f s\n', t_fixed);
    fprintf('   - Execution Speedup        : %.2fx\n', t_float / max(t_fixed, 1e-6));
end

% 5. 校验并保存结果到 assets 目录
if ~exist('assets', 'dir')
    mkdir('assets');
end

if ishandle(1)
    saveas(gcf, 'assets/benchmark_residual_comparison.png');
    fprintf('\n[System] Comparison plot saved to assets/benchmark_residual_comparison.png\n');
end
fprintf('---------------------------------------------------------\n');