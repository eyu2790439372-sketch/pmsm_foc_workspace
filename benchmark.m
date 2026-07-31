% =========================================================================
% 🚀 30-Day Project Benchmark & Comparison Suite
% 作用：自动对比 Day 1 与 Day 30 的算法表现，并自动保存高清结果图
% =========================================================================

clc; clear; close all;

% 1. 将 sim 文件夹加入环境变量
addpath('sim');

% 2. 创建图片保存目录
if ~exist('assets', 'dir')
    mkdir('assets');
end

fprintf('=========================================\n');
fprintf(' 正在进行 Day 1 vs Day 30 性能基准测试...\n');
fprintf('=========================================\n\n');

% 3. 运行 Day 1 (Baseline) 并计时
fprintf('[1/2] 正在运行 Day 1 基础模型...\n');
tic;
DAY1; % 执行 DAY1.m
t_day1 = toc;

% 4. 运行 Day 30 (Final) 并计时
fprintf('[2/2] 正在运行 Day 30 最终优化模型...\n');
tic;
DAY30; % 执行 DAY30.m
t_day30 = toc;

% 5. 输出性能对比数据
fprintf('\n-----------------------------------------\n');
fprintf('📊 算力耗时对比结果:\n');
fprintf('  - Day 1  单次运行耗时: %.4f 秒\n', t_day1);
fprintf('  - Day 30 单次运行耗时: %.4f 秒\n', t_day30);
if t_day1 > 0
    fprintf('  - 计算效率提升: %.2f%%\n', ((t_day1 - t_day30) / t_day1) * 100);
end
fprintf('-----------------------------------------\n');

% 6. 自动保存当前绘制的图表为 PNG 高清图（供 README 使用）
saveas(gcf, 'assets/comparison_result.png');
fprintf('✅ 比较图表已成功保存至 assets/comparison_result.png\n');