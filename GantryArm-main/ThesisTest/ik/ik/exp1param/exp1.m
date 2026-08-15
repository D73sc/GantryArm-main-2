% ========== 3D线图：Population-Migration-Fitness ==========
clear; clc; close all;

% 读取数据
data = readtable('experiment_results.csv');

fprintf('========== 数据加载完成 ==========\n');
fprintf('数据点总数: %d\n', height(data));
fprintf('==================================\n\n');

% 提取唯一值
pop_sizes = unique(data.PopulationSize);
mig_rates = unique(data.MigrationRate);

fprintf('种群规模: %d - %d (共%d个值)\n', min(pop_sizes), max(pop_sizes), length(pop_sizes));
fprintf('迁移比例: %.4f - %.4f (共%d个值)\n', min(mig_rates), max(mig_rates), length(mig_rates));

% 创建2D矩阵用于查询
Z = zeros(length(pop_sizes), length(mig_rates));
for i = 1:length(pop_sizes)
    for j = 1:length(mig_rates)
        idx = (data.PopulationSize == pop_sizes(i)) & ...
              (data.MigrationRate == mig_rates(j));
        if any(idx)
            Z(i, j) = data.Fitness(idx);
        else
            Z(i, j) = NaN;
        end
    end
end

% 处理无穷大和异常值
Z(isinf(Z)) = nan;
Z_max = max(Z(~isnan(Z) & ~isinf(Z)));
Z_min = min(Z(~isnan(Z) & ~isinf(Z)));

%% ==================== 图1: 3D线图（彩色） ====================
figure('Position', [100, 100, 1200, 900]);
hold on;

% 创建颜色映射
colormap('jet');
cmap = colormap;
num_colors = size(cmap, 1);

% 为每个migration rate画一条线
for j = 1:length(mig_rates)
    % 获取该migration rate对应的所有数据
    fitness_values = Z(:, j);
    
    % 跳过全是NaN的情况
    if all(isnan(fitness_values))
        continue;
    end
    
    % X轴是population size，Y轴是migration rate（固定），Z轴是fitness
    x = pop_sizes';
    y = repmat(mig_rates(j), length(pop_sizes), 1);
    z = fitness_values;
    
    % 颜色根据fitness值映射
    % 将fitness值归一化到[1, num_colors]
    color_idx = round((fitness_values - Z_min) / (Z_max - Z_min) * (num_colors - 1)) + 1;
    color_idx(color_idx < 1) = 1;
    color_idx(color_idx > num_colors) = num_colors;
    color_idx(isnan(color_idx)) = 1;
    
    % 为了实现渐变颜色，分段画线
    for i = 1:length(pop_sizes)-1
        if ~isnan(z(i)) && ~isnan(z(i+1))
            % 取两个点的中间颜色
            mid_color_idx = round((color_idx(i) + color_idx(i+1)) / 2);
            color = cmap(mid_color_idx, :);
            
            plot3(x(i:i+1), y(i:i+1), z(i:i+1), ...
                  'Color', color, ...
                  'LineWidth', 2.5, ...
                  'Marker', 'o', ...
                  'MarkerSize', 4, ...
                  'MarkerFaceColor', color);
        end
    end
end

% 设置轴标签和标题
xlabel('Population Size', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Migration Rate', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('Fitness', 'FontSize', 14, 'FontWeight', 'bold');
title('Parameter Space: Population × Migration × Fitness', ...
      'FontSize', 16, 'FontWeight', 'bold');

% 添加颜色条
cb = colorbar;
ylabel(cb, 'Fitness Value', 'FontSize', 12, 'FontWeight', 'bold');

% 设置视角
view(45, 30);
grid on;
set(gca, 'FontSize', 12);

% 设置轴范围
xlim([min(pop_sizes), max(pop_sizes)]);
ylim([min(mig_rates), max(mig_rates)]);
zlim([Z_min, Z_max]);

saveas(gcf, 'figure_3d_lines_colored.png');
print(gcf, 'figure_3d_lines_colored', '-dpng', '-r300');

%% ==================== 图2: 另一个角度 ====================
figure('Position', [100, 100, 1200, 900]);
hold on;

for j = 1:length(mig_rates)
    fitness_values = Z(:, j);
    if all(isnan(fitness_values))
        continue;
    end
    
    x = pop_sizes';
    y = repmat(mig_rates(j), length(pop_sizes), 1);
    z = fitness_values;
    
    color_idx = round((fitness_values - Z_min) / (Z_max - Z_min) * (num_colors - 1)) + 1;
    color_idx(color_idx < 1) = 1;
    color_idx(color_idx > num_colors) = num_colors;
    color_idx(isnan(color_idx)) = 1;
    
    for i = 1:length(pop_sizes)-1
        if ~isnan(z(i)) && ~isnan(z(i+1))
            mid_color_idx = round((color_idx(i) + color_idx(i+1)) / 2);
            color = cmap(mid_color_idx, :);
            
            plot3(x(i:i+1), y(i:i+1), z(i:i+1), ...
                  'Color', color, ...
                  'LineWidth', 2.5, ...
                  'Marker', 'o', ...
                  'MarkerSize', 4, ...
                  'MarkerFaceColor', color);
        end
    end
end

xlabel('Population Size', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Migration Rate', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('Fitness', 'FontSize', 14, 'FontWeight', 'bold');
title('Parameter Space (Top View)', 'FontSize', 16, 'FontWeight', 'bold');

cb = colorbar;
ylabel(cb, 'Fitness Value', 'FontSize', 12, 'FontWeight', 'bold');

view(0, 90);  % 俯视图
grid on;
set(gca, 'FontSize', 12);

xlim([min(pop_sizes), max(pop_sizes)]);
ylim([min(mig_rates), max(mig_rates)]);
zlim([Z_min, Z_max]);

saveas(gcf, 'figure_3d_lines_top_view.png');
print(gcf, 'figure_3d_lines_top_view', '-dpng', '-r300');

%% ==================== 图3: 侧视图 ====================
figure('Position', [100, 100, 1200, 900]);
hold on;

for j = 1:length(mig_rates)
    fitness_values = Z(:, j);
    if all(isnan(fitness_values))
        continue;
    end
    
    x = pop_sizes';
    y = repmat(mig_rates(j), length(pop_sizes), 1);
    z = fitness_values;
    
    color_idx = round((fitness_values - Z_min) / (Z_max - Z_min) * (num_colors - 1)) + 1;
    color_idx(color_idx < 1) = 1;
    color_idx(color_idx > num_colors) = num_colors;
    color_idx(isnan(color_idx)) = 1;
    
    for i = 1:length(pop_sizes)-1
        if ~isnan(z(i)) && ~isnan(z(i+1))
            mid_color_idx = round((color_idx(i) + color_idx(i+1)) / 2);
            color = cmap(mid_color_idx, :);
            
            plot3(x(i:i+1), y(i:i+1), z(i:i+1), ...
                  'Color', color, ...
                  'LineWidth', 2.5, ...
                  'Marker', 'o', ...
                  'MarkerSize', 4, ...
                  'MarkerFaceColor', color);
        end
    end
end

xlabel('Population Size', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Migration Rate', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('Fitness', 'FontSize', 14, 'FontWeight', 'bold');
title('Parameter Space (Side View)', 'FontSize', 16, 'FontWeight', 'bold');

cb = colorbar;
ylabel(cb, 'Fitness Value', 'FontSize', 12, 'FontWeight', 'bold');

view(0, 0);  % 侧视图
grid on;
set(gca, 'FontSize', 12);

xlim([min(pop_sizes), max(pop_sizes)]);
ylim([min(mig_rates), max(mig_rates)]);
zlim([Z_min, Z_max]);

saveas(gcf, 'figure_3d_lines_side_view.png');
print(gcf, 'figure_3d_lines_side_view', '-dpng', '-r300');

%% ==================== 统计输出 ====================
% 找最优点
[min_val, min_idx] = min(Z(:));
[opt_i, opt_j] = ind2sub(size(Z), min_idx);

fprintf('\n========== 最优参数 ==========\n');
fprintf('Population Size: %d\n', pop_sizes(opt_i));
fprintf('Migration Rate: %.4f\n', mig_rates(opt_j));
fprintf('Fitness: %.6e\n', min_val);
fprintf('==============================\n\n');

fprintf('========== 全局统计 ==========\n');
fprintf('Fitness最小值: %.6e\n', min_val);
fprintf('Fitness最大值: %.6e\n', max(Z(~isnan(Z))));
fprintf('Fitness平均值: %.6e\n', mean(Z(~isnan(Z))));
fprintf('Fitness标准差: %.6e\n', std(Z(~isnan(Z))));
fprintf('==============================\n\n');

fprintf('图片已生成:\n');
fprintf('  - figure_3d_lines_colored.png (45°视角)\n');
fprintf('  - figure_3d_lines_top_view.png (俯视图)\n');
fprintf('  - figure_3d_lines_side_view.png (侧视图)\n');