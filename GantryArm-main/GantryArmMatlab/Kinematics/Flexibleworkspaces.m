% 获取当前m文件所在目录
[current_dir, ~, ~] = fileparts(mfilename('fullpath'));
% 构建数据文件路径
filePath = fullfile(current_dir, 'data_output.xlsx');
% 读取工作空间点数据
points = readmatrix(filePath, 'Range', 'J:L');
X = points(:, 1);   % X坐标
Y = points(:, 2);   % Y坐标  
Z = points(:, 3);   % Z坐标

% 计算点云的凸包
[K, V] = convhull(X, Y, Z);

% 创建高质量图形
figure('Position', [100, 100, 800, 600], 'Color', 'w');
set(gcf, 'Renderer', 'OpenGL');

% 绘制凸包边界
h = trisurf(K, X, Y, Z, 'FaceColor', [0.2, 0.4, 0.8], ...
           'FaceAlpha', 0.25, 'EdgeColor', [0.1, 0.1, 0.1], ...
           'EdgeAlpha', 0.15, 'LineWidth', 0.3);
hold on;

% 稀疏显示原始采样点
scatter3(X(1:20:end), Y(1:20:end), Z(1:20:end), 8, ...
        'MarkerEdgeColor', 'none', ...
        'MarkerFaceColor', [0.8, 0.2, 0.2], ...
        'MarkerFaceAlpha', 0.4);

% 设置专业的三维坐标系标注
axis_range = max([range(X), range(Y), range(Z)]) * 0.15;
origin = [mean(X), mean(Y), min(Z)];

% 绘制并标注X轴
quiver3(origin(1), origin(2), origin(3), axis_range, 0, 0, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(origin(1)+axis_range*1.1, origin(2), origin(3), 'X', ...
     'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');

% 绘制并标注Y轴
quiver3(origin(1), origin(2), origin(3), 0, axis_range, 0, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(origin(1), origin(2)+axis_range*1.1, origin(3), 'Y', ...
     'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');

% 绘制并标注Z轴
quiver3(origin(1), origin(2), origin(3), 0, 0, axis_range, ...
        'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(origin(1), origin(2), origin(3)+axis_range*1.1, 'Z', ...
     'FontSize', 12, 'FontWeight', 'bold', 'Color', 'b');

% 设置图形属性
xlabel('X轴坐标 (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y轴坐标 (mm)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Z轴坐标 (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('八自由度冗余机械臂可达工作空间凸包边界', ...
      'FontSize', 14, 'FontWeight', 'bold');

% 设置坐标轴属性
axis equal;
grid on;
box on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.2);
set(gca, 'LineWidth', 1.2, 'FontSize', 11);

% 设置标准三维视角
view(45, 30);
rotate3d on;

% 优化光照效果
light('Position', [1, 0.5, 0.5], 'Style', 'infinite');
light('Position', [-1, -0.5, -0.5], 'Style', 'infinite');
lighting gouraud;
material([0.4, 0.6, 0.2, 5, 0.3]);

hold off;

% 输出详细的工作空间参数
fprintf('=== 八自由度机械臂工作空间分析 ===\n');
fprintf('工作空间体积: %.1f mm?\n', V);
fprintf('X轴范围: [%.1f, %.1f] mm (跨度: %.1f mm)\n', min(X), max(X), range(X));
fprintf('Y轴范围: [%.1f, %.1f] mm (跨度: %.1f mm)\n', min(Y), max(Y), range(Y));
fprintf('Z轴范围: [%.1f, %.1f] mm (跨度: %.1f mm)\n', min(Z), max(Z), range(Z));
fprintf('空间中心坐标: (%.1f, %.1f, %.1f) mm\n', mean(X), mean(Y), mean(Z));
fprintf('凸包面片数量: %d\n', size(K, 1));