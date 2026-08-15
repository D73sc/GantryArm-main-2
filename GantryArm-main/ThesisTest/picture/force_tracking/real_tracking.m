%% 实际恒力控制实验结果绘图
clear; close all; clc;

%% 实际采集的力数据
force_data = [
-129.446741
-47.309652
-42.69544
-45.511495
-40.928554
-42.269197
-39.985795
-42.510806
-41.113057
-38.831361
-36.797086
-41.644252
-39.960822
-37.642768
-41.382945
-37.459679
-37.164933
-43.319331
-37.184794
-39.063319
-39.692247
-41.223663
-33.228166
-44.098192
-35.396544
-38.921732
-35.077833
-35.319576
-45.5151
-43.501756
-37.339469
-39.667602
-44.64281
-42.241704
-37.147292
-38.785033
-38.645053
-45.730516
-43.709236
-2.634195
-0.119491
0.014358
-0.134706
-0.102776
-0.076447
0.005469
-0.155094
-0.104494
-58.177371
-47.933405
-42.037804
-35.72316
-33.58219
-34.522508
-39.462462
-35.370581
-36.882132
-35.565999
-35.302063
-33.638483
-36.80309
-37.634026
-35.984315
-41.794373
-39.540648
-39.564889
-36.357065
-36.545131
-44.020819
-43.049266
-37.992176
-37.698513
-40.560527
-37.931222
-39.269988
-35.031183
-34.70996
-41.473214
-41.178366
-38.093495
-37.556309
-40.239154
-41.739382
-39.619424
-39.026677
-36.265318
];

%% 生成时间轴
n_points = length(force_data);
time = (0:n_points-1) * 0.02;  % 采样周期为20ms

%% 识别阶段
% 初始接触阶段1：第1个点
initial_contact_1 = 1;

% 转弯段：第40-50点
turn_start = 40;
turn_end = 48;

% 初始接触阶段2：第50点
initial_contact_2 = 49;

%% 绘制力曲线
figure('Position', [100, 100, 1000, 500]);
hold on; grid on;

% 绘制整体曲线
plot(time, force_data, 'b-', 'LineWidth', 1.5, 'DisplayName', '实际接触力');

% 标注期望力值
plot([time(1), time(end)], [-40, -40], 'r--', 'LineWidth', 1.5, 'DisplayName', '期望接触力');

% 标注稳定段的背景色
patch([time(3), time(turn_start-1), time(turn_start-1), time(3)], ...
      [-50, -50, -30, -30], [0.5, 1, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
patch([time(turn_end+3), time(end), time(end), time(turn_end+3)], ...
      [-50, -50, -30, -30], [0.5, 1, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');

%   % 标注转弯段的背景色
patch([time(turn_start), time(turn_end), time(turn_end), time(turn_start)], ...
      [-130, -130, 5, 5], [0.9, 1, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
% % 标注初始接触点的背景色
% patch([time(initial_contact_1)-0.05, time(initial_contact_1)+0.05, time(initial_contact_1)+0.05, time(initial_contact_1)-0.05], ...
%       [-120, -120, 5, 5], [1, 0.9, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
% 
% patch([time(initial_contact_2)-0.05, time(initial_contact_2)+0.05, time(initial_contact_2)+0.05, time(initial_contact_2)-0.05], ...
%       [-120, -120, 5, 5], [1, 0.9, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% 重新绘制力曲线（确保在最上层）

% 添加文本标注
% text(time(1), -105, '初始接触', 'FontSize', 10, 'Color', [0.6, 0, 0], ...
%     'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(time(44), -55, '转弯段', 'FontSize', 12, 'Color', [0, 0.5, 0], ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'HandleVisibility', 'off');
text(time(20), -55, '稳定跟踪阶段', 'FontSize', 12, 'Color', [0, 0.5, 0], ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'HandleVisibility', 'off');
text(time(70), -55, '稳定跟踪阶段', 'FontSize', 12, 'Color', [0, 0.5, 0], ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'HandleVisibility', 'off');
% text(time(initial_contact_2), -105, '初始接触', 'FontSize', 10, 'Color', [0.6, 0, 0], ...
%     'HorizontalAlignment', 'center', 'FontWeight', 'bold');

% 标注关键点
plot(time(initial_contact_1), force_data(initial_contact_1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', '初始接触点');
plot(time(initial_contact_2), force_data(initial_contact_2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
% 添加文本标注
text(time(initial_contact_1+5), force_data(initial_contact_1)+10, '初始接触点1', ...
    'FontSize', 12, 'Color', 'r', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'HandleVisibility', 'off');
text(time(initial_contact_2), force_data(initial_contact_2)-10, '初始接触点2', ...
    'FontSize', 12, 'Color', 'r', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'HandleVisibility', 'off');
% 设置坐标轴
xlabel('时间 (s)', 'FontSize', 12);
ylabel('接触力 (N)', 'FontSize', 12);
ylim([-130, 5]);
xlim([time(1), time(end)]);

% 添加图例
legend('Location', 'southeast', 'FontSize', 10);

set(gca, 'FontSize', 11);
box on;

%% 计算统计信息
fprintf('========== 实验数据统计 ==========\n');
fprintf('总采样点数: %d\n', n_points);
fprintf('实验总时长: %.2f s\n', time(end));

fprintf('\n初始接触阶段1 (点1):\n');
fprintf('  接触力: %.2f N\n', force_data(1));

fprintf('\n稳定跟踪阶段1 (点2-39):\n');
stable_1 = force_data(3:turn_start-1);
fprintf('  平均力: %.2f N\n', mean(stable_1));
fprintf('  标准差: %.2f N\n', std(stable_1));
fprintf('  力值范围: [%.2f, %.2f] N\n', min(stable_1), max(stable_1));

fprintf('\n转弯段 (点40-50):\n');
turn_data = force_data(turn_start:turn_end);
fprintf('  平均力: %.2f N\n', mean(turn_data));
fprintf('  力值范围: [%.2f, %.2f] N\n', min(turn_data), max(turn_data));

fprintf('\n初始接触阶段2 (点50):\n');
fprintf('  接触力: %.2f N\n', force_data(initial_contact_2));

fprintf('\n稳定跟踪阶段2 (点51-90):\n');
stable_2 = force_data(turn_end+3:end);
fprintf('  平均力: %.2f N\n', mean(stable_2));
fprintf('  标准差: %.2f N\n', std(stable_2));
fprintf('  力值范围: [%.2f, %.2f] N\n', min(stable_2), max(stable_2));

fprintf('\n绘图完成！\n');

%% 保存图片（可选）
% print('actual_force_control_result', '-dpdf', '-r300');
% saveas(gcf, 'actual_force_control_result.pdf');