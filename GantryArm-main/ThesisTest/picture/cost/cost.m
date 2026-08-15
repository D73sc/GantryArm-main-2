% 生成运动连续性代价和关节极限代价的可视化图表
clear; close all; clc;

% 设置图表参数
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultTextFontSize', 11);

%% 图1：运动连续性代价函数（单图）
figure('Position', [100, 100, 700, 450]);

% 角度连续性代价
sigma = 0.3;  % 尺度参数
n = 2;        % 形状参数
delta_theta = linspace(0, 2, 200);
C_angle = 1 - exp(-((delta_theta / sigma) .^ n));

% 速度连续性代价
delta_v_threshold = 0.2;  % 速度变化容限
delta_v = linspace(0, 1, 200);
C_velocity = max(0, delta_v - delta_v_threshold);

hold on;
plot(delta_theta, C_angle, 'LineWidth', 2.5, 'Color', '#0072BD', ...
     'DisplayName', '角度连续性代价');
plot(delta_v, C_velocity, 'LineWidth', 2.5, 'Color', '#D95319', ...
     'DisplayName', '速度连续性代价');

grid on; grid minor;
xlabel('变化量', 'FontSize', 12);
ylabel('代价', 'FontSize', 12);
legend('FontSize', 11, 'Location', 'northwest', 'Box', 'on');
set(gca, 'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.3);
xlim([0, 2]); ylim([0, 1]);

%% 图2：关节极限代价函数
figure('Position', [100, 100, 800, 500]);

% 关节极限参数
theta_min = -pi;
theta_max = pi;
rho_values = [0.2, 0.4, 0.6];  % 不同的安全裕度参数

theta = linspace(theta_min - 0.5, theta_max + 0.5, 300);

colors = {'#0072BD', '#D95319', '#EDB120'};
hold on;

for idx = 1:length(rho_values)
    rho = rho_values(idx);
    C_limit = 2 ./ (1 + exp((theta - theta_min) / rho)) + ...
              2 ./ (1 + exp(-(theta - theta_max) / rho));
    plot(theta, C_limit, 'LineWidth', 2.5, 'Color', colors{idx}, ...
         'DisplayName', sprintf('\\rho_i = %.1f', rho));
end

% 标记极限位置
plot([theta_min, theta_min], [0, 4], '--', 'LineWidth', 1.5, 'Color', [0.5 0.5 0.5], 'DisplayName', '关节极限');
plot([theta_max, theta_max], [0, 4], '--', 'LineWidth', 1.5, 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');

grid on; grid minor;
xlabel('关节角度 (rad)', 'FontSize', 12);
ylabel('代价', 'FontSize', 12);
legend('FontSize', 11, 'Location', 'north', 'Box', 'on', 'Interpreter', 'tex');
set(gca, 'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.3);
xlim([theta_min - 0.5, theta_max + 0.5]);
ylim([0, 2.2]);

% 添加x轴标签
xticks([-pi, -pi/2, 0, pi/2, pi]);
xticklabels({'-\pi', '-\pi/2', '0', '\pi/2', '\pi'});