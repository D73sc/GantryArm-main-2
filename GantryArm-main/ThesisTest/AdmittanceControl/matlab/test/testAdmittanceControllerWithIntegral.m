
%% 使用示例
clear; clc;

% 参数设置
M = 100.0;
B = 50.0;
K = 200.0;
eta = 5.0;      % 积分增益分子
k_e = 2.0;      % 积分增益分母
dt = 0.001;     % 1ms采样

% 创建控制器
controller = AdmittanceControllerWithIntegral(M, B, K, eta, k_e, dt);

% 仿真参数
t_total = 5.0;
t = 0:dt:t_total;
n = length(t);

% 初始化数据
x_actual = zeros(1, n);
x_desired = zeros(1, n);
F_external = zeros(1, n);
F_desired = zeros(1, n);

% 期望轨迹（正弦轨迹）
omega = 2*pi*0.5;  % 0.5Hz
x_desired = 0.1 * sin(omega * t);
v_desired = 0.1 * omega * cos(omega * t);
a_desired = -0.1 * omega^2 * sin(omega * t);

% 期望力和外部力
F_desired(:) = 5.0;  % 恒定期望力5N
F_external(t >= 1 & t < 2) = 10.0;  % 1-2秒施加10N外力
F_external(t >= 3 & t < 4) = -5.0;  % 3-4秒施加-5N外力

% 仿真循环
for i = 1:n
    [controller, x_actual(i)] = controller.update(...
        F_external(i), ...
        F_desired(i), ...
        x_desired(i), ...
        v_desired(i), ...
        a_desired(i));
end

% 绘图
figure('Position', [100, 100, 1200, 800]);

subplot(3,1,1);
plot(t, x_desired, 'b--', 'LineWidth', 1.5); hold on;
plot(t, x_actual, 'r-', 'LineWidth', 1.5);
ylabel('位置 (m)');
legend('期望轨迹', '实际轨迹');
grid on;
title('带积分补偿的导纳控制');

subplot(3,1,2);
plot(t, F_desired, 'b--', 'LineWidth', 1.5); hold on;
plot(t, F_external, 'r-', 'LineWidth', 1.5);
ylabel('力 (N)');
legend('期望力', '外部力');
grid on;

subplot(3,1,3);
plot(t, x_actual - x_desired, 'LineWidth', 1.5);
ylabel('位置误差 (m)');
xlabel('时间 (s)');
grid on;