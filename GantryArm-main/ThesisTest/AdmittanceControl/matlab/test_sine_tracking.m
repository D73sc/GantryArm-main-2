%% 导纳控制器仿真 - 正弦力跟踪
clear; clc; close all;

% 仿真参数
dt = 0.001;
T = 10;
t = 0:dt:T;
N = length(t);
% 
% % 外部力：正弦信号
% freq = 0.5;  % 频率 0.5Hz
% F_ext = zeros(3, N);
% F_ext(3, :) = 50 * sin(2 * pi * freq * t);
% 第一段：0 + 50*sin(2*pi*freq*t)
% 计算三个时间段的索引

% 外部力：分段正弦信号
freq = 0.5;  % 频率 0.5Hz
F_ext = zeros(3, N);
N1 = round(N/3);  % 前1/3
N2 = round(2*N/3); % 前2/3
idx1 = 1:N1;
F_ext(3, idx1) = 0 + 50 * sin(2 * pi * freq * t(idx1));

% 第二段：20 + 50*sin(2*pi*freq*t)
idx2 = (N1+1):N2;
F_ext(3, idx2) = 20 + 50 * sin(2 * pi * freq * t(idx2));

% 第三段：50 + 50*sin(2*pi*freq*t)
idx3 = (N2+1):N;
F_ext(3, idx3) = 50 + 50 * sin(2 * pi * freq * t(idx3));
% 控制器参数
m = 1000;
c = 100;
k = 10;
controller = AdmittanceController(m, c, k, dt);

% 仿真
[pos, vel, acc, force] = simulate_admittance(controller, F_ext, dt);

% 绘图
figure('Name', '正弦力跟踪响应', 'Position', [100, 100, 1400, 900]);

subplot(3, 2, 1);
plot(t, F_ext(3, :), 'r--', 'LineWidth', 2); hold on;
plot(t, force(3, :), 'b', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)'); ylabel('力 (N)');
title('外部力输入');
legend('输入力', '实际力');

subplot(3, 2, 2);
plot(t, pos(3, :), 'LineWidth', 2);
grid on;
xlabel('时间 (s)'); ylabel('位移 (m)');
title('Z方向位移响应');

subplot(3, 2, 3);
plot(t, vel(3, :), 'LineWidth', 2);
grid on;
xlabel('时间 (s)'); ylabel('速度 (m/s)');
title('Z方向速度响应');

subplot(3, 2, 4);
plot(t, acc(3, :), 'LineWidth', 2);
grid on;
xlabel('时间 (s)'); ylabel('加速度 (m/s?)');
title('Z方向加速度响应');

% 相位图
subplot(3, 2, 5);
plot(pos(3, :), vel(3, :), 'LineWidth', 2);
grid on;
xlabel('位移 (m)'); ylabel('速度 (m/s)');
title('相位图 (位移-速度)');

% 力-位移关系
subplot(3, 2, 6);
plot(pos(3, :), F_ext(3, :), 'LineWidth', 2);
grid on;
xlabel('位移 (m)'); ylabel('力 (N)');
title('力-位移关系');

%% 仿真函数
function [pos, vel, acc, force] = simulate_admittance(controller, F_ext, dt)
    % 仿真导纳控制器响应
    N = size(F_ext, 2);
    pos = zeros(3, N);
    vel = zeros(3, N);
    acc = zeros(3, N);
    force = zeros(3, N);
    
    controller.reset();
    
    for i = 1:N
        % 输入力
        u = [F_ext(:, i); 0; 0; 0];
        
        % 更新控制器
        pos(:, i) = controller.get_position(u);
        state = controller.get_state();
        vel(:, i) = state(4:6);
        acc(:, i) = controller.get_acceleration();
        force(:, i) = F_ext(:, i);
    end
end