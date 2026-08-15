%% 导纳控制器 - 频域分析（正弦扫频）
clear; clc; close all;

fprintf('========================================\n');
fprintf('导纳控制器频域特性分析\n');
fprintf('========================================\n\n');

% 参数
m = 100;
c = 1000;
k = 500;
dt = 0.001;

%% 理论传递函数
fprintf('1. 计算理论传递函数...\n');

s = tf('s');
G_traditional = 1/(m*s^2 + c*s + k);
G_integral = (1 + 5/s)/(m*s^2 + c*s + k);

% Bode图
figure('Name', '频域特性-Bode图', 'Position', [100, 100, 1200, 500]);

subplot(1, 2, 1);
bode(G_traditional, G_integral);
grid on;
legend('传统导纳', '带积分导纳', 'Location', 'best');
title('Bode图对比');

% Nyquist图
subplot(1, 2, 2);
nyquist(G_traditional, G_integral);
grid on;
legend('传统导纳', '带积分导纳', 'Location', 'best');
title('Nyquist图对比');

saveas(gcf, 'figures/frequency_bode.png');
fprintf('  保存: figures/frequency_bode.png\n');

%% 时域正弦扫频验证
fprintf('\n2. 时域正弦扫频仿真...\n');

T = 10;
t = 0:dt:T;
N = length(t);

% 扫频信号（0.1Hz到10Hz）
freq_start = 0.1;
freq_end = 10;
chirp_signal = chirp(t, freq_start, T, freq_end, 'logarithmic');

% Z轴外部力
F_ext = zeros(6, N);
F_ext(3, :) = 30 * chirp_signal;

% 仿真传统导纳
ctrl_trad = AdmittanceController(m, c, k, dt);
[pos_trad, ~, ~] = simulate_admittance(ctrl_trad, F_ext, dt);

% 仿真带积分导纳
ctrl_int = AdmittanceController(m, c, k, dt, 'UseIntegral', true, 'Eta', 5);
[pos_int, ~, ~] = simulate_admittance(ctrl_int, F_ext, dt);

% 绘图
figure('Name', '频域特性-扫频响应', 'Position', [150, 150, 1200, 800]);

subplot(3, 1, 1);
plot(t, F_ext(3,:), 'k-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)'); ylabel('输入力 (N)');
title('(a) 扫频输入信号 (0.1-10 Hz)');

subplot(3, 1, 2);
plot(t, pos_trad(3,:)*1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, pos_int(3,:)*1000, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)'); ylabel('位移 (mm)');
title('(b) 位移响应');
legend('传统导纳', '带积分导纳', 'Location', 'best');

subplot(3, 1, 3);
% 计算幅值比
amp_input = abs(hilbert(F_ext(3,:)));
amp_output_trad = abs(hilbert(pos_trad(3,:)));
amp_output_int = abs(hilbert(pos_int(3,:)));

plot(t, amp_output_trad./amp_input*1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, amp_output_int./amp_input*1000, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)'); ylabel('幅值比 (mm/N)');
title('(c) 频率响应幅值');
legend('传统导纳', '带积分导纳', 'Location', 'best');

saveas(gcf, 'figures/frequency_chirp.png');
fprintf('  保存: figures/frequency_chirp.png\n');

%% 辅助函数
function [pos, vel, acc] = simulate_admittance(controller, F_ext, dt)
    N = size(F_ext, 2);
    pos = zeros(6, N);
    vel = zeros(6, N);
    acc = zeros(6, N);
    
    controller.reset();
    
    for i = 1:N
        pos(:, i) = controller.get_position(F_ext(:, i));
        vel(:, i) = controller.get_velocity();
        acc(:, i) = controller.get_acceleration();
    end
end