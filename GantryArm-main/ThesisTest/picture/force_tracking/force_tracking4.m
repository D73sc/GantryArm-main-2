%% 自适应导纳控制 - 刚度不变，表面高度恶劣变化
clear; close all; clc;

%% 参数设置
m = 20;
b = 400;
k = 300;
k_i = 0.15;
F_d = -40;

alpha = 0.3;
beta = 0.2;

dt = 0.02;
t_total = 8;
n_steps = round(t_total / dt);
time = (0:n_steps-1) * dt;

k_e_const = 4000;    % 刚度始终不变（同一块水泥）
spring_max = 0.020;

%% ===== 构建表面轮廓 =====
z_e_real = zeros(1, n_steps);

for i = 1:n_steps
    t = time(i);
    
    if t < 2.0
        z_e_real(i) = 0;                                        % 平面
    elseif t < 4.0
        z_e_real(i) = -0.004 * (t - 2.0) / 2.0;                % 缓慢斜坡上升4mm
    elseif t < 6.0
        z_e_real(i) = -0.004 + 0.002*sin(2*pi*0.5*(t - 4.0));  % 低频正弦起伏
    else
        z_e_real(i) = -0.004 + 0.004 * (t - 6.0) / 2.0;        % 缓慢下降恢复
    end
end

% 力传感器噪声
noise_std = 0.5;
force_noise = noise_std * randn(1, n_steps);

%% ===== 普通版本 =====
x1 = 0; v1 = 0; F_error_integral1 = 0;
force1 = zeros(1, n_steps);
position1 = zeros(1, n_steps);

for i = 1:n_steps
    z_c = x1;
    z_e = z_e_real(i);
    
    if z_c <= z_e
        F_z = 0;
    else
        compression = z_c - z_e;
        if compression <= spring_max
            F_z = -k_e_const * compression;
        else
            F_z = -(k_e_const * spring_max + 5*k_e_const * (compression - spring_max));
        end
    end
    
    F_z_measured = F_z + force_noise(i);
    
    F_error = F_z_measured - F_d;
    F_error_integral1 = F_error_integral1 + F_error * dt;
    
    a = (F_error - b * v1 - k * x1 - k_i * F_error_integral1) / m;
    v1 = v1 + a * dt;
    x1 = x1 + v1 * dt;
    
    force1(i) = F_z;
    position1(i) = x1;
end

%% ===== 自适应版本 =====
x2 = 0; v2 = 0; F_error_integral2 = 0;
k_e_hat = 4000;
x_e_hat = 0;

x_d = x2; % 参考位置初始化（初始即当前位置）
tau = 1; % 低通滤波时间常数

force2 = zeros(1, n_steps);
position2 = zeros(1, n_steps);
k_e_est = zeros(1, n_steps);
x_e_est = zeros(1, n_steps);
x_r_hat_store = zeros(1, n_steps);
x_d_est=zeros(1, n_steps);
for i = 1:n_steps
    z_c = x2;
    z_e = z_e_real(i);
    
    % 更新参考位置x_d，低通滤波，平滑目标位置
    x_d = (1 - tau) * x_d + tau * z_c;
    
    if z_c <= z_e
        F_z = 0;
    else
        compression = z_c - z_e;
        if compression <= spring_max
            F_z = -k_e_const * compression;
        else
            F_z = -(k_e_const * spring_max + 5*k_e_const * (compression - spring_max));
        end
    end
    
    F_z_measured = F_z + force_noise(i);
    
    % 自适应参数更新
    f_c_hat = -k_e_hat * (z_c - x_e_hat);
    force_error = f_c_hat - F_z_measured;
    
    delta_k = alpha * x_d * force_error;
    delta_k = max(min(delta_k, 500), -500);
    k_e_hat = k_e_hat + delta_k;
    k_e_hat = max(min(k_e_hat, 50000), 100);
    
    denom = max(k_e_hat, 1e-3);
    x_e_hat = x_e_hat - (beta + alpha * x_d * x_e_hat) / denom * force_error;
    
    x_r_hat = x_e_hat - F_d / k_e_hat;
    delta_x = z_c - x_r_hat;
    
    F_error = F_z_measured - F_d;
    F_error_integral2 = F_error_integral2 + F_error * dt;
    
    a = (F_error - b * v2 - k * delta_x - k_i * F_error_integral2) / m;
    v2 = v2 + a * dt;
    x2 = x2 + v2 * dt;
    
    force2(i) = F_z;
    position2(i) = x2;
    k_e_est(i) = k_e_hat;
    x_e_est(i) = x_e_hat;
    x_r_hat_store(i) = x_r_hat;
    x_d_est(i)=x_d;
end
%% 分段性能
seg_idx = {1:75, 76:125, 126:200, 201:250, 251:300, 301:325, 326:n_steps};
seg_names = {'平面', '台阶突起', '斜坡', '高频凹凸', '平台', '凹坑突变', '缓慢恢复'};

fprintf('\n========== 分段力误差对比 ==========\n');
fprintf('%-12s | 普通(N) | 自适应(N) | 改善\n', '场景');
fprintf('----------------------------------------------\n');
for s = 1:length(seg_idx)
    idx = seg_idx{s};
    err1 = mean(abs(force1(idx) - F_d));
    err2 = mean(abs(force2(idx) - F_d));
    improve = (err1 - err2) / err1 * 100;
    fprintf('%-12s | %7.2f | %9.2f | %+.1f%%\n', seg_names{s}, err1, err2, improve);
end


%% 第一张图：环境位置估计

figure('Position', [100, 100, 900, 600]);
plot(time, z_e_real*1000, 'k-', 'LineWidth', 2.5, 'DisplayName', '真实表面'); hold on;
plot(time, x_e_est*1000, 'r-', 'LineWidth', 2, 'DisplayName', '估计环境位置');

xlabel('时间 (s)', 'FontSize', 12);
ylabel('位置 (mm)', 'FontSize', 12);
title('环境位置估计与参考位置', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;
xlim([0, max(time)]);

label_times = [1, 3, 5, 7];
labels = {'平面', '斜坡', '凹凸', '恢复'};
for i = 1:length(labels)
    text(label_times(i), max(x_e_est*1000)*0.8, labels{i}, 'FontSize', 10, 'HorizontalAlignment', 'center');
end

%% 第一张图：环境位置估计

figure('Position', [100, 100, 900, 600]);
plot(time, z_e_real*1000, 'k-', 'LineWidth', 2.5, 'DisplayName', '真实表面'); hold on;
plot(time, x_d_est*1000, 'r-', 'LineWidth', 2, 'DisplayName', '估计环境位置');

xlabel('时间 (s)', 'FontSize', 12);
ylabel('位置 (mm)', 'FontSize', 12);
title('环境位置估计与参考位置', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;
xlim([0, max(time)]);

label_times = [1, 3, 5, 7];
labels = {'平面', '斜坡', '凹凸', '恢复'};
for i = 1:length(labels)
    text(label_times(i), max(x_d_est*1000)*0.8, labels{i}, 'FontSize', 10, 'HorizontalAlignment', 'center');
end

%% 第二张图：力跟踪对比

figure('Position', [100, 100, 900, 600]);
plot(time, force1, 'b-', 'LineWidth', 2, 'DisplayName', '普通导纳控制'); hold on;
plot(time, force2, 'r-', 'LineWidth', 2, 'DisplayName', '自适应导纳控制');
plot(time, F_d*ones(size(time)), 'k--', 'LineWidth', 2, 'DisplayName', '目标力');

xlabel('时间 (s)', 'FontSize', 12);
ylabel('接触力 (N)', 'FontSize', 12);
title('力跟踪对比', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;
xlim([0, max(time)]);
ylim([-100, 20]);

err1_mean = mean(abs(force1 - F_d));
err2_mean = mean(abs(force2 - F_d));
improve_pct = (err1_mean - err2_mean) / err1_mean * 100;

text(max(time)*0.1, -70, ...
    sprintf('普通误差：%.2f N\n自适应误差：%.2f N\n改善：%.1f%%', err1_mean, err2_mean, improve_pct), ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');