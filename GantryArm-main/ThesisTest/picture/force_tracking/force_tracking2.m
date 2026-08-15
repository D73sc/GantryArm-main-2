%% 基于导纳控制器的仿真 - 自适应 vs 普通对比（含突变）
clear; close all; clc;

%% 参数设置
m = 20;
b = 350;
k = 400;
k_i = 0.2;
k_e = 4000;
F_r = -40;

% 自适应参数
alpha = 1.0;
beta = 1;

dt = 0.02;
t_total = 4;
n_steps = round(t_total / dt);
time = (0:n_steps-1) * dt;

z_ref = 0.0;
z_e = 0;
spring_max = 0.020;
k_e1 = 4000;
k_e2 = 40000;

% 突变时刻
t_change = 2.0;
step_change = round(t_change / dt);

%% ===== 普通版本 =====
x1 = 0; v1 = 0; F_error_integral1 = 0;
force1 = zeros(1, n_steps);
position1 = zeros(1, n_steps);

for i = 1:n_steps
    z_c = z_ref + x1;
    
    % t=2s时目标力突变为-80N
    if i >= step_change
        F_r_curr = -80;
    else
        F_r_curr = -40;
    end
    
    if z_c <= z_e
        F_z_measured = 0;
    else
        compression = z_c - z_e;
        if compression <= spring_max
            F_z_measured = -k_e1 * compression;
        else
            F_z_measured = -(k_e1 * spring_max + k_e2 * (compression - spring_max));
        end
    end
    
    F_error = F_z_measured - F_r_curr;
    F_error_integral1 = F_error_integral1 + F_error * dt;
    
    a = (F_error - b * v1 - k * x1 - k_i * F_error_integral1) / m;
    
    v1 = v1 + a * dt;
    x1 = x1 + v1 * dt;
    
    force1(i) = F_z_measured;
    position1(i) = x1;
end

%% ===== 自适应版本 =====
x2 = 0; v2 = 0; F_error_integral2 = 0;
k_e_hat = 3000;
x_e_hat = 0.005;

force2 = zeros(1, n_steps);
position2 = zeros(1, n_steps);
k_e_est = zeros(1, n_steps);
x_r_hat_store = zeros(1, n_steps);

for i = 1:n_steps
    z_c = z_ref + x2;
    
    % t=2s时目标力突变为-80N
    if i >= step_change
        F_r_curr = -80;
    else
        F_r_curr = -40;
    end
    
    if z_c <= z_e
        F_z_measured = 0;
    else
        compression = z_c - z_e;
        if compression <= spring_max
            F_z_measured = -k_e1 * compression;
        else
            F_z_measured = -(k_e1 * spring_max + k_e2 * (compression - spring_max));
        end
    end
    
    % 自适应参数更新
    f_c_hat = -k_e_hat * (x2 - x_e_hat);
    force_error = f_c_hat - F_z_measured;
    
    k_e_hat = k_e_hat + alpha * x2 * force_error;
    k_e_hat = max(min(k_e_hat, 50000), 100);
    
    denom = max(k_e_hat, 1e-3);
    x_e_hat = x_e_hat - (beta + alpha * x2 * x_e_hat) / denom * force_error;
    
    x_r_hat = x_e_hat - F_r_curr / k_e_hat;
    delta_x = x2 - x_r_hat;
    
    F_error = F_z_measured - F_r_curr;
    F_error_integral2 = F_error_integral2 + F_error * dt;
    
    a = (F_error - b * v2 - k * delta_x - k_i * F_error_integral2) / m;
    
    v2 = v2 + a * dt;
    x2 = x2 + v2 * dt;
    
    force2(i) = F_z_measured;
    position2(i) = x2;
    k_e_est(i) = k_e_hat;
    x_r_hat_store(i) = x_r_hat;
end

%% 绘图对比
figure('Position', [100, 100, 1600, 800]);

subplot(2,2,1);
plot(time, force1, 'b-', 'LineWidth', 1.5); hold on;
plot(time, force2, 'r-', 'LineWidth', 1.5);
F_r_plot = -40 * ones(size(time));
F_r_plot(step_change:end) = -80;
plot(time, F_r_plot, 'k--', 'LineWidth', 1);
xline(t_change, 'k:', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('接触力 (N)');
title('力跟踪对比');
legend('普通', '自适应', '目标力');
grid on; ylim([-100, 10]);

subplot(2,2,2);
plot(time, position1*1000, 'b-', 'LineWidth', 1.5); hold on;
plot(time, position2*1000, 'r-', 'LineWidth', 1.5);
xline(t_change, 'k:', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('位置 (mm)');
title('位置跟踪对比');
legend('普通', '自适应');
grid on;

subplot(2,2,3);
plot(time, position2*1000, 'r-', 'LineWidth', 1.5); hold on;
plot(time, x_r_hat_store*1000, 'g--', 'LineWidth', 1.5);
xline(t_change, 'k:', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('位置 (mm)');
title('参考轨迹跟踪');
legend('实际位置', '参考轨迹');
grid on;

subplot(2,2,4);
plot(time, k_e_est, 'r-', 'LineWidth', 1.5); hold on;
plot(time, k_e * ones(size(time)), 'k--', 'LineWidth', 1);
xline(t_change, 'k:', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('刚度 (N/m)');
title('刚度估计');
legend('估计值', '真实值');
grid on;

fprintf('普通版本稳态力误差(前2s): %.4f N\n', mean(force1(1:step_change-1)) - (-40));
fprintf('自适应版本稳态力误差(前2s): %.4f N\n', mean(force2(1:step_change-1)) - (-40));
fprintf('最终刚度估计: %.2f N/m (真实: %.2f)\n', k_e_est(end), k_e);