%% 导纳参数对控制性能影响的仿真分析
clear; close all; clc;

%% 基础参数设置
dt = 0.02; 
t_total = 2.4; 
F_desired = [0; 0; -40; 0; 0; 0]; 

% 环境参数
z_e = 0; 
spring_max = 0.020; 
k_e1 = 4000;  % 正常接触刚度

% 参考位置（从上方接近）
z_ref = 0; 

n_steps = round(t_total / dt); 
time = (0:n_steps-1) * dt; 

% 固定alpha1=0（不使用自适应阻尼）
alpha1 = 0;

%% ========== 实验1：不同惯性系数m的影响 ==========
fprintf('实验1：不同惯性系数m的影响\n');

% 固定阻尼和刚度
damping_base = [200; 200; 200; 10; 10; 10]; 
stiffness_base = [4000; 4000; 400; 100; 100; 100]; 
K_i = [0; 0; 1.5; 0; 0; 0];

% 测试不同的惯性系数
mass_values = [10, 20, 40];  % kg
n_mass = length(mass_values);

force_mass = zeros(n_mass, n_steps);

for idx = 1:n_mass
    mass_test = [mass_values(idx); mass_values(idx); mass_values(idx); 1; 1; 1];
    
    x = zeros(6, 1); 
    v = zeros(6, 1); 
    F_error_prev = zeros(6, 1); 
    F_error_integral = zeros(6, 1); 
    
    for i = 1:n_steps
        z_c = z_ref + x(3); 
        
        % 计算接触力
        if z_c <= z_e
            F_z_measured = 0; 
        else
            compression = z_c - z_e; 
            if compression <= spring_max
                F_z_measured = -k_e1 * compression; 
            else
                F_z_measured = -(k_e1 * spring_max + k_e1 * (compression - spring_max)); 
            end
        end
        
        F_measured = [0; 0; F_z_measured; 0; 0; 0]; 
        F_error = F_measured - F_desired; 
        F_error_integral = F_error_integral + F_error * dt; 
        
        % 导纳控制（固定阻尼，不使用自适应）
        a = (F_error + K_i .* F_error_integral - damping_base .* v - stiffness_base .* x) ./ mass_test; 
        v = v + a * dt; 
        x = x + v * dt; 
        
        force_mass(idx, i) = F_z_measured; 
        F_error_prev = F_error; 
    end
end

%% 绘图 - 实验1：惯性系数影响
figure('Position', [100, 100, 800, 600]); 
hold on; grid on; 
colors = lines(n_mass); 
for idx = 1:n_mass
    plot(time, force_mass(idx, :), 'LineWidth', 1.5, 'Color', colors(idx, :), ... 
        'DisplayName', sprintf('m = %d kg', mass_values(idx))); 
end
plot(time, F_desired(3) * ones(size(time)), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望力'); 
xlabel('时间 (s)', 'FontSize', 11); 
ylabel('接触力 (N)', 'FontSize', 11); 
legend('Location', 'southeast', 'FontSize', 10); 
ylim([-65, 5]);
set(gca, 'FontSize', 10);

%% ========== 实验2：不同阻尼系数b的影响 ==========
fprintf('实验2：不同阻尼系数b的影响\n');

% 固定惯性和刚度
mass_base = [20; 20; 20; 1; 1; 1]; 
stiffness_base = [4000; 4000; 400; 100; 100; 100]; 

% 测试不同的阻尼系数
damping_values = [100, 200, 400];  % N·s/m
n_damping = length(damping_values);

force_damping = zeros(n_damping, n_steps);

for idx = 1:n_damping
    damping_test = [damping_values(idx); damping_values(idx); damping_values(idx); 10; 10; 10];
    
    x = zeros(6, 1); 
    v = zeros(6, 1); 
    F_error_prev = zeros(6, 1); 
    F_error_integral = zeros(6, 1); 
    
    for i = 1:n_steps
        z_c = z_ref + x(3); 
        
        % 计算接触力
        if z_c <= z_e
            F_z_measured = 0; 
        else
            compression = z_c - z_e; 
            if compression <= spring_max
                F_z_measured = -k_e1 * compression; 
            else
                F_z_measured = -(k_e1 * spring_max + k_e1 * (compression - spring_max)); 
            end
        end
        
        F_measured = [0; 0; F_z_measured; 0; 0; 0]; 
        F_error = F_measured - F_desired; 
        F_error_integral = F_error_integral + F_error * dt; 
        
        % 导纳控制
        a = (F_error + K_i .* F_error_integral - damping_test .* v - stiffness_base .* x) ./ mass_base; 
        v = v + a * dt; 
        x = x + v * dt; 
        
        force_damping(idx, i) = F_z_measured; 
        F_error_prev = F_error; 
    end
end

%% 绘图 - 实验2：阻尼系数影响
figure('Position', [100, 100, 800, 600]); 
hold on; grid on; 
colors = lines(n_damping); 
for idx = 1:n_damping
    plot(time, force_damping(idx, :), 'LineWidth', 1.5, 'Color', colors(idx, :), ... 
        'DisplayName', sprintf('b = %d N·s/m', damping_values(idx))); 
end
plot(time, F_desired(3) * ones(size(time)), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望力'); 
xlabel('时间 (s)', 'FontSize', 11); 
ylabel('接触力 (N)', 'FontSize', 11); 
legend('Location', 'southeast', 'FontSize', 10); 
ylim([-65, 5]);
set(gca, 'FontSize', 10);

%% ========== 实验3：不同刚度系数k的影响 ==========
fprintf('实验3：不同刚度系数k的影响\n');

% 固定惯性和阻尼
mass_base = [20; 20; 20; 1; 1; 1]; 
damping_base = [200; 200; 200; 10; 10; 10]; 

% 测试不同的刚度系数
stiffness_values = [200, 400, 800];  % N/m
n_stiffness = length(stiffness_values);

force_stiffness = zeros(n_stiffness, n_steps);

for idx = 1:n_stiffness
    stiffness_test = [4000; 4000; stiffness_values(idx); 100; 100; 100];
    
    x = zeros(6, 1); 
    v = zeros(6, 1); 
    F_error_prev = zeros(6, 1); 
    F_error_integral = zeros(6, 1); 
    
    for i = 1:n_steps
        z_c = z_ref + x(3); 
        
        % 计算接触力
        if z_c <= z_e
            F_z_measured = 0; 
        else
            compression = z_c - z_e; 
            if compression <= spring_max
                F_z_measured = -k_e1 * compression; 
            else
                F_z_measured = -(k_e1 * spring_max + k_e1 * (compression - spring_max)); 
            end
        end
        
        F_measured = [0; 0; F_z_measured; 0; 0; 0]; 
        F_error = F_measured - F_desired; 
        F_error_integral = F_error_integral + F_error * dt; 
        
        % 导纳控制
        a = (F_error + K_i .* F_error_integral - damping_base .* v - stiffness_test .* x) ./ mass_base; 
        v = v + a * dt; 
        x = x + v * dt; 
        
        force_stiffness(idx, i) = F_z_measured; 
        F_error_prev = F_error; 
    end
end

%% 绘图 - 实验3：刚度系数影响
figure('Position', [100, 100, 800, 600]); 
hold on; grid on; 
colors = lines(n_stiffness); 
for idx = 1:n_stiffness
    plot(time, force_stiffness(idx, :), 'LineWidth', 1.5, 'Color', colors(idx, :), ... 
        'DisplayName', sprintf('k = %d N/m', stiffness_values(idx))); 
end
plot(time, F_desired(3) * ones(size(time)), 'k--', 'LineWidth', 1.5, 'DisplayName', '期望力'); 
xlabel('时间 (s)', 'FontSize', 11); 
ylabel('接触力 (N)', 'FontSize', 11); 
legend('Location', 'southeast', 'FontSize', 10); 
ylim([-65, 5]);
set(gca, 'FontSize', 10);

%% 性能指标计算
fprintf('\n========== 性能指标统计 ==========\n');

% 计算调节时间（到达并保持在±5%误差带内的时间）
tolerance = 0.05 * abs(F_desired(3));

fprintf('\n惯性系数影响：\n');
for idx = 1:n_mass
    settling_idx = find(abs(force_mass(idx, :) - F_desired(3)) <= tolerance, 1, 'first');
    if ~isempty(settling_idx)
        settling_time = time(settling_idx);
        overshoot = (min(force_mass(idx, :)) - F_desired(3)) / abs(F_desired(3)) * 100;
        fprintf('  m = %d kg: 调节时间 = %.2f s, 超调量 = %.1f%%\n', ...
            mass_values(idx), settling_time, overshoot);
    end
end

fprintf('\n阻尼系数影响：\n');
for idx = 1:n_damping
    settling_idx = find(abs(force_damping(idx, :) - F_desired(3)) <= tolerance, 1, 'first');
    if ~isempty(settling_idx)
        settling_time = time(settling_idx);
        overshoot = (min(force_damping(idx, :)) - F_desired(3)) / abs(F_desired(3)) * 100;
        fprintf('  b = %d N·s/m: 调节时间 = %.2f s, 超调量 = %.1f%%\n', ...
            damping_values(idx), settling_time, overshoot);
    end
end

fprintf('\n刚度系数影响：\n');
for idx = 1:n_stiffness
    settling_idx = find(abs(force_stiffness(idx, :) - F_desired(3)) <= tolerance, 1, 'first');
    if ~isempty(settling_idx)
        settling_time = time(settling_idx);
        overshoot = (min(force_stiffness(idx, :)) - F_desired(3)) / abs(F_desired(3)) * 100;
        fprintf('  k = %d N/m: 调节时间 = %.2f s, 超调量 = %.1f%%\n', ...
            stiffness_values(idx), settling_time, overshoot);
    end
end

fprintf('\n仿真完成！\n');