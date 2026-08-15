%% 基于实际导纳控制器的仿真 - 三种实验场景（含积分项）
clear; close all; clc; 

%% 参数设置
mass = [20; 20; 20; 1; 1; 1];
damping = [200; 200; 200; 10; 10; 10];
stiffness = [4000; 4000; 400; 100; 100; 100];
K_i = [0; 0; 1.5; 0; 0; 0];  % 积分增益

dt = 0.02;
t_total = 2;
F_desired = [0; 0; -40; 0; 0; 0];

z_e = 0;
spring_max = 0.020;
k_e1 = 4000;
k_e2 = 40000;

n_steps = round(t_total / dt);
time = (0:n_steps-1) * dt;

%% ========== 实验3：不同alpha1系数对比（刚性接触） ==========
fprintf('实验3：不同alpha1系数对比\n');

%z_ref_3 = 0.03;
z_ref_3 = 0.0;

alpha_values = [0,0.5, 1.5,2];
n_alpha = length(alpha_values);

force_alpha = zeros(n_alpha, n_steps);
damping_alpha = zeros(n_alpha, n_steps);

for idx = 1:n_alpha
    alpha1 = alpha_values(idx);
    
    x = zeros(6, 1);
    v = zeros(6, 1);
    F_error_prev = zeros(6, 1);
    F_error_integral = zeros(6, 1);
    
    for i = 1:n_steps
        z_c = z_ref_3 + x(3);
        
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
        
        F_measured = [0; 0; F_z_measured; 0; 0; 0];
        F_error = F_measured - F_desired;
        F_error_integral = F_error_integral + F_error * dt;
        
        damping_adaptive = damping;
        
        if i > 1
            epsilon_z = abs(F_error(3) - F_error_prev(3));
            damping_adaptive(3) = damping(3) + alpha1 * epsilon_z;
        end
        
        a = (F_error + K_i .* F_error_integral - damping_adaptive .* v - stiffness .* x) ./ mass;
        v = v + a * dt;
        x = x + v * dt;
        
        force_alpha(idx, i) = F_z_measured;
        damping_alpha(idx, i) = damping_adaptive(3);
        F_error_prev = F_error;
    end
end


 
%% 绘图 - 实验3
figure('Position', [100, 750, 800, 600]);
hold on; grid on;
colors = lines(n_alpha);
for idx = 1:n_alpha
    plot(time, force_alpha(idx, :), 'LineWidth', 1.5, 'Color', colors(idx, :), ...
        'DisplayName', sprintf('α=%.1f', alpha_values(idx)));
end
plot(time, F_desired(3) * ones(size(time)), 'k--', 'LineWidth', 1, 'DisplayName', '期望力');
xlabel('时间 (s)'); ylabel('接触力 (N)'); 
%title('不同α?系数对比（刚性接触） - 力跟踪性能');
legend('Location', 'best'); ylim([-60, 10]);