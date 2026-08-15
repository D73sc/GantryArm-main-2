%% ========================================================================
%  修正版：导纳控制器完整分析
%% ========================================================================

clear; clc; close all;

set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultLineLineWidth', 1.5);

%% ========================================================================
%  第一部分：正确的参数设置
%% ========================================================================

fprintf('========================================\n');
fprintf('导纳控制器参数设置\n');
fprintf('========================================\n');

% 导纳控制器参数（重新设计）
M = 1.0;        % 虚拟质量 [kg]
B = 10.0;       % 虚拟阻尼 [N·s/m]
K = 100.0;      % 虚拟刚度 [N/m]
eta = 5.0;      % 积分增益分子
k_e = 1.0;      % 积分增益分母

% 验证计算
omega_n = sqrt(K/M);
zeta = B / (2*sqrt(M*K));

fprintf('参数验证:\n');
fprintf('  M = %.2f kg\n', M);
fprintf('  B = %.2f N·s/m\n', B);
fprintf('  K = %.2f N/m\n', K);
fprintf('  固有频率 ωn = %.4f rad/s (应该=10)\n', omega_n);
fprintf('  阻尼比 ζ = %.4f (应该=0.5)\n', zeta);

if abs(omega_n - 10) > 0.01 || abs(zeta - 0.5) > 0.01
    error('参数计算错误！请检查');
end

%% ========================================================================
%  第二部分：正确的系统建模
%% ========================================================================

% 传统导纳（对力误差的响应）
num_trad = 1;
den_trad = [M, B, K];
sys_traditional = tf(num_trad, den_trad);

% 带积分导纳（状态空间）
A_int = [0,      1,          0;
         -K/M,   -B/M,       eta/(k_e*M);
         0,      0,          0];

B_int = [0,     0;      % 输入: [F_e, F_d]
         1/M,   0;
         -1,    1];

C_int = [1, 0, 0];
D_int = [0, 0];

sys_integral = ss(A_int, B_int, C_int, D_int);

fprintf('\n传递函数验证:\n');
sys_integral_tf = tf(sys_integral);
fprintf('对F_e的传递函数:\n');
sys_integral_tf(1)
fprintf('对F_d的传递函数:\n');
sys_integral_tf(2)

%% ========================================================================
%  第三部分：正确的时域仿真
%% ========================================================================

dt = 0.001;
t_total = 5;
t = 0:dt:t_total;
n = length(t);

%% === 场景1：阶跃力测试（关键修正！）===
fprintf('\n========================================\n');
fprintf('场景1: 阶跃力响应（正确方法）\n');
fprintf('========================================\n');

% 定义力场景
F_external = 10 * ones(size(t));  % 外部力10N
F_desired = 5 * ones(size(t));    % 期望力5N（关键！）

% 传统导纳：对力误差响应
force_error_trad = F_external - F_desired;
[y_trad, ~] = lsim(sys_traditional, force_error_trad, t);

% 带积分导纳：手动仿真（更准确）
x_state = zeros(3, n);  % [位置误差; 速度误差; 积分]

for i = 1:n-1
    % 输入向量
    u = [F_external(i); F_desired(i)];
    
    % 状态导数
    x_dot = A_int * x_state(:,i) + B_int * u;
    
    % 欧拉积分
    x_state(:,i+1) = x_state(:,i) + x_dot * dt;
end

position_int = x_state(1,:);
velocity_int = x_state(2,:);
integral_term = x_state(3,:);

% Figure 1: 阶跃响应对比（修正版）
fig1 = figure('Position', [100, 100, 1200, 900]);

subplot(4,1,1);
plot(t, F_external, 'b-', 'LineWidth', 2); hold on;
plot(t, F_desired, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Force (N)');
title('Input Forces');
legend('F_{external}', 'F_{desired}');
grid on;
xlim([0 5]);

subplot(4,1,2);
plot(t, y_trad*1000, 'b-', 'LineWidth', 2); hold on;
plot(t, position_int*1000, 'r-', 'LineWidth', 2);
yline((F_external(1)-F_desired(1))/K*1000, 'k:', 'LineWidth', 1.5);
yline(0, 'g:', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position Error (mm)');
title('Position Response Comparison');
legend('Traditional', 'With Integral', 'Traditional SS Error', 'Zero Error');
grid on;
xlim([0 5]);

subplot(4,1,3);
plot(t, velocity_int, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Response (With Integral)');
grid on;
xlim([0 5]);

subplot(4,1,4);
plot(t, integral_term, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Force Error Integral');
title('Integral Term Evolution');
grid on;
xlim([0 5]);

sgtitle('Step Force Response Analysis', 'FontSize', 14, 'FontWeight', 'bold');

saveas(fig1, 'Fig1_Step_Response_Corrected.png');

%% 计算性能指标（修正版）
fprintf('\n性能指标分析:\n');
fprintf('----------------------------------------\n');

% 稳态值
ss_trad = y_trad(end);
ss_int = position_int(end);

% 理论稳态误差
theoretical_ss_trad = (F_external(1) - F_desired(1)) / K;
theoretical_ss_int = 0;  % 积分项消除稳态误差

fprintf('传统导纳:\n');
fprintf('  稳态位置误差: %.6f m (%.3f mm)\n', ss_trad, ss_trad*1000);
fprintf('  理论稳态误差: %.6f m (%.3f mm)\n', theoretical_ss_trad, theoretical_ss_trad*1000);
fprintf('  误差比例: %.2f%%\n', abs(ss_trad - theoretical_ss_trad)/theoretical_ss_trad*100);

fprintf('\n带积分导纳:\n');
fprintf('  稳态位置误差: %.6e m (%.6f mm)\n', ss_int, ss_int*1000);
fprintf('  理论稳态误差: %.6e m\n', theoretical_ss_int);
fprintf('  误差改善率: %.2f%%\n', (1 - abs(ss_int)/abs(ss_trad))*100);

% 调节时间（2%误差带）
settling_threshold_trad = 0.02 * abs(theoretical_ss_trad);
settling_threshold_int = 0.02 * max(abs(position_int));

idx_settle_trad = find(abs(y_trad - ss_trad) < settling_threshold_trad, 1, 'first');
idx_settle_int = find(abs(position_int - ss_int) < settling_threshold_int, 1, 'first');

if ~isempty(idx_settle_trad)
    ts_trad = t(idx_settle_trad);
    fprintf('  传统导纳调节时间: %.4f s\n', ts_trad);
end

if ~isempty(idx_settle_int)
    ts_int = t(idx_settle_int);
    fprintf('  积分导纳调节时间: %.4f s\n', ts_int);
end

%% === 场景2：力跟踪性能（实际应用场景）===
fprintf('\n========================================\n');
fprintf('场景2: 实际接触力跟踪\n');
fprintf('========================================\n');

% 变化的期望力
F_desired_var = 5 * ones(size(t));
F_desired_var(t>=2 & t<3) = 8;   % 2-3s: 8N
F_desired_var(t>=4 & t<5) = 3;   % 4-5s: 3N

% 环境接触力（模拟）
F_external_var = zeros(size(t));
F_external_var(t>=1) = 10;  % 1s后接触，持续10N

% 传统导纳仿真
x_trad_force = zeros(2, n);
for i = 1:n-1
    force_err = F_external_var(i) - F_desired_var(i);
    x_dot = [x_trad_force(2,i); 
             (force_err - B*x_trad_force(2,i) - K*x_trad_force(1,i))/M];
    x_trad_force(:,i+1) = x_trad_force(:,i) + x_dot * dt;
end

% 带积分导纳仿真
x_int_force = zeros(3, n);
for i = 1:n-1
    u = [F_external_var(i); F_desired_var(i)];
    x_dot = A_int * x_int_force(:,i) + B_int * u;
    x_int_force(:,i+1) = x_int_force(:,i) + x_dot * dt;
end

% 计算接触力（从位置反推）
F_contact_trad = K * x_trad_force(1,:) + B * x_trad_force(2,:);
F_contact_int = K * x_int_force(1,:) + B * x_int_force(2,:);

% Figure 2: 力跟踪对比
fig2 = figure('Position', [100, 100, 1200, 900]);

subplot(3,1,1);
plot(t, F_external_var, 'b-', 'LineWidth', 2); hold on;
plot(t, F_desired_var, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Force (N)');
title('External Force and Desired Force');
legend('F_{external}', 'F_{desired}');
grid on;

subplot(3,1,2);
plot(t, F_contact_trad, 'b-', 'LineWidth', 2); hold on;
plot(t, F_contact_int, 'r-', 'LineWidth', 2);
plot(t, F_desired_var, 'k--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Contact Force (N)');
title('Contact Force Tracking');
legend('Traditional', 'With Integral', 'Desired');
grid on;

subplot(3,1,3);
force_error_trad_track = F_desired_var - F_contact_trad;
force_error_int_track = F_desired_var - F_contact_int;
plot(t, force_error_trad_track, 'b-', 'LineWidth', 2); hold on;
plot(t, force_error_int_track, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Force Tracking Error (N)');
title('Force Tracking Error');
legend('Traditional', 'With Integral');
grid on;

sgtitle('Force Tracking Performance', 'FontSize', 14, 'FontWeight', 'bold');

saveas(fig2, 'Fig2_Force_Tracking_Corrected.png');

% 计算力跟踪误差
rms_error_trad = sqrt(mean(force_error_trad_track(t>=1).^2));
rms_error_int = sqrt(mean(force_error_int_track(t>=1).^2));

fprintf('\n力跟踪性能:\n');
fprintf('  传统导纳RMS误差: %.4f N\n', rms_error_trad);
fprintf('  积分导纳RMS误差: %.4f N\n', rms_error_int);
fprintf('  改善率: %.2f%%\n', (1 - rms_error_int/rms_error_trad)*100);

%% === 场景3：频域分析 ===
fprintf('\n========================================\n');
fprintf('场景3: 频域特性分析\n');
fprintf('========================================\n');

% Figure 3: Bode图
fig3 = figure('Position', [100, 100, 1200, 600]);

% 对F_e的响应
bode(sys_traditional, 'b', sys_integral(:,1), 'r');
legend('Traditional', 'With Integral (F_e input)', 'Location', 'southwest');
grid on;
title('Bode Diagram - Response to External Force');

saveas(fig3, 'Fig3_Bode_Diagram.png');

%% === 场景4：参数影响分析 ===
fprintf('\n========================================\n');
fprintf('场景4: 积分增益影响\n');
fprintf('========================================\n');

eta_values = [0, 2, 5, 10, 20];
colors = lines(length(eta_values));

fig4 = figure('Position', [100, 100, 1200, 800]);

subplot(2,2,1);
for idx = 1:length(eta_values)
    eta_test = eta_values(idx);
    
    % 构建系统
    A_test = [0, 1, 0; -K/M, -B/M, eta_test/(k_e*M); 0, 0, 0];
    
    % 仿真
    x_test = zeros(3, n);
    for i = 1:n-1
        u = [F_external(i); F_desired(i)];
        x_dot = A_test * x_test(:,i) + B_int * u;
        x_test(:,i+1) = x_test(:,i) + x_dot * dt;
    end
    
    plot(t, x_test(1,:)*1000, 'Color', colors(idx,:), 'LineWidth', 2, ...
         'DisplayName', sprintf('\\eta = %.0f', eta_test)); hold on;
end
yline(0, 'k--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position Error (mm)');
title('(a) Effect of Integral Gain on Response');
legend('Location', 'best');
grid on;
xlim([0 5]);

% 稳态误差 vs 积分增益
subplot(2,2,2);
eta_range = 0:0.5:20;
steady_errors = zeros(size(eta_range));

for idx = 1:length(eta_range)
    eta_test = eta_range(idx);
    
    A_test = [0, 1, 0; -K/M, -B/M, eta_test/(k_e*M); 0, 0, 0];
    
    x_test = zeros(3, n);
    for i = 1:n-1
        u = [F_external(i); F_desired(i)];
        x_dot = A_test * x_test(:,i) + B_int * u;
        x_test(:,i+1) = x_test(:,i) + x_dot * dt;
    end
    
    steady_errors(idx) = abs(x_test(1,end));
end

semilogy(eta_range, steady_errors * 1000, 'b-', 'LineWidth', 2);
xlabel('Integral Gain \eta');
ylabel('Steady-State Error (mm)');
title('(b) Steady-State Error vs \eta');
grid on;

% 阻尼比影响
subplot(2,2,3);
zeta_values = [0.3, 0.5, 0.7, 1.0];
for idx = 1:length(zeta_values)
    zeta_test = zeta_values(idx);
    B_test = 2 * zeta_test * sqrt(M*K);
    
    A_test = [0, 1, 0; -K/M, -B_test/M, eta/(k_e*M); 0, 0, 0];
    
    x_test = zeros(3, n);
    for i = 1:n-1
        u = [F_external(i); F_desired(i)];
        x_dot = A_test * x_test(:,i) + B_int * u;
        x_test(:,i+1) = x_test(:,i) + x_dot * dt;
    end
    
    plot(t, x_test(1,:)*1000, 'LineWidth', 2, ...
         'DisplayName', sprintf('\\zeta = %.1f', zeta_test)); hold on;
end
xlabel('Time (s)');
ylabel('Position Error (mm)');
title('(c) Effect of Damping Ratio');
legend('Location', 'best');
grid on;
xlim([0 5]);

% 积分项演化
subplot(2,2,4);
plot(t, integral_term, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Integral Term Value');
title('(d) Integral Term Evolution');
grid on;
xlim([0 5]);

sgtitle('Parameter Sensitivity Analysis', 'FontSize', 14, 'FontWeight', 'bold');

saveas(fig4, 'Fig4_Parameter_Analysis.png');

%% ========================================================================
%  生成性能对比表
%% ========================================================================

fprintf('\n========================================\n');
fprintf('最终性能对比表\n');
fprintf('========================================\n');

Performance = {'Steady-State Error (mm)'; 
               'Force Tracking RMS Error (N)';
               'Natural Frequency (rad/s)'; 
               'Damping Ratio';
               'Integral Gain'};
           
Traditional_Val = [ss_trad*1000; 
                   rms_error_trad;
                   omega_n;
                   zeta;
                   0];
           
WithIntegral_Val = [ss_int*1000; 
                    rms_error_int;
                    omega_n;
                    zeta;
                    eta/k_e];

comparison_table = table(Traditional_Val, WithIntegral_Val, ...
                         'RowNames', Performance, ...
                         'VariableNames', {'Traditional', 'WithIntegral'});
disp(comparison_table);

writetable(comparison_table, 'Performance_Comparison_Corrected.xlsx', ...
           'WriteRowNames', true);

fprintf('\n========================================\n');
fprintf('分析完成！\n');
fprintf('所有图片已保存为PNG格式\n');
fprintf('性能对比表已保存为Excel文件\n');
fprintf('========================================\n');