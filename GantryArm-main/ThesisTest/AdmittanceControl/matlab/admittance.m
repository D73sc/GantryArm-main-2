%% ========================================
%  导纳控制器学术级仿真
%  作者: Robot Control Lab
%  日期: 2024
%% ========================================

clear; clc; close all;

%% 配置
config = struct();
config.dt = 0.001;              % 采样时间 1ms
config.T_sim = 5;               % 仿真时间 5s
config.M = 1.0;                 % 质量 [kg]
config.B = 10.0;                % 阻尼 [Ns/m]
config.K = 100.0;               % 刚度 [N/m]
config.eta = 5.0;               % 积分增益
config.K_env = 1000;            % 环境刚度 [N/m]
config.B_env = 10;              % 环境阻尼 [Ns/m]
config.z0 = 0.25;               % 环境表面位置 [m]
config.F_desired = 5.0;         % 期望接触力 [N]

%% 运行仿真模块
fprintf('========================================\n');
fprintf('导纳控制器学术级仿真\n');
fprintf('========================================\n\n');

% 模块1: 理论分析（开环）
fprintf('[1/5] 理论分析（传递函数、极点零点）...\n');
results_theory = run_theoretical_analysis(config);

% 模块2: 频域分析（开环）
fprintf('[2/5] 频域分析（Bode图、Nyquist图）...\n');
results_frequency = run_frequency_analysis(config);

% 模块3: 时域响应（开环）
fprintf('[3/5] 时域响应（阶跃、正弦）...\n');
results_time = run_time_response(config);

% 模块4: 力跟踪性能（闭环）
fprintf('[4/5] 力跟踪性能（闭环仿真）...\n');
results_closed = run_closed_loop_tracking(config);

% 模块5: 参数敏感性与对比
fprintf('[5/5] 参数敏感性与性能对比...\n');
results_comparison = run_parameter_comparison(config);

fprintf('\n仿真完成！\n');

%% 生成学术论文级别的图表
fprintf('\n生成图表...\n');
generate_academic_figures(results_theory, results_frequency, ...
                         results_time, results_closed, results_comparison);
fprintf('图表已保存到 figures/ 目录\n');

%% ========================================
%  模块1: 理论分析
%% ========================================
function results = run_theoretical_analysis(cfg)
    s = tf('s');
    
    % 传统导纳控制器（无积分）
    G_trad = 1/(cfg.M*s^2 + cfg.B*s + cfg.K);
    
    % 带积分的导纳控制器
    G_int = (1 + cfg.eta/s) / (cfg.M*s^2 + cfg.B*s + cfg.K);
    
    % 计算特征参数
    omega_n = sqrt(cfg.K/cfg.M);
    zeta = cfg.B/(2*sqrt(cfg.M*cfg.K));
    
    % 极点零点
    [z_int, p_int, k_int] = zpkdata(G_int, 'v');
    
    results = struct();
    results.G_trad = G_trad;
    results.G_int = G_int;
    results.omega_n = omega_n;
    results.zeta = zeta;
    results.poles = p_int;
    results.zeros = z_int;
    
    fprintf('  自然频率 ω? = %.2f rad/s\n', omega_n);
    fprintf('  阻尼比 ζ = %.3f\n', zeta);
    fprintf('  极点: '); disp(p_int');
end

%% ========================================
%  模块2: 频域分析
%% ========================================
function results = run_frequency_analysis(cfg)
    s = tf('s');
    G_int = (1 + cfg.eta/s) / (cfg.M*s^2 + cfg.B*s + cfg.K);
    
    % Bode图
    [mag, phase, wout] = bode(G_int, logspace(-1, 3, 1000));
    mag = squeeze(mag);
    phase = squeeze(phase);
    
    % 计算带宽（-3dB）
    mag_db = 20*log10(mag);
    idx_bw = find(mag_db < max(mag_db) - 3, 1);
    if ~isempty(idx_bw)
        bandwidth = wout(idx_bw);
    else
        bandwidth = NaN;
    end
    
    % 相位裕度
    [Gm, Pm, Wcg, Wcp] = margin(G_int);
    
    results = struct();
    results.mag = mag;
    results.phase = phase;
    results.wout = wout;
    results.bandwidth = bandwidth;
    results.Gm = Gm;
    results.Pm = Pm;
    
    fprintf('  带宽: %.2f rad/s\n', bandwidth);
    fprintf('  相位裕度: %.2f deg\n', Pm);
    fprintf('  增益裕度: %.2f dB\n', 20*log10(Gm));
end

%% ========================================
%  模块3: 时域响应（开环）
%% ========================================
function results = run_time_response(cfg)
    t = 0:cfg.dt:cfg.T_sim;
    
    % 阶跃响应（期望力阶跃）
    F_step = cfg.F_desired * (t >= 1.0);
    [x_step, ~, ~] = simulate_open_loop(t, F_step, cfg);
    
    % 正弦响应（期望力正弦）
    freq = 2.0;  % 2 Hz
    F_sin = cfg.F_desired * (1 + 0.5*sin(2*pi*freq*t));
    [x_sin, ~, ~] = simulate_open_loop(t, F_sin, cfg);
    
    % 斜坡响应
    F_ramp = min(cfg.F_desired * (t - 1.0), cfg.F_desired);
    F_ramp(t < 1.0) = 0;
    [x_ramp, ~, ~] = simulate_open_loop(t, F_ramp, cfg);
    
    % 计算性能指标（阶跃）
    idx_start = find(t >= 1.0, 1);
    idx_settle = find(abs(x_step(idx_start:end) - x_step(end)) < 0.02*abs(x_step(end)), 1);
    if ~isempty(idx_settle)
        settling_time = t(idx_start + idx_settle - 1) - 1.0;
    else
        settling_time = NaN;
    end
    
    overshoot = (max(x_step(idx_start:end)) - x_step(end)) / x_step(end) * 100;
    
    results = struct();
    results.t = t;
    results.x_step = x_step;
    results.x_sin = x_sin;
    results.x_ramp = x_ramp;
    results.F_step = F_step;
    results.F_sin = F_sin;
    results.F_ramp = F_ramp;
    results.settling_time = settling_time;
    results.overshoot = overshoot;
    
    fprintf('  阶跃响应 - 调节时间: %.3f s\n', settling_time);
    fprintf('  阶跃响应 - 超调量: %.2f%%\n', overshoot);
end

%% ========================================
%  模块4: 闭环力跟踪（最重要！）
%% ========================================
function results = run_closed_loop_tracking(cfg)
    t = 0:cfg.dt:cfg.T_sim;
    N = length(t);
    
    % 初始化
    x = zeros(N, 1);
    v = zeros(N, 1);
    F_actual = zeros(N, 1);
    F_error = zeros(N, 1);
    integral_term = zeros(N, 1);
    
    % 期望力轨迹（分段）
    F_desired = zeros(N, 1);
    F_desired(t >= 0.5 & t < 2.0) = 5.0;    % 阶跃
    F_desired(t >= 2.0 & t < 3.5) = 5.0 + 2.0*sin(4*pi*(t(t>=2.0 & t<3.5) - 2.0));  % 正弦
    F_desired(t >= 3.5) = 8.0;              % 第二个阶跃
    
    % 初始位置（未接触）
    x(1) = cfg.z0 + 0.01;
    integral = 0;
    
    % 闭环仿真
    for i = 2:N
        % 计算环境接触力
        if x(i-1) <= cfg.z0
            penetration = cfg.z0 - x(i-1);
            F_actual(i-1) = cfg.K_env * penetration + cfg.B_env * (-v(i-1));
        else
            F_actual(i-1) = 0;
        end
        
        % 导纳控制器
        F_error(i-1) = F_desired(i-1) - F_actual(i-1);
        integral = integral + F_error(i-1) * cfg.dt;
        integral = max(min(integral, 1.0), -1.0);  % 限幅
        integral_term(i-1) = integral;
        
        % 动力学更新
        F_control = F_error(i-1) + cfg.eta * integral;
        a = (F_control - cfg.B * v(i-1) - cfg.K * (x(i-1) - cfg.z0)) / cfg.M;
        v(i) = v(i-1) + a * cfg.dt;
        x(i) = x(i-1) + v(i) * cfg.dt;
    end
    
    % 最后一个点的力
    if x(end) <= cfg.z0
        penetration = cfg.z0 - x(end);
        F_actual(end) = cfg.K_env * penetration + cfg.B_env * (-v(end));
    end
    
    % 性能指标（只计算接触阶段）
    idx_contact = find(F_actual > 0.5, 1);
    if ~isempty(idx_contact)
        F_err_steady = F_actual(end-1000:end) - F_desired(end-1000:end);
        RMSE = sqrt(mean(F_err_steady.^2));
        MAE = mean(abs(F_err_steady));
        steady_state_error = mean(F_err_steady);
    else
        RMSE = NaN; MAE = NaN; steady_state_error = NaN;
    end
    
    results = struct();
    results.t = t;
    results.x = x;
    results.F_actual = F_actual;
    results.F_desired = F_desired;
    results.F_error = F_error;
    results.integral = integral_term;
    results.RMSE = RMSE;
    results.MAE = MAE;
    results.steady_error = steady_state_error;
    
    fprintf('  力跟踪 RMSE: %.4f N\n', RMSE);
    fprintf('  稳态误差: %.4f N\n', steady_state_error);
end

%% ========================================
%  模块5: 参数对比
%% ========================================
function results = run_parameter_comparison(cfg)
    t = 0:cfg.dt:3.0;
    N = length(t);
    F_desired = 5.0 * (t >= 0.5);
    
    % 测试不同积分增益
    eta_values = [0, 2, 5, 10, 20];
    n_cases = length(eta_values);
    
    X_all = zeros(N, n_cases);
    F_all = zeros(N, n_cases);
    metrics = struct('eta', [], 'RMSE', [], 'settling_time', []);
    
    for i = 1:n_cases
        cfg_temp = cfg;
        cfg_temp.eta = eta_values(i);
        
        [x, F_actual] = simulate_closed_loop_simple(t, cfg_temp);
        X_all(:, i) = x;
        F_all(:, i) = F_actual;
        
        % 计算指标
        idx_valid = F_actual > 0.5;
        if any(idx_valid)
            RMSE = sqrt(mean((F_actual(idx_valid) - 5.0).^2));
        else
            RMSE = NaN;
        end
        
        metrics(i).eta = eta_values(i);
        metrics(i).RMSE = RMSE;
    end
    
    results = struct();
    results.t = t;
    results.eta_values = eta_values;
    results.X_all = X_all;
    results.F_all = F_all;
    results.metrics = metrics;
end

%% ========================================
%  辅助函数: 开环仿真
%% ========================================
function [x, v, a] = simulate_open_loop(t, F_desired, cfg)
    N = length(t);
    x = zeros(N, 1);
    v = zeros(N, 1);
    a = zeros(N, 1);
    integral = 0;
    
    for i = 2:N
        % 积分项
        integral = integral + F_desired(i-1) * cfg.dt;
        
        % 导纳控制力
        F_control = F_desired(i-1) + cfg.eta * integral;
        
        % 动力学
        a(i-1) = (F_control - cfg.B * v(i-1) - cfg.K * x(i-1)) / cfg.M;
        v(i) = v(i-1) + a(i-1) * cfg.dt;
        x(i) = x(i-1) + v(i) * cfg.dt;
    end
end

%% ========================================
%  辅助函数: 闭环仿真（简化版）
%% ========================================
function [x, F_actual] = simulate_closed_loop_simple(t, cfg)
    N = length(t);
    x = zeros(N, 1);
    v = zeros(N, 1);
    F_actual = zeros(N, 1);
    
    x(1) = cfg.z0 + 0.01;
    integral = 0;
    
    for i = 2:N
        % 环境力
        if x(i-1) <= cfg.z0
            penetration = cfg.z0 - x(i-1);
            F_actual(i-1) = cfg.K_env * penetration + cfg.B_env * (-v(i-1));
        else
            F_actual(i-1) = 0;
        end
        
        % 导纳控制
        F_error = cfg.F_desired - F_actual(i-1);
        integral = integral + F_error * cfg.dt;
        integral = max(min(integral, 1.0), -1.0);
        
        F_control = F_error + cfg.eta * integral;
        
        % 动力学
        a = (F_control - cfg.B * v(i-1) - cfg.K * (x(i-1) - cfg.z0)) / cfg.M;
        v(i) = v(i-1) + a * cfg.dt;
        x(i) = x(i-1) + v(i) * cfg.dt;
    end
    
    % 最后一点
    if x(end) <= cfg.z0
        F_actual(end) = cfg.K_env * (cfg.z0 - x(end));
    end
end

%% ========================================
%  生成学术论文级别图表
%% ========================================
function generate_academic_figures(theory, freq, time, closed, comparison)
    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    
    set(0, 'DefaultAxesFontName', 'Times New Roman');
    set(0, 'DefaultAxesFontSize', 11);
    set(0, 'DefaultTextFontName', 'Times New Roman');
    set(0, 'DefaultTextFontSize', 11);
    
    %% 图1: 理论分析（极点零点 + Bode图）
    figure('Position', [100, 100, 1200, 400]);
    
    % 极点零点图
    subplot(1, 3, 1);
    plot(real(theory.poles), imag(theory.poles), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    hold on;
    plot(real(theory.zeros), imag(theory.zeros), 'bo', 'MarkerSize', 12, 'LineWidth', 2);
    grid on; axis equal;
    xlabel('Real Axis'); ylabel('Imaginary Axis');
    title('(a) Pole-Zero Map');
    legend('Poles', 'Zeros', 'Location', 'best');
    
    % Bode幅值
    subplot(1, 3, 2);
    semilogx(freq.wout, 20*log10(freq.mag), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Frequency (rad/s)'); ylabel('Magnitude (dB)');
    title('(b) Bode Magnitude');
    
    % Bode相位
    subplot(1, 3, 3);
    semilogx(freq.wout, freq.phase, 'r-', 'LineWidth', 2);
    grid on;
    xlabel('Frequency (rad/s)'); ylabel('Phase (deg)');
    title('(c) Bode Phase');
    
    saveas(gcf, 'figures/Fig1_Theory.png');
    
    %% 图2: 开环时域响应
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(3, 2, 1);
    plot(time.t, time.F_step, 'k--', 'LineWidth', 1.5); hold on;
    plot(time.t, time.x_step*1000, 'b-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Position (mm)');
    title('(a) Step Response');
    grid on; legend('Input Force', 'Position', 'Location', 'best');
    
    subplot(3, 2, 2);
    plot(time.t, time.F_sin, 'k--', 'LineWidth', 1.5); hold on;
    plot(time.t, time.x_sin*1000, 'b-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Position (mm)');
    title('(b) Sinusoidal Response');
    grid on;
    
    subplot(3, 2, 3);
    plot(time.t, time.F_ramp, 'k--', 'LineWidth', 1.5); hold on;
    plot(time.t, time.x_ramp*1000, 'b-', 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Position (mm)');
    title('(c) Ramp Response');
    grid on;
    
    saveas(gcf, 'figures/Fig2_OpenLoop.png');
    
    %% 图3: 闭环力跟踪（核心图）
    figure('Position', [100, 100, 1200, 900]);
    
    % 力跟踪
    subplot(3, 1, 1);
    plot(closed.t, closed.F_desired, 'k--', 'LineWidth', 2); hold on;
    plot(closed.t, closed.F_actual, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Force (N)');
    title('(a) Force Tracking Performance');
    legend('Desired', 'Actual', 'Location', 'best');
    grid on;
    
    % 位置
    subplot(3, 1, 2);
    plot(closed.t, closed.x*1000, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(250, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Surface');
    xlabel('Time (s)'); ylabel('Z Position (mm)');
    title('(b) TCP Position');
    legend('Location', 'best');
    grid on;
    
    % 力误差
    subplot(3, 1, 3);
    plot(closed.t, closed.F_error, 'Color', [0.8, 0.4, 0], 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Force Error (N)');
    title(sprintf('(c) Force Error (RMSE=%.4f N, Steady-State=%.4f N)', ...
                  closed.RMSE, closed.steady_error));
    grid on;
    
    saveas(gcf, 'figures/Fig3_ClosedLoop_Tracking.png');
    
    %% 图4: 参数对比
    figure('Position', [100, 100, 1200, 500]);
    
    subplot(1, 2, 1);
    colors = lines(length(comparison.eta_values));
    for i = 1:length(comparison.eta_values)
        plot(comparison.t, comparison.F_all(:,i), 'Color', colors(i,:), ...
             'LineWidth', 1.5, 'DisplayName', sprintf('\\eta=%.0f', comparison.eta_values(i)));
        hold on;
    end
    yline(5.0, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xlabel('Time (s)'); ylabel('Contact Force (N)');
    title('(a) Effect of Integral Gain on Force Tracking');
    legend('Location', 'best');
    grid on;
    
    subplot(1, 2, 2);
    eta_plot = [comparison.metrics.eta];
    rmse_plot = [comparison.metrics.RMSE];
    bar(eta_plot, rmse_plot);
    xlabel('Integral Gain \eta'); ylabel('RMSE (N)');
    title('(b) Performance vs Integral Gain');
    grid on;
    
    saveas(gcf, 'figures/Fig4_Parameter_Comparison.png');
    
    fprintf('  保存 Fig1_Theory.png\n');
    fprintf('  保存 Fig2_OpenLoop.png\n');
    fprintf('  保存 Fig3_ClosedLoop_Tracking.png (核心)\n');
    fprintf('  保存 Fig4_Parameter_Comparison.png\n');
end