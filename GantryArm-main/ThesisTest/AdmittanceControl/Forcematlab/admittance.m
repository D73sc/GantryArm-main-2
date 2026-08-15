clear;
clc;
close all;

%% 1. 机器人参数设置
% 连杆长度 (Link lengths)
L1 = 1; % m
L2 = 1; % m
% 连杆质量 (Link masses, simplified: assumed concentrated at end)
m1 = 1; % kg
m2 = 1; % kg
% 重力加速度 (Gravitational acceleration)
g = 9.81; % m/s^2
% 初始关节位置 (Initial joint positions) - 注意这里是机器人实际的初始位置
q_initial = [pi/4; pi/4]; % rad

%% 2. 导纳参数设置 (笛卡尔空间)
% 期望的惯性矩阵 Md (Desired Inertia Matrix Md)
Md = diag([0.5, 0.5]); % kg (for translation)

% 期望的刚度矩阵 Kd (Desired Stiffness Matrix Kd)
Kd = diag([20, 20]); % N/m

% 期望的阻尼矩阵 Bd (Desired Damping Matrix Bd)
% Bd 通常设置为 Md 和 Kd 的一个比例，以实现临界阻尼或欠阻尼
Bd = 2 * sqrt(Md) * sqrt(Kd); % Ns/m

% 机器人末端执行器参考轨迹的初始位置 (自由空间中的期望静止点)
% 这个x0是导纳模型中，没有外部力时，机器人希望保持的“理想”位置
[x0_initial, y0_initial] = forward_kinematics(q_initial, L1, L2);
x0 = x0_initial; % 参考位置，这里设为初始时的末端执行器位置
y0 = y0_initial;

dx0 = 0;
dy0 =0;

% 导纳控制器内部的期望运动状态（这些是导纳模型积分出来的）
% xd, dxd, ddxd 是导纳控制器计算并输出给低层控制器的轨迹
xd = x0;      % 期望位置，初始与参考位置相同
dxd = 0; % 期望速度，初始为零
ddxd = 0; % 期望加速度，初始为零

yd = x0;      % 期望位置，初始与参考位置相同
dyd = 0; % 期望速度，初始为零
ddyd = 0; % 期望加速度，初始为零

%% 3. 仿真设置
dt = 0.01; % 仿真步长 (Simulation step size)
T_end = 5; % 仿真总时长 (Total simulation time)
time = 0:dt:T_end;

% 初始化机器人实际状态 (这些是机器人实际运动的状态)
q_actual = q_initial;
dq_actual = [0; 0];
ddq_actual = [0; 0]; % 假设初始静止

% 存储仿真数据 (Store simulation data)
q_hist = zeros(length(time), 2);
dq_hist = zeros(length(time), 2);
x_actual_hist = zeros(length(time), 2);
Fext_hist = zeros(length(time), 2);
tau_hist = zeros(length(time), 2); % 实际施加的力矩
xd_hist = zeros(length(time), 2);
x_error_hist = zeros(length(time), 2); % 实际位置与参考位置的误差

%% 4. 仿真主循环
for i = 1:length(time)
    t = time(i);

    % --- 机器人实际状态 (模拟真实机器人反馈) ---
    [x_actual, y_actual] = forward_kinematics(q_actual, L1, L2);
    J_actual = jacobian_matrix(q_actual, L1, L2); % 实际雅可比

    xe = xd - x0;
    ye = yd - y0;

    dxe = dxd - dx0;
    dye = dyd - dy0;

    % --- 外部力 Fext (模拟接触或环境力) ---
    F_ext = [0; 0]; % 初始没有外部力
    % 在仿真2秒后，施加一个持续的外部力 (模拟与环境接触)
    if t > 2 && t < 4
        F_ext = [10; 5]; % N (例如，在x方向推10N，y方向推5N)
    end

    % --- 导纳控制律计算 (外环) ---
    % 计算期望加速度 ddxd
    % M_d * (ddxd - ddxd_0) + B_d * (dxd - dxd_0) + K_d * (xd - x0) = F_ext
    % 假设 ddxd_0 = 0, dxd_0 = 0

    damp_xy = [dxe;dye];
    stiff_xy = [xe; ye];
    
    ddxd = Md \ (F_ext - Bd *damp_xy  - Kd *stiff_xy);

    % 积分计算期望速度 dxd 和期望位置 xd (欧拉积分)
    dxd = dxd + ddxd(1) * dt;
    xd = xd + dxd * dt;

    dyd = dyd+ddxd(2)*dt;
    yd = yd +dyd*dt;

    % --- 将期望笛卡尔位置/速度映射到关节空间 (低层位置控制器输入) ---
    % 在实际系统中，这里会进行逆运动学计算得到 qd，
    % 然后将 qd 作为指令发送给机器人底层的关节位置控制器。
    % 简化的低层位置控制（假设能够完美跟踪期望笛卡尔位置xd）
    % 这里我们直接通过伪逆雅可比来计算期望关节速度和加速度
    % 更真实的导纳控制器通常是输出xd，然后由机器人内部的运动规划器和关节控制器来执行

    % 为了模拟实际机器人对xd的跟踪，我们假设有一个P控制器或者更复杂的控制器
    % 这里我们用一个简化的PD控制器作为底层跟踪器，目标是跟踪xd
    Kp_low = diag([500, 500]); % 低层位置环P增益
    Kv_low = diag([50, 50]);   % 低层速度环D增益

    % 实际末端执行器速度
    dx_actual = J_actual * dq_actual;

    x_err = xd - x_actual;
    y_err = yd - y_actual;
    err_xy =[x_err;y_err];
    dx_err = dxd - dx_actual(1);
    dy_err = dyd - dx_actual(2);
    derr_xy = [dx_err;dy_err];

    % 计算低层控制器的末端执行器指令
    % 这里的tau是低层控制器计算的力矩，不是导纳控制器直接输出的力矩
    % 这是一个简化的力矩指令，模拟低层控制器试图让机器人跟踪xd
    tau_low_level = J_actual' * (Kp_low * err_xy + Kv_low * derr_xy);

    % --- 机器人动力学模型 (模拟真实机器人运动) ---
    % M(q)*ddq + C(q,dq)*dq + G(q) = tau + J'*F_ext
    M_q = mass_matrix(q_actual, m1, m2, L1, L2);
    C_q_dq = coriolis_centripetal(q_actual, dq_actual, m2, L1, L2);
    G_q = gravity_vector(q_actual, m1, m2, L1, L2, g);

    % 计算实际关节加速度 (由低层力矩和外部力驱动)
    ddq_actual = M_q \ (tau_low_level - C_q_dq * dq_actual - G_q - J_actual' * F_ext);

    % --- 关节状态积分 (模拟机器人运动) ---
    dq_actual = dq_actual + ddq_actual * dt;
    q_actual = q_actual + dq_actual * dt;

    % --- 存储数据 ---
    q_hist(i, :) = q_actual';
    dq_hist(i, :) = dq_actual';
    x_actual_hist(i, 1) = x_actual';
    x_actual_hist(i, 2) = y_actual';
    Fext_hist(i, :) = F_ext';
    tau_hist(i, :) = tau_low_level'; % 存储低层控制器输出的力矩
    xd_hist(i, 1) = xd';
    xd_hist(i,2) = yd';
    x_error_hist(i, 1) = (x_actual - x0)'; % 实际位置与参考位置的误差
    x_error_hist(i, 2) = (y_actual - y0)';
end

%% 5. 仿真结果可视化
% figure;
subplot(4,1,1);
plot(time, q_hist(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time, q_hist(:,2), 'r', 'LineWidth', 1.5);
legend('q1', 'q2');
title('关节位置 (Joint Positions)');
xlabel('时间 (s) (Time (s))');
ylabel('角度 (rad) (Angle (rad))');
grid on;
% 
subplot(4,1,2);
plot(time, x_actual_hist(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_actual_hist(:,2), 'r', 'LineWidth', 1.5);
plot(time, xd_hist(:,1), 'b--', 'LineWidth', 1);
plot(time, xd_hist(:,2), 'r--', 'LineWidth', 1);
plot(time, repmat(x0, size(time)), 'k:', 'LineWidth', 1); % 参考轨迹x0
plot(time, repmat(y0, size(time)), 'm:', 'LineWidth', 1); % 参考轨迹x0
legend('X_{actual}', 'Y_{actual}', 'X_{desired}', 'Y_{desired}', 'X_{ref}', 'Y_{ref}');
title('末端执行器位置 (End-Effector Positions)');
xlabel('时间 (s) (Time (s))');
ylabel('位置 (m) (Position (m))');
grid on;
% 

subplot(4,1,3);
plot(time, Fext_hist(:,1), 'k--', 'LineWidth', 1.5); hold on;
plot(time, Fext_hist(:,2), 'm--', 'LineWidth', 1.5);
legend('F_{ext,x}', 'F_{ext,y}');
title('外部力 (External Forces)');
xlabel('时间 (s) (Time (s))');
ylabel('力 (N) (Force (N))');
grid on;

subplot(4,1,4);
plot(time, tau_hist(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time, tau_hist(:,2), 'r', 'LineWidth', 1.5);
legend('\tau_1', '\tau_2');
title('外部关节力矩 (External Joint Torques)');
xlabel('时间 (s) (Time (s))');
ylabel('力矩 (Nm) (Torque (Nm))');
grid on;

%% 辅助函数定义 (Auxiliary Function Definitions)

% 正运动学 (Forward Kinematics)
function [x_ee, y_ee] = forward_kinematics(q, L1, L2)
    x_ee = L1 * cos(q(1)) + L2 * cos(q(1) + q(2));
    y_ee = L1 * sin(q(1)) + L2 * sin(q(1) + q(2));
end

% 雅可比矩阵 (Jacobian Matrix)
function J = jacobian_matrix(q, L1, L2)
    J = [-L1*sin(q(1)) - L2*sin(q(1)+q(2)), -L2*sin(q(1)+q(2));
          L1*cos(q(1)) + L2*cos(q(1)+q(2)),  L2*cos(q(1)+q(2))];
end

% 雅可比矩阵的导数 (Derivative of Jacobian Matrix)
% 注意：在导纳控制中，我们通常不直接使用 dJ 来计算关节加速度，
% 因为导纳控制器输出的是期望位置/速度，而不是期望加速度。
% 但这里为了仿真机器人实际动力学，J_actual 的计算仍需 dJ。
function dJ = djacobian_matrix(q, dq, L1, L2)
    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);

    dJ11 = -L1*cos(q1)*dq1 - L2*cos(q1+q2)*(dq1+dq2);
    dJ12 = -L2*cos(q1+q2)*(dq1+dq2);
    dJ21 = -L1*sin(q1)*dq1 - L2*sin(q1+q2)*(dq1+dq2);
    dJ22 = -L2*sin(q1+q2)*(dq1+dq2);

    dJ = [dJ11, dJ12;
          dJ21, dJ22];
end

% 关节空间质量矩阵 (Joint Space Mass Matrix - Simplified)
function M_q = mass_matrix(q, m1, m2, L1, L2)
    m11 = (m1+m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q(2));
    m12 = m2*L2^2 + m2*L1*L2*cos(q(2));
    m21 = m12;
    m22 = m2*L2^2;

    M_q = [m11, m12;
           m21, m22];
end

% 科里奥利力和向心力矩阵 (Coriolis and Centripetal Force Matrix)
function C_q_dq = coriolis_centripetal(q, dq, m2, L1, L2)
    h = m2*L1*L2*sin(q(2));

    c11 = -h*dq(2);
    c12 = -h*(dq(1)+dq(2));
    c21 = h*dq(1);
    c22 = 0;

    C_q_dq = [c11, c12;
              c21, c22];
end

% 重力向量 (Gravity Vector)
function G_q = gravity_vector(q, m1, m2, L1, L2, g)
    g1 = (m1+m2)*g*L1*cos(q(1)) + m2*g*L2*cos(q(1)+q(2));
    g2 = m2*g*L2*cos(q(1)+q(2));

    G_q = [g1; g2];
end