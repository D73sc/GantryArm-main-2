% MATLAB 机器人阻抗控制器仿真 (2DOF 平面机械臂)

clear;
clc;
close all;

%% 1. 机器人参数设置
% 连杆长度 (Link lengths)
L1 = 1; % m
L2 = 1.5; % m

% 连杆质量 (Link masses, simplified: assumed concentrated at end)
m1 = 1; % kg
m2 = 1; % kg

% 重力加速度 (Gravitational acceleration)
g = 9.81; % m/s^2

% 期望的初始关节位置 (Desired initial joint positions)
q0 = [pi/3; pi/4]; % rad

%% 2. 阻抗参数设置 (笛卡尔空间) (Impedance parameters setting in Cartesian space)
% 期望的惯性矩阵 Md (Desired Inertia Matrix Md)
% 通常选择对角矩阵，其对角线元素反映了在各个方向上对加速度的“抵抗”
% (Usually a diagonal matrix, its diagonal elements reflect the "resistance" to acceleration in each direction)
Md = diag([0.5, 0.5]); % kg, kg (for translation)

% 期望的阻尼矩阵 Bd (Desired Damping Matrix Bd)
% Bd 通常设置为 Md 的一个比例，以实现临界阻尼或欠阻尼
% (Bd is usually set as a proportion of Md to achieve critical or under-damping)
Kd_for_damping = diag([20, 20]); % Temporary Kd for damping calculation
Bd = 2 * sqrt(Md) * sqrt(Kd_for_damping); % Ns/m (e.g., for critical damping when Kd=diag([20,20]))

% 期望的刚度矩阵 Kd (Desired Stiffness Matrix Kd)
% Kd 决定了机器人对位置偏差的“恢复力”
% (Kd determines the robot's "restoring force" for position deviation)
Kd = diag([20, 20]); % N/m

% 期望的末端执行器参考位置 (自由空间中的期望位置) (Desired end-effector reference position (desired position in free space))
% 初始设置为机器人当前末端执行器位置
% (Initially set to the robot's current end-effector position)
[x0_initial, y0_initial] = forward_kinematics(q0, L1, L2);
x0 = x0_initial; 
y0 = y0_initial; 

%% 3. 仿真设置 (Simulation settings)
dt = 0.01; % 仿真步长 (Simulation step size)
T_end = 5; % 仿真总时长 (Total simulation time)
time = 0:dt:T_end;

% 初始化关节状态 (Initialize joint states)
q = q0;
dq = [0; 0];
ddq = [0; 0];

% 存储仿真数据 (Store simulation data)
q_hist = zeros(length(time), 2);
dq_hist = zeros(length(time), 2);
x_hist = zeros(length(time), 2);
Fext_hist = zeros(length(time), 2);
tau_hist = zeros(length(time), 2);
x_error_hist = zeros(length(time), 2);

%% 4. 仿真主循环 (Main simulation loop)
for i = 1:length(time)
    t = time(i);

    % --- 机器人正运动学和雅可比矩阵 --- (Robot forward kinematics and Jacobian matrix)
    [x, y] = forward_kinematics(q, L1, L2);
    J = jacobian_matrix(q, L1, L2);
    dJ = djacobian_matrix(q, dq, L1, L2);

    % 末端执行器速度 (End-effector velocity)
    dx = J * dq;

    % 末端执行器位置偏差 xe (End-effector position error xe)
    xe = x - x0; % x0 是期望的自由空间位置，这里我们保持它不变 (x0 is the desired free space position, we keep it constant here)
    ye = y - y0;

    % --- 外部力 Fext (模拟接触或环境力) --- (External force Fext (simulating contact or environmental force))
    F_ext = [0; 0]; % 初始没有外部力 (Initially no external force)

    % 在仿真2秒后，施加一个持续的外部力 (模拟与环境接触)
    % (After 2 seconds of simulation, apply a continuous external force (simulating contact with the environment))
    if t > 2 && t < 4
        F_ext = [10; 5]; % N (例如，在x方向推10N，y方向推5N) (e.g., pushing 10N in x-direction, 5N in y-direction)
    end
    
    % --- 阻抗控制律计算 --- (Impedance control law calculation)
    % 期望的末端执行器加速度 (从目标阻抗模型解出)
    % (Desired end-effector acceleration (derived from target impedance model))
    % M_d * ddx_cmd + B_d * dx_e + K_d * x_e = F_ext
    % ddx_cmd = inv(Md) * (F_ext - Bd * dx_e - Kd * x_e)
    % 注意：这里的 dx_e 是指 dx - dx0，但如果 x0 是固定点，则 dx0=0，所以 dx_e = dx
    % (Note: here dx_e refers to dx - dx0, but if x0 is a fixed point, then dx0=0, so dx_e = dx)
    ddx_cmd = Md \ (F_ext - Bd * dx - Kd *[xe;ye]); 
    
    % 期望的关节加速度 (通过雅可比逆变换)
    % (Desired joint acceleration (via Jacobian inverse transform))
    % ddx_cmd = J * ddq_cmd + dJ * dq
    % ddq_cmd = pinv(J) * (ddx_cmd - dJ * dq)
    ddq_cmd = pinv(J) * (ddx_cmd - dJ * dq);

    % --- 机器人动力学模型 --- (Robot dynamic model)
    % M(q)*ddq + C(q,dq)*dq + G(q) = tau + J'*F_ext
    M_q = mass_matrix(q, m1, m2, L1, L2);
    C_q_dq = coriolis_centripetal(q, dq, m2, L1, L2);
    G_q = gravity_vector(q, m1, m2, L1, L2, g);

    % --- 计算所需的关节力矩 --- (Calculate required joint torques)
    % 这里我们使用简化版的逆动力学，假设机器人能够精确跟踪ddq_cmd
    % (Here we use a simplified inverse dynamics, assuming the robot can accurately track ddq_cmd)
    % 真实的控制器会计算一个tau，使得实际ddq尽可能接近ddq_cmd
    % (A real controller would calculate a tau such that the actual ddq is as close as possible to ddq_cmd)
    % tau = M(q)*ddq_cmd + C(q,dq)*dq + G(q)
    tau = M_q * ddq_cmd + C_q_dq * dq + G_q;
    
    % 在实际仿真中，我们需要用tau驱动实际的机器人动力学
    % (In actual simulation, we need to drive the actual robot dynamics with tau)
    % M(q)*ddq = tau - C(q,dq)*dq - G(q) - J'*F_ext
    ddq = M_q \ (tau - C_q_dq * dq - G_q - J' * F_ext);

    % --- 关节状态积分 --- (Joint state integration)
    dq = dq + ddq * dt;
    q = q + dq * dt;

    % --- 存储数据 --- (Store data)
    q_hist(i, :) = q';
    dq_hist(i, :) = dq';
    x_hist(i, 1) = x';
    x_hist(i,2) = y';
    Fext_hist(i, :) = F_ext';
    tau_hist(i, :) = tau';
    x_error_hist(i, :) = xe';
end

%% 5. 仿真结果可视化 (Simulation results visualization)
figure;
subplot(4,1,1);
plot(time, q_hist(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time, q_hist(:,2), 'r', 'LineWidth', 1.5);
legend('q1', 'q2');
title('关节位置 (Joint Positions)');
xlabel('时间 (s) (Time (s))');
ylabel('角度 (rad) (Angle (rad))');
grid on;

subplot(4,1,2);
plot(time, x_hist(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hist(:,2), 'r', 'LineWidth', 1.5); hold on;
plot(time, repmat(x0(1), size(time)), 'b--', 'LineWidth', 1); hold on;
plot(time, repmat(y0(1), size(time)), 'r--', 'LineWidth', 1);
legend('X', 'Y', 'X_{ref}', 'Y_{ref}');
title('末端执行器位置 (End-Effector Positions)');
xlabel('时间 (s) (Time (s))');
ylabel('位置 (m) (Position (m))');
grid on;

subplot(4,1,3);
plot(time, Fext_hist(:,1), 'k--', 'LineWidth', 2); hold on;
plot(time, Fext_hist(:,2), 'm--', 'LineWidth', 2);
legend('F_{ext,x}', 'F_{ext,y}');
title('外部力 (External Forces)');
xlabel('时间 (s) (Time (s))');
ylabel('力 (N) (Force (N))');
grid on;

subplot(4,1,4)
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
    % 简化模型，更精确的模型会涉及转动惯量和连杆几何中心
    % (Simplified model, more precise models would involve moments of inertia and link centers of mass)
    % 此处为常见教科书中的简化2R机械臂质量矩阵
    % (This is the simplified 2R manipulator mass matrix found in common textbooks)
    
    % Consider mass at the end of the links
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