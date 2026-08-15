clc;
clear;

%% ================== 符号定义部分 (DH参数建模) ==================
syms nx ny nz ox oy oz ax ay az px py pz real   % 目标位姿矩阵参数
syms d0 d1 d2 theta3 theta4 theta5 theta6 theta7 real
syms Load real
global Load;
Load=100;
radian1 = pi/180;
% 定义MDH参数
a = sym([0,0,0,90, 200, 0,0, 0]);        % 连杆长度
alpha = sym([pi/2,pi/2,pi/2,0, pi/2, pi/2, pi/2, pi/2]); % 扭角
d = sym([d0,d1,d2,-160-135.5, 0, 152.5+228.5,0, 129+Load]); % 偏移
theta = [pi/2,-pi/2,pi/2,theta3, theta4 + pi/2, theta5+ pi, theta6+ pi, theta7+ pi]; % 关节角
%定义关节角度限制
lim0_min = 0 ; lim0_max = 2500 ; %关节1(-170，170)
lim1_min = 0 ; lim1_max = 3000 ; %关节1(-170，170)
lim2_min = 0 ; lim2_max = 2000 ; %关节1(-170，170)
lim3_min = -180 * radian1; lim3_max = 180 * radian1; %关节1(-170，170)
lim4_min = -75 * radian1; lim4_max =   75 * radian1; %关节2(-132，0)
lim5_min = -180 * radian1; lim5_max = 180 * radian1; %关节3(1，141)
lim6_min = -120 * radian1; lim6_max = 120 * radian1; %关节5(-105，105)
lim7_min = -180 * radian1; lim7_max = 180 * radian1; %关节5(-105，105)

% 构造目标位姿矩阵
T_goal = [nx ox ax px;
          ny oy ay py;
          nz oz az pz;
          0  0  0  1];
T08 = eye(4);  % Store T0i (Transformation matrix from base to joint i)
% Compute each homogeneous transformation matrix T0i and Ti,i-1, then output the formula
for i = 1:8
    % MDH formula to compute each transformation matrix Ti,i-1
    Ti_i_minus_1 = simplify([
        cos(theta(i)), -sin(theta(i)), 0, a(i);
        sin(theta(i)) * cos(alpha(i)), cos(theta(i)) * cos(alpha(i)), -sin(alpha(i)), -d(i) * sin(alpha(i));
        sin(theta(i)) * sin(alpha(i)), cos(theta(i)) * sin(alpha(i)), cos(alpha(i)), d(i) * cos(alpha(i));
        0, 0, 0, 1;
    ]);

    % Update the total transformation matrix T0i (from base to joint i)
    T08 = T08 * Ti_i_minus_1;
end
    % 生成数值计算函数
    T08_numeric = matlabFunction(T08,...
       'Vars', {'d0','d1','d2','theta3','theta4','theta5','theta6','theta7','Load'},...
       'Outputs', {'T08'});
   %% ================ 生成批量测试样本 ================
    N = 1000000;  % 样本数量
    tolerance = 1e-6;  % 矩阵比较容差
    % 创建各参数的边界值数组（列向量）
    limit_min = [lim0_min; lim1_min; lim2_min; 
                lim3_min; lim4_min; lim5_min; lim6_min; lim7_min];
    limit_max = [lim0_max; lim1_max; lim2_max; 
                lim3_max; lim4_max; lim5_max; lim6_max; lim7_max];
    range = limit_max - limit_min;
    
    % 生成随机参数矩阵
    rng('default'); % 保证结果可重复
    param_matrix = limit_min' + (limit_max - limit_min)' .* rand(N, 8);
    % 预分配结果存储
    result_cell = cell(N,24);  % 最终存储Excel数据
    success_count = 0;         % 成功计数器
    valid_positions = [];      % 存储有效点坐标
    % param_matrix每列对应参数顺序：
    % [d0, d1, d2, theta3, theta4, theta5, theta6, theta7]
%% ================ 批量测试循环 ================
% 在循环前添加以下代码启用多线程
if maxNumCompThreads > 1
    parpool('local'); % 需要Parallel Computing Toolbox
end
for n = 1:N
    % 解析当前参数
    current_params = num2cell(param_matrix(n, :));
    [d0_val, d1_val, d2_val, theta3_val, theta4_val, theta5_val, theta6_val, theta7_val] = deal(current_params{:});
    
    % 正解计算
    T_goal = T08_numeric(d0_val, d1_val, d2_val,...
                        theta3_val, theta4_val, theta5_val,...
                        theta6_val, theta7_val, Load); % 生成时theta6/7暂时用0
    
    % 提取目标姿态参数
    ax=  T_goal(1,3); 
    ay=  T_goal(2,3); 
    az=  T_goal(3,3); 

    px=  T_goal(1,4);
    py=  T_goal(2,4);
    pz=  T_goal(3,4);
    
    oz = T_goal(3,2);  % 注意MATLAB旋转矩阵定义
    nz = T_goal(3,1);  % 确认与机械臂定义一致
    % 逆解计算
    [d0_ik] = d0_solver(theta3_val, theta4_val, py, ay, Load);
    [d1_ik] = d1_solver(theta3_val, theta4_val, px, ax, Load);
    [d2_ik] = d2_solver(theta4_val, pz, az, Load);
    [theta6_sols, theta7_sols] = deal(zeros(2,1));
    try
        [theta6_sols(1), theta6_sols(2)] = theta6_solver(theta4_val, theta5_val, az);
        [theta7_sols(1), theta7_sols(2)] = theta7_solver(theta4_val, theta5_val, oz, nz);
    catch
        % 处理奇异性情况
        theta6_sols = [nan; nan];
        theta7_sols = [nan; nan];
    end
    
    % 四组可能解的正逆向验证
    solution_valid = false(4,1);
    joint_limits_valid = false(4,1);
    
    for sol_num = 1:4
        % 选择解组合
        [t6_idx, t7_idx] = ind2sub([2,2], sol_num);
        theta6_ik = theta6_sols(t6_idx);
        theta7_ik = theta7_sols(t7_idx);
        
        % 检查关节限位
        limits_ok = check_joint_limits([d0_ik, d1_ik, d2_ik,...
                                       theta3_val, theta4_val, theta5_val,...
                                       theta6_ik, theta7_ik],...
                                       limit_min, limit_max);
        
        % 计算逆向位姿
        try
            T_ik = T08_numeric(d0_ik, d1_ik, d2_ik,...
                              theta3_val, theta4_val, theta5_val,...
                              theta6_ik, theta7_ik, Load);
            pos_error = norm(T_goal(1:3,4) - T_ik(1:3,4));
            rot_error = norm(T_goal(1:3,1:3) - T_ik(1:3,1:3), 'fro');
            solution_valid(sol_num) = (pos_error < 1e-3) && (rot_error < 1e-3);
        catch
            solution_valid(sol_num) = false;
        end
        joint_limits_valid(sol_num) = all(limits_ok);
    end
    
    % 综合判断是否成功
    is_success = any(joint_limits_valid & solution_valid);
    success_count = success_count + is_success;
    
    % 记录有效点
    if is_success
        valid_positions = [valid_positions; T_goal(1:3,4)'];
    end
    
    % 构建数据行
    result_row = {...
        n, d0_val, d1_val, d2_val,...
        rad2deg(theta3_val),rad2deg( theta4_val),rad2deg( theta5_val),...
        rad2deg(theta6_val),rad2deg( theta7_val),...
        px,py,pz,... 
        d0_ik, d1_ik, d2_ik,...
        rad2deg(theta6_sols(1)),rad2deg(theta6_sols(2)),  rad2deg(theta7_sols(1)), rad2deg(theta7_sols(2)),...
        joint_limits_valid(1),joint_limits_valid(2),solution_valid(1),solution_valid(2),is_success...
    };
    result_cell(n,:) = result_row;
    
    fprintf('Sample %d: %s\n', n, string(is_success));
end

%% ================ 统计与输出 ================
% 样本成功率
success_rate = success_count / N * 100;
fprintf('\n总成功率: %.1f%%\n', success_rate);

% 表格列标题设置
column_names = {...
    'SampleID', 'd0_original', 'd1_original', 'd2_original',...
    'theta3_deg', 'theta4_deg', 'theta5_deg',...
    'theta6_deg','theta7_deg',...
    'px', 'py', 'pz',...
    'd0_ik', 'd1_ik', 'd2_ik',...
    'theta6_1', 'theta6_2', 'theta7_1', 'theta7_2',...
    'ValidSolution_1', 'ValidSolution_2', 'ValidSolution_3', 'ValidSolution_4',...
    'IsSuccess'...
};

% 转换为表格并保存Excel
result_table = cell2table(result_cell, 'VariableNames', column_names);
writetable(result_table, 'F:\5DOFGantryArm (3)\5DOFGantryArmMatlab\InverseKinematics_Validation.xlsx');

%% ================ 可视化 ================
figure('Name', '工作空间分析', 'Position', [100 100 1200 500])
subplot(1,2,1)
scatter3(valid_positions(:,1), valid_positions(:,2), valid_positions(:,3),...
    15, 'filled', 'MarkerFaceAlpha',0.6)
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
title('有效逆解末端位置分布')
grid on; axis equal;

subplot(1,2,2)
success_rates = [success_rate, 100-success_rate];
pie(success_rates, {'成功', '失败'})
title(sprintf('总成功率 %.1f%%', success_rate))

% ==== 关节限位检查函数 ====
function is_valid = check_joint_limits(params, min_limits, max_limits)
    is_valid = (params >= min_limits') & (params <= max_limits');
end

%% ================ 主要计算函数 ================
function [d0_value] = d0_solver(theta3,theta4, py, ay,Load)
    cosTheta4 = boundedTrig(@cos, theta4);
    sinTheta3 = boundedTrig(@sin, theta3);
    

    d0_value=-200*sinTheta3-381*sinTheta3*cosTheta4-py+ay*(Load+129);
    fprintf('d0: %.2f\n',d0_value);
end

function [d1_value] = d1_solver(theta3,theta4, px, ax,Load)
    cosTheta4 = boundedTrig(@cos, theta4);
    cosTheta3 = boundedTrig(@cos, theta3);

    d1_value=px-ax*(Load+129)-200*cosTheta3-381*cosTheta3*cosTheta4-90;
    fprintf('d1: %.2f\n',d1_value);
end

function [d2_value] = d2_solver(theta4, pz, az,Load)
    sinTheta4 = boundedTrig(@sin, theta4);
    d2_value=591/2-381*sinTheta4-pz+az*(Load+129);
    fprintf('d2: %.2f\n',d2_value);
end

function [theta6_1, theta6_2] = theta6_solver(theta4, theta5, az)
    % 使用数值替换后的变量进行计算
    c4 = cos(theta4);
    s4 = sin(theta4);
    c5 = cos(theta5);
    term = (c4*c5)^2 + s4^2 - az^2;
    if term < 0
        error('[theta6错误] 无实数解：(c4*c5)^2 + s4^2 - az^2 = %.2f < 0', term);
    end
    % 符号选择传递性计算
    delta_plus = sqrt(term);
    delta_minus = -delta_plus;

    % theta6解的形式
    theta6_1  = atan2(-az, delta_plus) - atan2(s4, c4*c5);
    theta6_2 = atan2(-az, delta_minus) - atan2(s4, c4*c5);
    theta6_1=wrapToPi(theta6_1);
    theta6_2=wrapToPi(theta6_2);

    % 检查奇异性条件（接近0时视为奇异）
    tolerance_singular = 1e-6;
    if abs(s4) < tolerance_singular && (abs(c4*c5) < tolerance_singular)
        warning('[theta6警告] 奇异性条件成立：s4=%.2e, c4*c5=%.2e', s4, c4*c5);
    end
    
    % 整理θ6解集
    if abs(theta6_1 - theta6_2) < 1e-6
        theta6_solutions = (theta6_1);
    else
        theta6_solutions = ([(theta6_1), (theta6_2)]);
    end
    
    fprintf('theta6解 (度): 解1: %.2f°  |  解2: %.2f°\n', ...
    wrapTo180(rad2deg(theta6_solutions(1))), wrapTo180(rad2deg(theta6_solutions(end))));
end

function [theta7_1, theta7_2] = theta7_solver(theta4, theta5, oz, nz)
    % 参数安全处理
    c4 = cos(theta4);
    s5 = sin(theta5); 
    oz = real(oz); nz = real(nz);
    
    % 计算参考量
    R = sqrt(oz^2 + nz^2);
    threshold = (c4 * s5) / R;
    
    % 奇异性管理
    if abs(threshold) > 1
        fprintf('当前阈值: %.4f，将进行剪切处理\n', threshold);
        threshold = sign(threshold) * 1;
    end
    
    % 相位计算
    phi = (atan2(-oz, -nz)); % 使用0~2pi范围
    
    % 基础角度计算
    base_angle = asin(threshold);
    
    % 多个解的生成
    theta7_1 = (-phi + base_angle);
    theta7_2 = (-phi + pi - base_angle);
    theta7_1=wrapToPi(theta7_1);
    theta7_2=wrapToPi(theta7_2);    
    
    % 解的输出
    
    theta7_solutions = struct(...
        'Solution1', wrapTo180(rad2deg(theta7_1)),...
        'Solution2', wrapTo180(rad2deg(theta7_2)));
    fprintf('theta7解 (度): 解1: %.2f°  |  解2: %.2f°\n', ...
    theta7_solutions.Solution1, theta7_solutions.Solution2);
end

%% ============== 辅助工具函数 ================
% 安全三角函数计算
function val = boundedTrig(func, angle)
    raw_val = func(angle);
    if abs(raw_val) > 1
        if abs(raw_val) - 1 > 1e-6
            warning('调整三角函数值 %.4f 到 ±1', raw_val);
        end
        val = sign(raw_val) * 1;
    else
        val = real(raw_val);
    end
end

% 角度范围限制函数
function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function angle = wrapTo2Pi(angle)
    angle = mod(angle, 2*pi);
end

function deg = wrapTo180(deg)
    deg = mod(deg + 180, 360) - 180;
end