clc;
clear;

%% ================== 符号定义部分 (DH参数建模) ==================
syms nx ny nz ox oy oz ax ay az px py pz real   % 目标位姿矩阵参数
syms d0 d1 d2 theta3 theta4 theta5 theta6 theta7 real
syms Load real
global Load;
Load=10;
radian1 = pi/180;
% 定义MDH参数
a = sym([0,0,0,0, 200, 0,0, 0]);        % 连杆长度
alpha = sym([pi/2,pi/2,pi/2,0, pi/2, pi/2, pi/2, pi/2]); % 扭角
d = sym([d0,d1,d2,160+135.5, 0, 152.5+228.5,0, 129+Load]); % 偏移
theta = [pi/2,-pi/2,pi/2,theta3, theta4 + pi/2, theta5+ pi, theta6+ pi, theta7+ pi]; % 关节�??
%定义关节角度限制
lim0_min = 0 ; lim0_max = 2500 ; %关节1(-170�??170)
lim1_min = 0 ; lim1_max = 3000 ; %关节1(-170�??170)
lim2_min = 0 ; lim2_max = 2000 ; %关节1(-170�??170)
lim3_min = -180 * radian1; lim3_max = 180 * radian1; %关节1(-170�??170)
lim4_min = -75 * radian1; lim4_max =   75 * radian1; %关节2(-132�??0)
lim5_min = -180 * radian1; lim5_max = 180 * radian1; %关节3(1�??141)
lim6_min = -120 * radian1; lim6_max = 120 * radian1; %关节5(-105�??105)
lim7_min = -180 * radian1; lim7_max = 180 * radian1; %关节5(-105�??105)

% 构�?�目标位姿矩�??
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
    % 生成数�?�计算函�??
    T08_numeric = matlabFunction(T08,...
       'Vars', {'d0','d1','d2','theta3','theta4','theta5','theta6','theta7','Load'},...
       'Outputs', {'T08'});
   %% ================ 生成批量测试样本 ================
    N = 20;  % 样本数量
    tolerance = 1e-6;  % 矩阵比较容差
    % 创建各参数的边界值数组（列向量）
    limit_min = [lim0_min; lim1_min; lim2_min; 
                lim3_min; lim4_min; lim5_min; lim6_min; lim7_min];
    limit_max = [lim0_max; lim1_max; lim2_max; 
                lim3_max; lim4_max; lim5_max; lim6_max; lim7_max];
    range = limit_max - limit_min;
    
    % 生成随机参数矩阵
    rng('default'); % 保证结果可重�??
    param_matrix_rand = limit_min' + (limit_max - limit_min)' .* rand(N, 8);
    % 预分配结果存�??
    success_count = 0;         % 成功计数�??
    valid_positions = [];      % 存储有效点坐�??

%% ================ 批量测试循环 ================
% 在循环前添加以下代码启用多线
% if maxNumCompThreads > 1
%     parpool('local'); 
% end
joint_solutions = struct(...
    'theta3',   [], ...
    'theta4',   [], ...
    'theta5',   [], ...
    'theta6',   [], ...
    'theta7',   [], ...
    'd0',       [], ...
    'd1',       [], ...
    'd2',       [], ...
    'is_valid', false, ...
    'errors',   [Inf, Inf],...
    'limits_ok',false);
for n = 11:N
    current_params = num2cell(param_matrix_rand(n, :));
    [d0_val, d1_val, d2_val, theta3_val, theta4_val, theta5_val, theta6_val, theta7_val] = deal(current_params{:});
    
    T_goal = T08_numeric(d0_val, d1_val, d2_val,...
                        theta3_val, theta4_val, theta5_val,...
                        theta6_val, theta7_val, Load); 
    
    ax=  T_goal(1,3); 
    ay=  T_goal(2,3); 
    az=  T_goal(3,3); 

    px=  T_goal(1,4);
    py=  T_goal(2,4);
    pz=  T_goal(3,4);
    
    ox = T_goal(1,2);  
    nx = T_goal(1,1); 
    
    oy = T_goal(2,2);  
    ny = T_goal(2,1); 

    oz = T_goal(3,2);  
    nz = T_goal(3,1); 
    
    
    %% ================ decision space ================
    step = 100; % 调节步进以提高效�??
    offset=0.01;
    theta4_values = linspace(lim4_min+offset, lim4_max-offset,step);
    theta7_values = linspace(lim7_min+offset, lim7_max-offset,step);
    [grid_theta4, grid_theta7] = ndgrid(theta4_values, theta7_values);
        param_matrix = [zeros(size(grid_theta4(:))) ... % d0
                    zeros(size(grid_theta4(:))) ... % d1
                    zeros(size(grid_theta4(:))) ... % d2
                    zeros(size(grid_theta4(:))) ... % theta3
                    grid_theta4(:) grid_theta4(:) ...
                    zeros(numel(grid_theta4(:)),2)]; % theta6/7
    N_ds = size(param_matrix, 1);
    theta7_ds=[];
    theta4_ds=[];
    theta5_ds=[];
    success_grid=0;

    for n = 1:N_ds
        theta4_val=grid_theta4(n);
        theta7_val=grid_theta7(n);
        
    %% ================ calculate ================
    
    

        [theta5_sols,theta6_sols] = deal(zeros(2,1));
        try
            [theta5_sols(1), theta5_sols(2)] = theta5_solver(theta4_val, theta7_val, oz,nz);
        catch
            continue;
        end
        total_solutions = 0;
        for t5_idx = 1:2
            theta5_ik=theta5_sols(t5_idx);
            try
                [theta6_sols(1), theta6_sols(2)] = theta6_solver(theta4_val, theta5_ik, az);
            catch
                continue;
            end
            for t6_idx = 1:2
                theta6_ik = theta6_sols(t6_idx);
                try
                    [theta3_1, theta3_2] = theta3_solver2(theta4_val, theta5_ik, theta7_val, oy, ny);
                    theta3_candidates = [theta3_1, theta3_2]; 
                    for t3_case = 1:2
                        theta3_ik = theta3_candidates(t3_case); 

                        total_solutions = total_solutions + 1;
                        joint_solutions(total_solutions).theta3 = theta3_ik;
                        joint_solutions(total_solutions).theta5  = theta5_ik;
                        joint_solutions(total_solutions).theta6  = theta6_ik;
                    end
                catch
                    continue;
                end
            end
        end    

        for sol_idx = 1:total_solutions
            joint_solutions(sol_idx).theta4=theta4_val;
            joint_solutions(sol_idx).theta7=theta7_val;
            sol = joint_solutions(sol_idx);
            theta3_ik = joint_solutions(sol_idx).theta3;
            theta6_ik = joint_solutions(sol_idx).theta6;
            theta5_ik = joint_solutions(sol_idx).theta5;
            try
                d0_ik = d0_solver(sol.theta3, theta4_val, py, ay, Load);
                d1_ik = d1_solver(sol.theta3, theta4_val, px, ax, Load);
                d2_ik = d2_solver(theta4_val, pz, az, Load);
                sol.d0 = d0_ik; sol.d1 = d1_ik; sol.d2 = d2_ik;
            catch
                continue; 
            end
            joint_solutions(sol_idx).d0 = d0_ik;
            joint_solutions(sol_idx).d1 = d1_ik;
            joint_solutions(sol_idx).d2 = d2_ik;
            
            joint_values = [d0_ik, d1_ik, d2_ik, sol.theta3, sol.theta4, sol.theta5, sol.theta6, sol.theta7];
            joint_solutions(sol_idx).limits_ok = check_joint_limits(joint_values, limit_min, limit_max);        



            try
                T_ik = T08_numeric(d0_ik, d1_ik, d2_ik, sol.theta3, sol.theta4, sol.theta5, sol.theta6, sol.theta7, Load);
                pos_error = norm(T_goal(1:3,4) - T_ik(1:3,4));
                rot_error = norm(T_goal(1:3,1:3) - T_ik(1:3,1:3), 'fro');
                joint_solutions(sol_idx).errors = [pos_error, rot_error];
                joint_solutions(sol_idx).is_valid = all([pos_error < 1e-3, rot_error < 1e-3]) && all(joint_solutions(sol_idx).limits_ok);
            catch
                joint_solutions(sol_idx).is_valid = false;
            end

        end

        valid_solutions = joint_solutions([joint_solutions.is_valid]);
        unique_valid_solutions = remove_duplicate_solutions(valid_solutions);
        success_count=0;
        for sol = unique_valid_solutions
            success_count=success_count+1;
%             fprintf('d0=%.2f, d1=%.2f, d2=%.2f,theta3=%.2f,theta4=%.2f, theta5=%.2f, theta6=%.2f, theta7=%.2f\n', ...
%                 sol.d0, sol.d1, sol.d2,sol.theta3,sol.theta4, sol.theta5, sol.theta6,sol.theta7 );
            theta7_ds=[theta7_ds,sol.theta7/radian1];
            theta4_ds=[theta4_ds,sol.theta4/radian1];
            theta5_ds=[theta5_ds,sol.theta5/radian1];
            
        end
%         fprintf('There are %d sets of solutions\n', success_count);
        if success_count>0
                success_grid=success_grid+1;
        end
    %% ================ decision space ================
    end
        fprintf('success_grid��%d\n', success_grid);
        disp(T_goal);
    % 绘制三维散点�??
    figure('Position', [100, 100, 800, 600]);
    scatter3( theta4_ds, theta7_ds, theta5_ds,40, 'filled', 'MarkerEdgeColor', 'k');
    xlabel('Theta4 (deg)');
    ylabel('Theta7 (deg)');
    zlabel('Theta5 (deg)');
    title('Theta4-7-5');
    grid on;
    view(45, 30); % 调整视角
    
end

%% ================ 统计与输�?? ================

% 转换为表格并保存Excel
% result_table = cell2table(result_cell, 'VariableNames', column_names);
% writetable(result_table, 'F:\5DOFGantryArm (3)\5DOFGantryArmMatlab\InverseKinematics_Validation_345.xlsx');

%% ================ 可视�?? ================
% figure('Name', '工作空间分析', 'Position', [100 100 1200 500])
% subplot(1,2,1)
% scatter3(valid_positions(:,1), valid_positions(:,2), valid_positions(:,3),...
%     15, 'filled', 'MarkerFaceAlpha',0.6)
% xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
% title('有效逆解末端位置分布')
% grid on; axis equal;
% 
% subplot(1,2,2)
% success_rates = [success_rate, 100-success_rate];
% pie(success_rates, {'成功', '失败'})
% title(sprintf('总成功率 %.1f%%', success_rate))

% ==== 关节限位�??查函�?? ====
function is_valid = check_joint_limits(params, min_limits, max_limits)
    is_valid = (params >= min_limits') & (params <= max_limits');
end
function unique_solutions = remove_duplicate_solutions(solutions)

    tol_position = 1e-4;  
    tol_angle    = 1e-4;  
    
    if isempty(solutions)
        unique_solutions = [];
        return;
    end
    
    param_matrix = [...
        [solutions.d0]', [solutions.d1]', [solutions.d2]', ...
        [solutions.theta3]', [solutions.theta4]', ...
        [solutions.theta5]', [solutions.theta6]', [solutions.theta7]' ...
    ];
    
    normalize = @(x) mod(x + pi, 2*pi) - pi;
    param_matrix(:,4:8) = normalize(param_matrix(:,4:8));
    
    scaled_pos = round(param_matrix(:,1:3) / tol_position); 
    scaled_ang = round(param_matrix(:,4:8) / tol_angle);
    scaled_matrix = [scaled_pos, scaled_ang];
    
    [~, unique_idx] = unique(scaled_matrix, 'rows', 'stable');
    unique_solutions = solutions(unique_idx);
end
    

%% ================ 主要计算函数 ================
function [d0_value] = d0_solver(theta3,theta4, py, ay,Load)
    cosTheta4 = boundedTrig(@cos, theta4);
    sinTheta3 = boundedTrig(@sin, theta3);
    

    d0_value=-200*sinTheta3-381*sinTheta3*cosTheta4-py+ay*(Load+129);
%     fprintf('d0: %.2f\n',d0_value);
end

function [d1_value] = d1_solver(theta3,theta4, px, ax,Load)
    cosTheta4 = boundedTrig(@cos, theta4);
    cosTheta3 = boundedTrig(@cos, theta3);

    d1_value=px-ax*(Load+129)-200*cosTheta3-381*cosTheta3*cosTheta4-90;
%     fprintf('d1: %.2f\n',d1_value);
end

function [d2_value] = d2_solver(theta4, pz, az,Load)
    sinTheta4 = boundedTrig(@sin, theta4);
    d2_value=-591/2-381*sinTheta4-pz+az*(Load+129);
%     fprintf('d2: %.2f\n',d2_value);
end

function [theta6_1, theta6_2] = theta6_solver(theta4, theta5, az)
    % 使用数�?�替换后的变量进行计�??
    c4 = cos(theta4);
    s4 = sin(theta4);
    c5 = cos(theta5);
    term = (c4*c5)^2 + s4^2 - az^2;
    if term < 0
        error('[theta6错误] 无实数解�??(c4*c5)^2 + s4^2 - az^2 = %.2f < 0', term);
        
    end
    % 符号选择传�?��?�计�??
    delta_plus = sqrt(term);
    delta_minus = -delta_plus;

    % theta6解的形式
    theta6_1  = atan2(-az, delta_plus) - atan2(s4, c4*c5);
    theta6_2 = atan2(-az, delta_minus) - atan2(s4, c4*c5);
    theta6_1=wrapToPi(theta6_1);
    theta6_2=wrapToPi(theta6_2);

    % �??查奇异�?�条件（接近0时视为奇异）
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
    
%     fprintf('theta6�?? (�??): �??1: %.2f°  |  �??2: %.2f°\n', ...
%     wrapTo180(rad2deg(theta6_solutions(1))), wrapTo180(rad2deg(theta6_solutions(end))));
end

function [theta7_1, theta7_2] = theta7_solver(theta4, theta5, oz, nz)
    % 参数安全处理
    c4 = cos(theta4);
    s5 = sin(theta5); 
    oz = real(oz); nz = real(nz);
    
    R = sqrt(oz^2 + nz^2);
    threshold = (c4 * s5) / R;
    
    phi = (atan2(-oz, -nz)); % 使用0~2pi范围
    
    base_angle = asin(threshold);
    
    theta7_1 = (-phi + base_angle);
    theta7_2 = (-phi + pi - base_angle);
    theta7_1=wrapToPi(theta7_1);
    theta7_2=wrapToPi(theta7_2);    

end


function [theta3_1, theta3_2] = theta3_solver2(theta4, theta5,theta7, oy, ny)
    % 参数安全处理
    s4 = sin(theta4);
    s5 = sin(theta5); 
    c5 = cos(theta5); 
    s7 = sin(theta7); 
    c7 = cos(theta7); 
    oy = real(oy); ny = real(ny);
    
    % 计算参�?�量
    R = sqrt(c5^2 + s4^2*s5^2);
    threshold = (-oy*c7-ny*s7) / R;
    
    % 奇异性管�??
%     if abs(threshold) > 1
%         fprintf('当前阈�??: %.4f，将进行剪切处理\n', threshold);
%         threshold = sign(threshold) * 1;
%     end
    
    % 相位计算
    phi = (atan2(c5, -s4*s5)); % 使用0~2pi范围
    
    % 基础角度计算
    base_angle = asin(threshold);
    
    % 多个解的生成
    theta3_1 = (-phi + base_angle);
    theta3_2 = (-phi + pi - base_angle);
    theta3_1=wrapToPi(theta3_1);
    theta3_2=wrapToPi(theta3_2);    
%     theta3_solutions = struct(...
%         'Solution1', wrapTo180(rad2deg(theta3_1)),...
%         'Solution2', wrapTo180(rad2deg(theta3_2)));
%     fprintf('theta3�?? (�??): �??1: %.2f°  |  �??2: %.2f°\n', ...
%     theta3_solutions.Solution1, theta3_solutions.Solution2);
end
function [theta3_3, theta3_4] = theta3_solver1(theta4, theta5,theta6, ay)
    % 参数安全处理
    c4 = cos(theta4); 
    s4 = sin(theta4);
    s5 = sin(theta5); 
    c5 = cos(theta5); 
    s6 = sin(theta6); 
    c6 = cos(theta6); 
    ay = real(ay); 
    A=-s5*s6;
    B=c4*c6-s6*c5*s4;
        % 计算参�?�量
    R = sqrt(B^2 + A^2);
    threshold = (-ay) / R;



    % 奇异性管�??
%     if abs(threshold) > 1
%         fprintf('当前阈�??: %.4f，将进行剪切处理\n', threshold);
%         threshold = sign(threshold) * 1;
%     end
    
    % 相位计算
    phi = (atan2(A,B)); % 使用0~2pi范围
    
    % 基础角度计算
    base_angle = asin(threshold);
    
    % 多个解的生成
    theta3_3 = (-phi + base_angle);
    theta3_4 = (-phi + pi - base_angle);
    theta3_3=wrapToPi(theta3_3);
    theta3_4=wrapToPi(theta3_4);    

end

function [theta5_1, theta5_2] = theta5_solver(theta4, theta7, oz,nz)
    % 输入参数
    % theta4, theta6, az: 必�?�参�?
    % varargin: 当分母为零时，需补充theta3, ax, ay
    c4 = cos(theta4);
    s4 = sin(theta4);
    c7 = cos(theta7);
    s7 = sin(theta7);
    oz = real(oz); nz = real(nz);

    denom = c4;
    numerator = -oz*c7 - nz*s7;
    term = numerator / denom;

    theta5_1 = wrapToPi(asin(term));
    theta5_2 = wrapToPi(theta5_1+ pi);
    

end

function [theta6_3, theta6_4] = theta6_solver2(theta4, theta5,theta7, oz,nz)
    % 参数安全处理
    c4 = cos(theta4); 
    s4 = sin(theta4);
    s5 = sin(theta5); 
    c5 = cos(theta5); 
    s7 = sin(theta7); 
    c7 = cos(theta7); 
    oz = real(oz); nz = real(nz);
    A=-c5*c4;
    B=s4;
    
    R = sqrt(B^2 + A^2);
    threshold = (oz*s7-nz*c7) / R;
    
    % 相位计算
    phi = (atan2(A,B)); % 使用0~2pi范围
    
    % 基础角度计算
    base_angle = asin(threshold);
    
    % 多个解的生成
    theta6_3 = (-phi + base_angle);
    theta6_4 = (-phi + pi - base_angle);
    theta6_3=wrapToPi(theta6_3);
    theta6_4=wrapToPi(theta6_4);    

end
%% ============== 辅助工具函数 ================
% 安全三角函数计算
function val = boundedTrig(func, angle)
    raw_val = func(angle);
    if abs(raw_val) > 1
        if abs(raw_val) - 1 > 1e-6
            warning('调整三角函数�?? %.4f �?? ±1', raw_val);
        end
        val = sign(raw_val) * 1;
    else
        val = real(raw_val);
    end
end

% 角度范围限制函数
function angle = wrapToPi(angle)

    try
        if isempty(angle)
            angle = [];
            return;
        end
    angle = mod(angle + pi, 2*pi) - pi;
    
    catch ME
%         warning('%s', ME.message);
        angle = NaN; 
    end

end

function angle = wrapTo2Pi(angle)
    try
        if isempty(angle)
            angle = [];
            return;
        end
    angle = mod(angle, 2*pi);
    catch ME
%         warning('%s', ME.message);
        angle = NaN; 
    end
end

function deg = wrapTo180(deg)
    try
        if isempty(deg)
            deg = [];
            return;
        end
    deg = mod(deg + 180, 360) - 180;
    catch ME
%         warning('%s', ME.message);
        deg = NaN;
    end
end