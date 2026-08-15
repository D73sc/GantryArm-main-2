clc;
clear;

%% ================== 绗﹀彿瀹氫箟閮ㄥ垎 (DH鍙傛暟寤烘ā) ==================
syms nx ny nz ox oy oz ax ay az px py pz real   % 鐩爣浣嶅Э鐭╅樀鍙傛暟
syms d0 d1 d2 theta3 theta4 theta5 theta6 theta7 real
syms Load real
global Load;
Load=100;
radian1 = pi/180;
% 瀹氫箟MDH鍙傛暟
a = sym([0,0,0,90, 200, 0,0, 0]);        % 杩炴潌闀垮害
alpha = sym([pi/2,pi/2,pi/2,0, pi/2, pi/2, pi/2, pi/2]); % 鎵
d = sym([d0,d1,d2,160+135.5, 0, 152.5+228.5,0, 129+Load]); % 鍋忕Щ
theta = [pi/2,-pi/2,pi/2,theta3, theta4 + pi/2, theta5+ pi, theta6+ pi, theta7+ pi]; % 鍏宠妭瑙?
%瀹氫箟鍏宠妭瑙掑害闄愬埗
lim0_min = 0 ; lim0_max = 2500 ; %鍏宠妭1(-170锛?170)
lim1_min = 0 ; lim1_max = 3000 ; %鍏宠妭1(-170锛?170)
lim2_min = 0 ; lim2_max = 2000 ; %鍏宠妭1(-170锛?170)
lim3_min = -180 * radian1; lim3_max = 180 * radian1; %鍏宠妭1(-170锛?170)
lim4_min = -45 * radian1; lim4_max =  100 * radian1; %鍏宠妭2(-132锛?0)
lim5_min = -180 * radian1; lim5_max = 180 * radian1; %鍏宠妭3(1锛?141)
lim6_min = -120 * radian1; lim6_max = 120 * radian1; %鍏宠妭5(-105锛?105)
lim7_min = -180 * radian1; lim7_max = 180 * radian1; %鍏宠妭5(-105锛?105)

% 鏋勯?犵洰鏍囦綅濮跨煩闃?
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
    % 鐢熸垚鏁板?艰绠楀嚱鏁?
    T08_numeric = matlabFunction(T08,...
       'Vars', {'d0','d1','d2','theta3','theta4','theta5','theta6','theta7','Load'},...
       'Outputs', {'T08'});
   %% ================ 鐢熸垚鎵归噺娴嬭瘯鏍锋湰 ================
    N = 1;  % 鏍锋湰鏁伴噺
    tolerance = 1e-6;  % 鐭╅樀姣旇緝瀹瑰樊
    % 鍒涘缓鍚勫弬鏁扮殑杈圭晫鍊兼暟缁勶紙鍒楀悜閲忥級
    limit_min = [lim0_min; lim1_min; lim2_min; 
                lim3_min; lim4_min; lim5_min; lim6_min; lim7_min];
    limit_max = [lim0_max; lim1_max; lim2_max; 
                lim3_max; lim4_max; lim5_max; lim6_max; lim7_max];
    range = limit_max - limit_min;
    
    % 鐢熸垚闅忔満鍙傛暟鐭╅樀
    rng('default'); % 淇濊瘉缁撴灉鍙噸澶?
    param_matrix = limit_min' + (limit_max - limit_min)' .* rand(N, 8);
    % 棰勫垎閰嶇粨鏋滃瓨鍌?
    result_cell = cell(N,24);  % 鏈?缁堝瓨鍌‥xcel鏁版嵁
    success_count = 0;         % 鎴愬姛璁℃暟鍣?
    valid_positions = [];      % 瀛樺偍鏈夋晥鐐瑰潗鏍?
    % param_matrix姣忓垪瀵瑰簲鍙傛暟椤哄簭锛?
    % [d0, d1, d2, theta3, theta4, theta5, theta6, theta7]
%% ================ 鎵归噺娴嬭瘯寰幆 ================
% 鍦ㄥ惊鐜墠娣诲姞浠ヤ笅浠ｇ爜鍚敤澶氱嚎绋?
% if maxNumCompThreads > 1
%     parpool('local'); % 闇?瑕丳arallel Computing Toolbox
% end
% 初始化解矩阵结构
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
for n = 1:N
    current_params = num2cell(param_matrix(n, :));
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
    step = 100; % 璋冭妭姝ヨ繘浠ユ彁楂樻晥鐜?
    offset=0.01;
    theta4_values = linspace(lim4_min+offset, lim4_max-offset,step);
    theta5_values = linspace(lim5_min+offset, lim5_max-offset,step);
    [grid_theta4, grid_theta5] = ndgrid(theta4_values, theta5_values);
        param_matrix = [zeros(size(grid_theta4(:))) ... % d0鏆傝涓?0
                    zeros(size(grid_theta4(:))) ... % d1
                    zeros(size(grid_theta4(:))) ... % d2
                    zeros(size(grid_theta4(:))) ... % theta3
                    grid_theta4(:) grid_theta5(:) ...
                    zeros(numel(grid_theta4(:)),2)]; % theta6/7鏆傝涓?0
    N_ds = size(param_matrix, 1);
    theta3_ds=[];
    theta4_ds=[];
    theta5_ds=[];

    for n = 1:N_ds
        theta4_val=grid_theta4(n);
        theta5_val=grid_theta5(n);
        
    %% ================ calculate ================

        [theta6_sols, theta7_sols] = deal(zeros(2,1));
        try
            [theta7_sols(1), theta7_sols(2)] = theta7_solver(theta4_val, theta5_val, oz, nz);
            [theta6_sols(1), theta6_sols(2)] = theta6_solver(theta4_val, theta5_val, az);
            t6_num=2;
            t7_num=2;
        catch
            try
                [theta7_sols(1), theta7_sols(2)] = theta7_solver(theta4_val, theta5_val, oz, nz);
                [theta6_sols(1), theta6_sols(2)] = theta6_solver2(theta4_val, theta5_val,theta7_sols(1), oz,nz);
                [theta6_sols(3), theta6_sols(4)] = theta6_solver2(theta4_val, theta5_val,theta7_sols(2), oz,nz);
                t6_num=4;
                t7_num=2;
            catch
                continue;
            end
        end
        
        total_solutions = 0;

        for t6_idx = 1:t6_num
            for t7_idx = 1:t7_num
                theta6_ik = theta6_sols(t6_idx);
                theta7_ik = theta7_sols(t7_idx);

                % 计算 theta3 的两个解
                try
                    [theta3_1, theta3_2] = theta3_solver2(theta4_val, theta5_val, theta7_ik, oy, ny);
                catch
                    try
                        [theta3_1, theta3_2] = theta3_solver1(theta4_val, theta5_val,theta6_ik, ay);
                    catch
                        continue;
                    end
                end
                theta3_candidates = [theta3_1, theta3_2]; % 明确赋值给变量
                for t3_case = 1:2
                     theta3_ik = theta3_candidates(t3_case); % 合法语法：用圆括号索引
                     % 存储每组解并递增计数器
                     total_solutions = total_solutions + 1;
                     joint_solutions(total_solutions).theta3 = theta3_ik;
                     joint_solutions(total_solutions).theta6  = theta6_ik;
                     joint_solutions(total_solutions).theta7  = theta7_ik;
                 end
            end
        end    

        for sol_idx = 1:total_solutions
            sol = joint_solutions(sol_idx);
            joint_solutions(sol_idx).theta4=theta4_val;
            joint_solutions(sol_idx).theta5=theta5_val;
            % 跳过未填充的解
            if isnan(joint_solutions(sol_idx).theta3)
                continue;
            end

            % 提取当前解的 theta 值和 d 值
            theta3_ik = joint_solutions(sol_idx).theta3;
            theta6_ik = joint_solutions(sol_idx).theta6;
            theta7_ik = joint_solutions(sol_idx).theta7;
                % 计算 d0/d1/d2
            try


                d0_ik = d0_solver(sol.theta3, theta4_val, py, ay, Load);
                d1_ik = d1_solver(sol.theta3, theta4_val, px, ax, Load);
                d2_ik = d2_solver(theta4_val, pz, az, Load);
                sol.d0 = d0_ik; sol.d1 = d1_ik; sol.d2 = d2_ik;
            catch
                continue; % 该分支传递参数不合法
            end
                % 填充 d0/d1/d2 到结构体中
            joint_solutions(sol_idx).d0 = d0_ik;
            joint_solutions(sol_idx).d1 = d1_ik;
            joint_solutions(sol_idx).d2 = d2_ik;
            % 关节限位校验
            joint_values = [d0_ik, d1_ik, d2_ik, sol.theta3, theta4_val, theta5_val, sol.theta6, sol.theta7];
            joint_solutions(sol_idx).limits_ok = check_joint_limits(joint_values, limit_min, limit_max);        



            % 正运动学校验
            try
                T_ik = T08_numeric(d0_ik, d1_ik, d2_ik, theta3_ik, theta4_val, theta5_val, theta6_ik, theta7_ik, Load);
                pos_error = norm(T_goal(1:3,4) - T_ik(1:3,4));
                rot_error = norm(T_goal(1:3,1:3) - T_ik(1:3,1:3), 'fro');
                joint_solutions(sol_idx).errors = [pos_error, rot_error];
                joint_solutions(sol_idx).is_valid = all([pos_error < 1e-3, rot_error < 1e-3]) && all(joint_solutions(sol_idx).limits_ok);
            catch
                joint_solutions(sol_idx).is_valid = false;
            end

        end

        % 统计成功解数目
        valid_solutions = joint_solutions([joint_solutions.is_valid]);
        unique_valid_solutions = remove_duplicate_solutions(valid_solutions);
        success_count=0;
        for sol = unique_valid_solutions
            success_count=success_count+1;
            fprintf('解: θ3=%.2f, θ6=%.2f, θ7=%.2f, d0=%.2f, d1=%.2f, d2=%.2f\n',...
                sol.theta3, sol.theta6, sol.theta7, sol.d0, sol.d1, sol.d2);
            theta3_ds=[theta3_ds,sol.theta3/radian1];
            theta4_ds=[theta4_ds,sol.theta4/radian1];
            theta5_ds=[theta5_ds,sol.theta5/radian1];
            
        end
        fprintf('共找到 %d 组有效解：\n', success_count);

    %% ================ decision space ================
    end
    
    % 缁樺埗涓夌淮鏁ｇ偣鍥?
    figure('Position', [100, 100, 800, 600]);
    scatter3( theta4_ds, theta5_ds, theta3_ds,40, 'filled', 'MarkerEdgeColor', 'k');
    xlabel('Theta4 (deg)');
    ylabel('Theta5 (deg)');
    zlabel('Theta3 (deg)');
    title('Theta4-5-3');
    grid on;
    view(45, 30); % 璋冩暣瑙嗚
    
end

%% ================ 缁熻涓庤緭鍑? ================

% 杞崲涓鸿〃鏍煎苟淇濆瓨Excel
% result_table = cell2table(result_cell, 'VariableNames', column_names);
% writetable(result_table, 'F:\5DOFGantryArm (3)\5DOFGantryArmMatlab\InverseKinematics_Validation_345.xlsx');

%% ================ 鍙鍖? ================
% figure('Name', '宸ヤ綔绌洪棿鍒嗘瀽', 'Position', [100 100 1200 500])
% subplot(1,2,1)
% scatter3(valid_positions(:,1), valid_positions(:,2), valid_positions(:,3),...
%     15, 'filled', 'MarkerFaceAlpha',0.6)
% xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
% title('鏈夋晥閫嗚В鏈浣嶇疆鍒嗗竷')
% grid on; axis equal;
% 
% subplot(1,2,2)
% success_rates = [success_rate, 100-success_rate];
% pie(success_rates, {'鎴愬姛', '澶辫触'})
% title(sprintf('鎬绘垚鍔熺巼 %.1f%%', success_rate))

% ==== 鍏宠妭闄愪綅妫?鏌ュ嚱鏁? ====
function is_valid = check_joint_limits(params, min_limits, max_limits)
    is_valid = (params >= min_limits') & (params <= max_limits');
end
function unique_solutions = remove_duplicate_solutions(solutions)
    % 输入: solutions - 逆运动学解的结构体数组（包含d0,d1,d2,θ3-θ7等字段）
    % 输出: unique_solutions - 去重后的唯一结构体数组
    
    % --- 参数定义 ---
    tol_position = 1e-4;  % 平移关节公差 (单位：米)
    tol_angle    = 1e-4;  % 旋转关节公差 (单位：弧度)
    
    % --- 去重逻辑 ---
    if isempty(solutions)
        unique_solutions = [];
        return;
    end
    
    % 构造参数矩阵
    param_matrix = [...
        [solutions.d0]', [solutions.d1]', [solutions.d2]', ...
        [solutions.theta3]', [solutions.theta4]', ...
        [solutions.theta5]', [solutions.theta6]', [solutions.theta7]' ...
    ];
    
    % 归一化旋转关节角度到 [-π, π]
    normalize = @(x) mod(x + pi, 2*pi) - pi;
    param_matrix(:,4:8) = normalize(param_matrix(:,4:8));
    
    % 按精度离散化参数
    scaled_pos = round(param_matrix(:,1:3) / tol_position); 
    scaled_ang = round(param_matrix(:,4:8) / tol_angle);
    scaled_matrix = [scaled_pos, scaled_ang];
    
    % 提取唯一解索引
    [~, unique_idx] = unique(scaled_matrix, 'rows', 'stable');
    unique_solutions = solutions(unique_idx);
end
    

%% ================ 涓昏璁＄畻鍑芥暟 ================
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
    % 浣跨敤鏁板?兼浛鎹㈠悗鐨勫彉閲忚繘琛岃绠?
    c4 = cos(theta4);
    s4 = sin(theta4);
    c5 = cos(theta5);
    term = (c4*c5)^2 + s4^2 - az^2;
    if term < 0
        error('[theta6閿欒] 鏃犲疄鏁拌В锛?(c4*c5)^2 + s4^2 - az^2 = %.2f < 0', term);
        
    end
    % 绗﹀彿閫夋嫨浼犻?掓?ц绠?
    delta_plus = sqrt(term);
    delta_minus = -delta_plus;

    % theta6瑙ｇ殑褰㈠紡
    theta6_1  = atan2(-az, delta_plus) - atan2(s4, c4*c5);
    theta6_2 = atan2(-az, delta_minus) - atan2(s4, c4*c5);
    theta6_1=wrapToPi(theta6_1);
    theta6_2=wrapToPi(theta6_2);

    % 妫?鏌ュ寮傛?ф潯浠讹紙鎺ヨ繎0鏃惰涓哄寮傦級
    tolerance_singular = 1e-6;
    if abs(s4) < tolerance_singular && (abs(c4*c5) < tolerance_singular)
        warning('[theta6璀﹀憡] 濂囧紓鎬ф潯浠舵垚绔嬶細s4=%.2e, c4*c5=%.2e', s4, c4*c5);
    end
    
    % 鏁寸悊胃6瑙ｉ泦
    if abs(theta6_1 - theta6_2) < 1e-6
        theta6_solutions = (theta6_1);
    else
        theta6_solutions = ([(theta6_1), (theta6_2)]);
    end
    
%     fprintf('theta6瑙? (搴?): 瑙?1: %.2f掳  |  瑙?2: %.2f掳\n', ...
%     wrapTo180(rad2deg(theta6_solutions(1))), wrapTo180(rad2deg(theta6_solutions(end))));
end

function [theta7_1, theta7_2] = theta7_solver(theta4, theta5, oz, nz)
    % 鍙傛暟瀹夊叏澶勭悊
    c4 = cos(theta4);
    s5 = sin(theta5); 
    oz = real(oz); nz = real(nz);
    
    % 璁＄畻鍙傝?冮噺
    R = sqrt(oz^2 + nz^2);
    threshold = (c4 * s5) / R;
    
%     % 濂囧紓鎬х鐞?
%     if abs(threshold) > 1
%         fprintf('褰撳墠闃堝??: %.4f锛屽皢杩涜鍓垏澶勭悊\n', threshold);
%         threshold = sign(threshold) * 1;
%     end
    
    % 鐩镐綅璁＄畻
    phi = (atan2(-oz, -nz)); % 浣跨敤0~2pi鑼冨洿
    
    % 鍩虹瑙掑害璁＄畻
    base_angle = asin(threshold);
    
    % 澶氫釜瑙ｇ殑鐢熸垚
    theta7_1 = (-phi + base_angle);
    theta7_2 = (-phi + pi - base_angle);
    theta7_1=wrapToPi(theta7_1);
    theta7_2=wrapToPi(theta7_2);    

end


function [theta3_1, theta3_2] = theta3_solver2(theta4, theta5,theta7, oy, ny)
    % 鍙傛暟瀹夊叏澶勭悊
    s4 = sin(theta4);
    s5 = sin(theta5); 
    c5 = cos(theta5); 
    s7 = sin(theta7); 
    c7 = cos(theta7); 
    oy = real(oy); ny = real(ny);
    
    % 璁＄畻鍙傝?冮噺
    R = sqrt(c5^2 + s4^2*s5^2);
    threshold = (-oy*c7-ny*s7) / R;
    
    % 濂囧紓鎬х鐞?
%     if abs(threshold) > 1
%         fprintf('褰撳墠闃堝??: %.4f锛屽皢杩涜鍓垏澶勭悊\n', threshold);
%         threshold = sign(threshold) * 1;
%     end
    
    % 鐩镐綅璁＄畻
    phi = (atan2(c5, -s4*s5)); % 浣跨敤0~2pi鑼冨洿
    
    % 鍩虹瑙掑害璁＄畻
    base_angle = asin(threshold);
    
    % 澶氫釜瑙ｇ殑鐢熸垚
    theta3_1 = (-phi + base_angle);
    theta3_2 = (-phi + pi - base_angle);
    theta3_1=wrapToPi(theta3_1);
    theta3_2=wrapToPi(theta3_2);    
%     theta3_solutions = struct(...
%         'Solution1', wrapTo180(rad2deg(theta3_1)),...
%         'Solution2', wrapTo180(rad2deg(theta3_2)));
%     fprintf('theta3瑙? (搴?): 瑙?1: %.2f掳  |  瑙?2: %.2f掳\n', ...
%     theta3_solutions.Solution1, theta3_solutions.Solution2);
end
function [theta3_3, theta_4] = theta3_solver1(theta4, theta5,theta6, ay)
    % 鍙傛暟瀹夊叏澶勭悊
    c4 = cos(theta4); 
    s4 = sin(theta4);
    s5 = sin(theta5); 
    c5 = cos(theta5); 
    s6 = sin(theta6); 
    c6 = cos(theta6); 
    ay = real(ay); 
    A=-s5*s6;
    B=c4*c6-s6*c5*s4;
        % 璁＄畻鍙傝?冮噺
    R = sqrt(B^2 + A^2);
    threshold = (-ay) / R;



%     % 濂囧紓鎬х鐞?
%     if abs(threshold) > 1
%         fprintf('褰撳墠闃堝??: %.4f锛屽皢杩涜鍓垏澶勭悊\n', threshold);
% %         threshold = sign(threshold) * 1;
%     end
    
    % 鐩镐綅璁＄畻
    phi = (atan2(A,B)); % 浣跨敤0~2pi鑼冨洿
    
    % 鍩虹瑙掑害璁＄畻
    base_angle = asin(threshold);
    
    % 澶氫釜瑙ｇ殑鐢熸垚
    theta3_3 = (-phi + base_angle);
    theta_4 = (-phi + pi - base_angle);
    theta3_3=wrapToPi(theta3_3);
    theta_4=wrapToPi(theta_4);    

end
function [theta6_3, theta6_4] = theta6_solver2(theta4, theta5,theta7, oz,nz)
    % 鍙傛暟瀹夊叏澶勭悊
    c4 = cos(theta4); 
    s4 = sin(theta4);
    s5 = sin(theta5); 
    c5 = cos(theta5); 
    s7 = sin(theta7); 
    c7 = cos(theta7); 
    oz = real(oz); nz = real(nz);
    A=-c5*c4;
    B=s4;
        % 璁＄畻鍙傝?冮噺
    R = sqrt(B^2 + A^2);
    threshold = (oz*s7-nz*c7) / R;



    % 濂囧紓鎬х鐞?
%     if abs(threshold) > 1
%         fprintf('褰撳墠闃堝??: %.4f锛屽皢杩涜鍓垏澶勭悊\n', threshold);
%         threshold = sign(threshold) * 1;
%     end
    
    % 鐩镐綅璁＄畻
    phi = (atan2(A,B)); % 浣跨敤0~2pi鑼冨洿
    
    % 鍩虹瑙掑害璁＄畻
    base_angle = asin(threshold);
    
    % 澶氫釜瑙ｇ殑鐢熸垚
    theta6_3 = (-phi + base_angle);
    theta6_4 = (-phi + pi - base_angle);
    theta6_3=wrapToPi(theta6_3);
    theta6_4=wrapToPi(theta6_4);    

end
%% ============== 杈呭姪宸ュ叿鍑芥暟 ================
% 瀹夊叏涓夎鍑芥暟璁＄畻
function val = boundedTrig(func, angle)
    raw_val = func(angle);
    if abs(raw_val) > 1
        if abs(raw_val) - 1 > 1e-6
            warning('璋冩暣涓夎鍑芥暟鍊? %.4f 鍒? 卤1', raw_val);
        end
        val = sign(raw_val) * 1;
    else
        val = real(raw_val);
    end
end

% 瑙掑害鑼冨洿闄愬埗鍑芥暟
function angle = wrapToPi(angle)

    try
        % ----- 输入验证 -----
        % 空值处理
        if isempty(angle)
            angle = [];
            return;
        end
    angle = mod(angle + pi, 2*pi) - pi;
    
    catch ME
        % 捕获异常并给出友好提示
        warning('角度归约失败: %s', ME.message);
        angle = NaN; % 返回NaN标记无效值
    end

end

function angle = wrapTo2Pi(angle)
    try
        % ----- 输入验证 -----
        % 空值处理
        if isempty(angle)
            angle = [];
            return;
        end
    angle = mod(angle, 2*pi);
    catch ME
        % 捕获异常并给出友好提示
        warning('角度归约失败: %s', ME.message);
        angle = NaN; % 返回NaN标记无效值
    end
end

function deg = wrapTo180(deg)
    try
        % ----- 输入验证 -----
        % 空值处理
        if isempty(deg)
            deg = [];
            return;
        end
    deg = mod(deg + 180, 360) - 180;
    catch ME
        % 捕获异常并给出友好提示
        warning('角度归约失败: %s', ME.message);
        deg = NaN; % 返回NaN标记无效值
    end
end