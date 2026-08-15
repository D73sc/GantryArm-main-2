% 使用方法一
ieeeStyleVisualization('2025-09-09-15-02-upperpoint.csv');
%ieeeStyleVisualization('line.csv');

function ieeeStyleVisualization(csvFile)
    % IEEE双栏论文风格
    
    data = readmatrix(csvFile);
    x = data(:,1); y = data(:,2); z = data(:,3);
    qx = data(:,4); qy = data(:,5); qz = data(:,6); qw = data(:,7);
    
    % IEEE双栏图尺寸: 3.5英寸宽
    fig = figure('Units', 'inches', 'Position', [1, 1, 7, 3], 'Color', 'w');
    
    % 子图1: 3D轨迹
    subplot(1, 2, 1);
    hold on; box on; grid on;
    
    plot3(x, y, z, 'k-', 'LineWidth', 1.5);
    plot3(x(1), y(1), z(1), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    plot3(x(end), y(end), z(end), 'ks', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    
     angleThreshold = 30;      % 转折角度阈值（度）
    minPointSpacing = 40;      % 相邻箭头最小间隔
    offsetBefore = 2;         % 在转折点前多少个点放置箭头（★关键参数★）
    
    % 计算所有点的转折角
    turningAngles = calculateTurningAngles(x, y, z);
    
    % 找出大转折点
    turningIndices = [];
    lastIndex = 0;
    
    for i = 2:length(x)-1
        angle_deg = turningAngles(i);
        
        if angle_deg > angleThreshold && (i - lastIndex) >= minPointSpacing
            turningIndices = [turningIndices, i];
            lastIndex = i;
        end
    end
    
    % 在转折前放置箭头
    arrowLength = max([range(x), range(y), range(z)]) * 0.10;
    
    for idx = turningIndices
        % 关键改动：箭头位置在转折前offsetBefore个点
        arrowPosIdx = max(2, idx - offsetBefore);
        
        % 计算该点的运动方向
        if arrowPosIdx < length(x)
            direction = [x(arrowPosIdx+1) - x(arrowPosIdx), ...
                        y(arrowPosIdx+1) - y(arrowPosIdx), ...
                        z(arrowPosIdx+1) - z(arrowPosIdx)];
        else
            direction = [x(idx) - x(idx-1), ...
                        y(idx) - y(idx-1), ...
                        z(idx) - z(idx-1)];
        end
        direction = direction / norm(direction);
        
        % 箭头颜色随转折角大小变化（红色=大转折）
        angle_normalized = turningAngles(idx) / 180;
        color = [angle_normalized, 0, 1-angle_normalized];
        
        % 绘制箭头
        quiver3(x(arrowPosIdx), y(arrowPosIdx), z(arrowPosIdx), ...
                direction(1)*arrowLength, ...
                direction(2)*arrowLength, ...
                direction(3)*arrowLength, ...
                0, 'Color', color, 'LineWidth', 2, ...
                'MaxHeadSize', 0.8, 'AutoScale', 'off');
    end
    
    % 姿态箭头
    indices = round(linspace(1, length(x), 10));
    arrowLength = max([range(x), range(y), range(z)]) * 0.12;
    
    for i = indices
        R = quat2rotm([qw(i), qx(i), qy(i), qz(i)]);
        quiver3(x(i), y(i), z(i), ...
                R(1,3)*arrowLength, R(2,3)*arrowLength, R(3,3)*arrowLength, ...
                0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    end
    
%         % 计算方向向量
%     direction = [x(end) - x(1), y(end) - y(1), z(end) - z(1)];
%     direction_norm = norm(direction);
%     
%     % 绘制起点到终点的箭头（使用80%的实际距离，方便显示）
%     quiver3(x(1), y(1), z(1), ...
%             direction(1)*0.5, direction(2)*0.5, direction(3)*0.5, ...
%             0, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 1.0, 'AutoScale', 'off');
    
    xlabel('$x$ (m)', 'Interpreter', 'latex');
    ylabel('$y$ (m)', 'Interpreter', 'latex');
    zlabel('$z$ (m)', 'Interpreter', 'latex');
    axis equal; view(45, 30);
    set(gca, 'FontSize', 9, 'FontName', 'Times');
    title('(a) Trajectory and orientation', 'FontSize', 10);
    
    % 子图2: 姿态变化
    subplot(1, 2, 2);
    hold on; box on; grid on;
    
    % 计算转角
    angles = zeros(length(x)-1, 1);
    for i = 1:length(x)-1
        q1 = [qw(i), qx(i), qy(i), qz(i)];
        q2 = [qw(i+1), qx(i+1), qy(i+1), qz(i+1)];
        q_rel = quatmultiply(quatconj(q1), q2);
        angles(i) = 2 * acos(min(1, abs(q_rel(1)))) * 180/pi;
    end
    
    plot(2:length(x), angles, 'k-', 'LineWidth', 1.5);
    xlabel('Waypoint index', 'Interpreter', 'latex');
    ylabel('Angular change (deg)', 'Interpreter', 'latex');
    set(gca, 'FontSize', 9, 'FontName', 'Times');
    title('(b) Orientation change rate', 'FontSize', 10);
    
    % 保存
    % print(fig, 'trajectory_ieee', '-depsc', '-tiff');
end


function angles = calculateTurningAngles(x, y, z)
    % 计算轨迹上每一点的转折角度（单位：度）
    % 转折角 = 前后两个方向向量的夹角
    
    n = length(x);
    angles = zeros(n, 1);
    
    for i = 2:n-1
        % 前向量：从i-1指向i
        v1 = [x(i) - x(i-1), y(i) - y(i-1), z(i) - z(i-1)];
        % 后向量：从i指向i+1
        v2 = [x(i+1) - x(i), y(i+1) - y(i), z(i+1) - z(i)];
        
        % 计算向量夹角
        norm_v1 = norm(v1);
        norm_v2 = norm(v2);
        
        if norm_v1 > 1e-9 && norm_v2 > 1e-9
            % 点积公式：cos(θ) = (v1·v2)/(|v1||v2|)
            cos_angle = dot(v1, v2) / (norm_v1 * norm_v2);
            cos_angle = max(-1, min(1, cos_angle));  % 防止数值误差
            
            % 夹角 = 180° - arccos(cos_angle)
            % 因为我们要的是转向角，而不是向量夹角
            angle_rad = acos(cos_angle);
            angles(i) = (pi - angle_rad) * 180 / pi;
        end
    end
end