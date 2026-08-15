% AlgorithmVisualization.m
classdef AlgorithmVisualization
    methods (Static)
        
    % 图1：主性能对比图（精简英文版）
function plotMainPerformance(data)
    figure('Position', [100, 100, 800, 600], 'Name', 'Figure 1: Algorithm Performance Comparison');
    ms=5;
    algorithms = unique(data.Algorithm, 'stable');
    colors = lines(length(algorithms));
    
    %% 主图：计算时间序列
    axes('Position', [0.08, 0.12, 0.64, 0.78]);
    hold on;
    
    % 预先计算统计数据
    success10ms = zeros(length(algorithms), 1);
    avgTimes = zeros(length(algorithms), 1);
    maxTimes = zeros(length(algorithms), 1);
    
    for i = 1:length(algorithms)
        algoData = data(strcmp(data.Algorithm, algorithms{i}), :);
        totalPoints = height(algoData);
        
        % 计算10ms成功率和时间统计
        success10ms(i) = sum(algoData.ComputationTime_ms_ <= ms) / totalPoints * 100;
        avgTimes(i) = mean(algoData.ComputationTime_ms_);
        maxTimes(i) = max(algoData.ComputationTime_ms_);
        
        % 绘制时间序列曲线
        plot(algoData.PointIndex, algoData.ComputationTime_ms_, ...
            'Color', colors(i,:), 'LineWidth', 2.5, 'DisplayName', algorithms{i});
    end
    ylineText = sprintf('%dms Threshold',ms);

    % 添加10ms参考线
    yline(ms, 'r--', 'LineWidth', 2, 'DisplayName', ylineText);
    
    title('Computation Time Series', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Trajectory Point Index', 'FontSize', 12);
    ylabel('Computation Time (ms)', 'FontSize', 12);
    legend('Location', 'northwest', 'FontSize', 11);
    grid on;
    set(gca, 'FontSize', 11);
    hold off;
    
    %% 右侧统计信息
    statsText = 'Performance Statistics:';
        statsText = sprintf('%s\n', statsText);

    for i = 1:length(algorithms)
        statsText = sprintf('%s%s\n', statsText, algorithms{i});
        statsText = sprintf('%s  %dms Success: %.1f%%\n', statsText,ms, success10ms(i));
        statsText = sprintf('%s  Avg Time: %.2f ms\n', statsText, avgTimes(i));
        statsText = sprintf('%s  Max Time: %.2f ms\n\n', statsText, maxTimes(i));
    end
    
    % 创建右侧文字框
    annotation('textbox', [0.74, 0.20, 0.24, 0.65], ...
        'String', statsText, ...
        'FitBoxToText', 'off', ...
        'BackgroundColor', [0.96 0.96 0.98], ...
        'EdgeColor', [0.3 0.3 0.6], ...
        'LineWidth', 1.5, ...
        'FontSize', 10, ...
        'FontName', 'Arial', ...
        'VerticalAlignment', 'top', ...
        'Interpreter', 'none');
end
        
        %% 主函数（修复版）
        function plotJointMotionAnalysis(data)
            if ismember('Algorithm', data.Properties.VariableNames)
                algorithms = unique(data.Algorithm, 'stable');
                fprintf('\n=== 检测到 %d 种算法 ===\n', length(algorithms));
                for i = 1:length(algorithms)
                    fprintf('%d. %s\n', i, algorithms{i});
                end
            else
                algorithms = {'Unknown'};
            end
            
            % 按算法检测异常
            allAnomalies = struct();
            allData = struct();
            
            for algoIdx = 1:length(algorithms)
                if ismember('Algorithm', data.Properties.VariableNames)
                    algoData = data(strcmp(data.Algorithm, algorithms{algoIdx}), :);
                else
                    algoData = data;
                end
                
                fprintf('\n--- 分析算法: %s ---\n', algorithms{algoIdx});
                
                [angleAnomalies, velocityAnomalies, anomalousJoints] = ...
                    AlgorithmVisualization.detectAllAnomaliesForAlgorithm(algoData);
                
                algoFieldName = matlab.lang.makeValidName(algorithms{algoIdx});
                allAnomalies.(algoFieldName) = struct(...
                    'angleAnomalies', {angleAnomalies}, ...
                    'velocityAnomalies', {velocityAnomalies}, ...
                    'anomalousJoints', anomalousJoints);
                allData.(algoFieldName) = algoData;
            end
            
            % 生成图表（角度和速度分开）
            try
                AlgorithmVisualization.createMultiAlgorithmOverviewFigure(allData, allAnomalies, algorithms);
                
                % 分别生成角度和速度的详细图
                %AlgorithmVisualization.createComparativeDetailFigure(allData, allAnomalies, algorithms, 'angle');
                %AlgorithmVisualization.createComparativeDetailFigure(allData, allAnomalies, algorithms, 'velocity');
                
                AlgorithmVisualization.createAlgorithmComparisonFigure(allAnomalies, algorithms);
                
                    % 角度图（独立）
                AlgorithmVisualization.createComparativeDetailFigure(...
                    allData, allAnomalies, algorithms, 'angle');

                % 速度图（独立）
                AlgorithmVisualization.createComparativeDetailFigure(...
                    allData, allAnomalies, algorithms, 'velocity');
                fprintf('\n=== 所有图表生成完成 ===\n');
                AlgorithmVisualization.printAnomalyStatisticsTable(allAnomalies, algorithms);
            catch ME
                fprintf('绘图过程出错: %s\n', ME.message);
                fprintf('错误位置: %s (行%d)\n', ME.stack(1).name, ME.stack(1).line);
            end
        end
        
        %% 异常检测（修复数值溢出问题）
        function [angleAnomalies, velocityAnomalies, anomalousJoints] = detectAllAnomaliesForAlgorithm(data)
            angleAnomalies = cell(8, 1);
            velocityAnomalies = cell(8, 1);
            anomalousJoints = struct('angle', [], 'velocity', []);
            
            for joint = 1:8
                jointCol = sprintf('Joint%d', joint);
                if ismember(jointCol, data.Properties.VariableNames)
                    angles = data.(jointCol);
                    pointIndex = (1:length(angles))';
                    
                    [hasAnomaly, anomalyInfo] = AlgorithmVisualization.detectObviousAnomalies(...
                        angles, pointIndex, 'angle', joint);
                    
                    if hasAnomaly
                        angleAnomalies{joint} = anomalyInfo;
                        anomalousJoints.angle = [anomalousJoints.angle, joint];
                        fprintf('  关节%d角度: 检测到 %d 个异常点 (严重度: %.1f-%.1f)\n', ...
                            joint, length(anomalyInfo.points), ...
                            min(anomalyInfo.severity), max(anomalyInfo.severity));
                    end
                end
                
                velCol = sprintf('Vel%d', joint);
                if ismember(velCol, data.Properties.VariableNames)
                    velocities = data.(velCol);
                    pointIndex = (1:length(velocities))';
                    
                    [hasAnomaly, anomalyInfo] = AlgorithmVisualization.detectObviousAnomalies(...
                        velocities, pointIndex, 'velocity', joint);
                    
                    if hasAnomaly
                        velocityAnomalies{joint} = anomalyInfo;
                        anomalousJoints.velocity = [anomalousJoints.velocity, joint];
                        fprintf('  关节%d速度: 检测到 %d 个异常点 (严重度: %.1f-%.1f)\n', ...
                            joint, length(anomalyInfo.points), ...
                            min(anomalyInfo.severity), max(anomalyInfo.severity));
                    end
                end
            end
        end
        
%% 核心异常检测算法（最小修改版）
function [hasAnomaly, anomalyInfo] = detectObviousAnomalies(data, pointIndex, dataType, jointNum)
    hasAnomaly = false;
    anomalyInfo = struct();
    
    if length(data) < 20
        return;
    end
    
    if nargin < 4
        jointNum = 0;
    end
    
    slopes = diff(data);
    ignoreBoundary = 5;
    
    candidateIndices = [];
    localRatios = [];
    localSlopes = [];
    
    % ★★★ 根据数据类型设置阈值 ★★★
    if strcmp(dataType, 'velocity')
        % 速度数据的阈值（更宽松）
        if jointNum >= 1 && jointNum <= 3
            minSlopeThreshold = 5;    % 大关节速度
            ratioThreshold = 6;         % 比率阈值更低
        else
            minSlopeThreshold = 0.1;   % 小关节速度
            ratioThreshold = 8;         % 比率阈值更低
        end
    else
        % 角度数据的阈值（保持原样）
        if jointNum >= 1 && jointNum <= 3
            minSlopeThreshold = 5;
            ratioThreshold = 6;
        else
            minSlopeThreshold = 0.1;
            ratioThreshold = 8;
        end
    end
    
    for i = 1:length(slopes)
        if i <= ignoreBoundary || i >= length(slopes) - ignoreBoundary
            continue;
        end
        
        currentSlope = slopes(i);
        
        % 如果当前斜率本身就接近零，跳过
        if abs(currentSlope) < 1e-6
            continue;
        end
        
        windowSize = 5;
        startIdx = max(1, i - windowSize);
        endIdx = min(length(slopes), i + windowSize);
        
        windowData = slopes(startIdx:endIdx);
        windowData = windowData(abs(windowData - currentSlope) > 1e-6);
        
        if isempty(windowData)
            continue;
        end
        
        % 计算窗口平均
        windowAvg = mean(abs(windowData));
        
        % 设置合理的最小阈值
        minWindowAvg = 1e-3;
        if windowAvg < minWindowAvg
            windowAvg = minWindowAvg;
        end
        
        % 计算异常比率
        slopeRatio = abs(currentSlope) / windowAvg;
        
        % 添加严重度上限
        maxSeverity = 30;
        if slopeRatio > maxSeverity
            slopeRatio = maxSeverity;
        end
        
        % ★★★ 使用上面设置的阈值 ★★★
        if slopeRatio > ratioThreshold && abs(currentSlope) > minSlopeThreshold
            candidateIndices = [candidateIndices, i];
            localRatios = [localRatios, slopeRatio];
            localSlopes = [localSlopes, currentSlope];
        end
    end
    
    if ~isempty(candidateIndices)
        hasAnomaly = true;
        anomalyInfo.indices = candidateIndices + 1;
        anomalyInfo.points = pointIndex(candidateIndices + 1);
        anomalyInfo.values = data(candidateIndices + 1);
        anomalyInfo.slopes = localSlopes;
        anomalyInfo.localRatios = localRatios;
        anomalyInfo.severity = localRatios;
        
        % 最终检查：移除任何异常的严重度值
        validIdx = anomalyInfo.severity < maxSeverity;
% 最终检查：限制严重度值到上限，但不移除点
if any(anomalyInfo.severity > maxSeverity)
    exceedCount = sum(anomalyInfo.severity > maxSeverity);
    fprintf('警告：将 %d 个异常点的严重度限制到上限 %.1f×\n', exceedCount, maxSeverity);
    
    % 只修改严重度值，保留所有点
    anomalyInfo.severity(anomalyInfo.severity > maxSeverity) = maxSeverity;
    anomalyInfo.localRatios(anomalyInfo.localRatios > maxSeverity) = maxSeverity;
end
        
        if isempty(anomalyInfo.points)
            hasAnomaly = false;
        end
    end
end
        
       %% 修改 createMultiAlgorithmOverviewFigure 函数
function createMultiAlgorithmOverviewFigure(allData, allAnomalies, algorithms)
    fig = figure('Position', [100, 100, 1600, 900], 'Name', 'Fig.1 Multi-Algorithm Motion Overview');
    set(fig, 'Color', 'w');
    
    numAlgos = length(algorithms);
    algoColors = lines(numAlgos);
    
    % 线型样式（用于区分同一算法的不同关节）
    lineStyles = {'-', '--', ':', '-.', '-', '--', ':', '-.'};
    
    %% 子图1：平移轴角度（J1-J3）
    subplot(2, 2, 1);
    hold on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
    
    legendEntries = {};
    for algoIdx = 1:numAlgos
        algoName = matlab.lang.makeValidName(algorithms{algoIdx});
        algoData = allData.(algoName);
        
        for joint = 1:3
            col = sprintf('Joint%d', joint);
            if ismember(col, algoData.Properties.VariableNames)
                plot(1:length(algoData.(col)), algoData.(col), ...
                    'Color', algoColors(algoIdx,:), ...
                    'LineStyle', lineStyles{joint}, ...
                    'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s-J%d', algorithms{algoIdx}, joint));
                legendEntries{end+1} = sprintf('%s-J%d', algorithms{algoIdx}, joint);
            end
        end
    end
    
    title('(a) Translational Joints (J1-J3) - Angle', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Joint Angle (rad)', 'FontSize', 10);
    legend('Location', 'bestoutside', 'FontSize', 8, 'NumColumns', 2);
    grid on; box on;
    hold off;
    
    %% 子图2：平移轴速度（J1-J3）
    subplot(2, 2, 2);
    hold on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
    
    for algoIdx = 1:numAlgos
        algoName = matlab.lang.makeValidName(algorithms{algoIdx});
        algoData = allData.(algoName);
        
        for joint = 1:3
            col = sprintf('Vel%d', joint);
            if ismember(col, algoData.Properties.VariableNames)
                plot(1:length(algoData.(col)), algoData.(col), ...
                    'Color', algoColors(algoIdx,:), ...
                    'LineStyle', lineStyles{joint}, ...
                    'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s-J%d', algorithms{algoIdx}, joint));
            end
        end
    end
    
    title('(b) Translational Joints (J1-J3) - Velocity', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Joint Velocity (rad/s)', 'FontSize', 10);
    legend('Location', 'bestoutside', 'FontSize', 8, 'NumColumns', 2);
    grid on; box on;
    hold off;
    
    %% 子图3：旋转轴角度（J4-J8）
    subplot(2, 2, 3);
    hold on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
    
    for algoIdx = 1:numAlgos
        algoName = matlab.lang.makeValidName(algorithms{algoIdx});
        algoData = allData.(algoName);
        
        for joint = 4:8
            col = sprintf('Joint%d', joint);
            if ismember(col, algoData.Properties.VariableNames)
                plot(1:length(algoData.(col)), algoData.(col), ...
                    'Color', algoColors(algoIdx,:), ...
                    'LineStyle', lineStyles{joint-3}, ...
                    'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s-J%d', algorithms{algoIdx}, joint));
            end
        end
    end
    
    title('(c) Rotational Joints (J4-J8) - Angle', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Joint Angle (rad)', 'FontSize', 10);
    legend('Location', 'bestoutside', 'FontSize', 8, 'NumColumns', 2);
    grid on; box on;
    hold off;
    
    %% 子图4：旋转轴速度（J4-J8）
    subplot(2, 2, 4);
    hold on;
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
    
    for algoIdx = 1:numAlgos
        algoName = matlab.lang.makeValidName(algorithms{algoIdx});
        algoData = allData.(algoName);
        
        for joint = 4:8
            col = sprintf('Vel%d', joint);
            if ismember(col, algoData.Properties.VariableNames)
                plot(1:length(algoData.(col)), algoData.(col), ...
                    'Color', algoColors(algoIdx,:), ...
                    'LineStyle', lineStyles{joint-3}, ...
                    'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s-J%d', algorithms{algoIdx}, joint));
            end
        end
    end
    
    title('(d) Rotational Joints (J4-J8) - Velocity', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Point Index', 'FontSize', 10);
    ylabel('Joint Velocity (rad/s)', 'FontSize', 10);
    legend('Location', 'bestoutside', 'FontSize', 8, 'NumColumns', 2);
    grid on; box on;
    hold off;
    
    sgtitle('Multi-Algorithm Joint Motion Analysis (Translational vs Rotational)', ...
        'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');
end

%% 新增：按关节组统计异常
function plotAnomalyCountByJointGroup(allAnomalies, algorithms, jointGroup, groupName)
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
    
    numAlgos = length(algorithms);
    numJoints = length(jointGroup);
    
    % 统计数据：每个算法在该组关节的角度和速度异常总数
    angleCount = zeros(numAlgos, 1);
    velCount = zeros(numAlgos, 1);
    
    for algoIdx = 1:numAlgos
        algoName = matlab.lang.makeValidName(algorithms{algoIdx});
        
        for joint = jointGroup
            if ~isempty(allAnomalies.(algoName).angleAnomalies{joint})
                angleCount(algoIdx) = angleCount(algoIdx) + ...
                    length(allAnomalies.(algoName).angleAnomalies{joint}.points);
            end
            if ~isempty(allAnomalies.(algoName).velocityAnomalies{joint})
                velCount(algoIdx) = velCount(algoIdx) + ...
                    length(allAnomalies.(algoName).velocityAnomalies{joint}.points);
            end
        end
    end
    
    % 绘制堆叠柱状图
    b = bar(1:numAlgos, [angleCount, velCount], 'stacked');
    b(1).FaceColor = [0.2 0.4 0.8];
    b(2).FaceColor = [0.8 0.4 0.2];
    
    if strcmp(groupName, 'Translational')
        titleStr = '(c) Translational Joints Anomaly Count';
    else
        titleStr = '(f) Rotational Joints Anomaly Count';
    end
    
    title(titleStr, 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Total Anomaly Count', 'FontSize', 10);
    xticks(1:numAlgos);
    xticklabels(algorithms);
    xtickangle(15);
    legend({'Angle', 'Velocity'}, 'Location', 'best', 'FontSize', 8);
    
    % 在柱子上显示总数
    for i = 1:numAlgos
        total = angleCount(i) + velCount(i);
        if total > 0
            text(i, total, sprintf('%d', total), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                'FontSize', 9, 'FontWeight', 'bold');
        end
    end
    
    grid on; box on;
end
        
function plotAnomalyCountByAlgorithm(allAnomalies, algorithms, dataType)
            set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
            
            numAlgos = length(algorithms);
            anomalyMatrix = zeros(numAlgos, 8);
            
            for algoIdx = 1:numAlgos
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                
                if strcmp(dataType, 'angle')
                    anomalies = allAnomalies.(algoName).angleAnomalies;
                    titleStr = '(c) Angle Anomaly Count';
                else
                    anomalies = allAnomalies.(algoName).velocityAnomalies;
                    titleStr = '(d) Velocity Anomaly Count';
                end
                
                for joint = 1:8
                    if ~isempty(anomalies{joint})
                        anomalyMatrix(algoIdx, joint) = length(anomalies{joint}.points);
                    end
                end
            end
            
            b = bar(anomalyMatrix, 'grouped');
            colors = lines(numAlgos);
            for i = 1:numAlgos
                b(i).FaceColor = colors(i,:);
            end
            
            title(titleStr, 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('Joint Number', 'FontSize', 10);
            ylabel('Anomaly Count', 'FontSize', 10);
            xticks(1:numAlgos);
            xticklabels(algorithms);
            xtickangle(15);
            legend(arrayfun(@(x) sprintf('J%d', x), 1:8, 'UniformOutput', false), ...
                'Location', 'bestoutside', 'NumColumns', 2, 'FontSize', 8);
            grid on; box on;
        end
        
%% ★★★ 修复后的图2函数 ★★★
function createComparativeDetailFigure(allData, allAnomalies, algorithms, dataTypeFilter)
    % dataTypeFilter: 'angle' 或 'velocity'
    
    % 收集指定类型的异常
    allAnomalyEvents = AlgorithmVisualization.collectAnomalyEventsByType(...
        allAnomalies, algorithms, dataTypeFilter);
    
    if isempty(allAnomalyEvents)
        fprintf('未检测到%s类型的异常需要详细展示\n', dataTypeFilter);
        return;
    end
    
    % 按关节分组，每个关节选择最严重的异常
    jointMap = containers.Map('KeyType', 'double', 'ValueType', 'any');
    
    for i = 1:length(allAnomalyEvents)
        event = allAnomalyEvents(i);
        key = event.joint;
        
        if ~isKey(jointMap, key) || event.severity > jointMap(key).severity
            jointMap(key) = event;
        end
    end
    
    % 获取所有关节的最严重异常
    jointKeys = keys(jointMap);
    jointValues = values(jointMap);
    
    % 转换为结构体数组
    if ~isempty(jointValues)
        % 确保是结构体数组而不是cell数组
        if iscell(jointValues)
            jointEvents = [jointValues{:}];
        else
            jointEvents = jointValues;
        end
        
        % 按严重度排序
        severities = [jointEvents.severity];
        [~, sortIdx] = sort(severities, 'descend');
        topEvents = jointEvents(sortIdx);
        
        % 确保不超过6个
        numEvents = min(6, length(topEvents));
        topEvents = topEvents(1:numEvents);
    else
        topEvents = [];
    end
    
    if isempty(topEvents)
        fprintf('没有可展示的%s异常事件\n', dataTypeFilter);
        return;
    end
    
    numEvents = length(topEvents);
    
    % 显示选择的异常信息
    fprintf('\n=== 选择的%s异常事件 ===\n', dataTypeFilter);
    for i = 1:numEvents
        event = topEvents(i);
        fprintf('%d. 关节%d - %s算法 - 点%d - 严重度%.1f×\n', ...
            i, event.joint, event.algorithm, event.point, event.severity);
    end
    
    if strcmp(dataTypeFilter, 'angle')
        figTitle = 'Fig.2a Joint Angle Anomaly Analysis';
    else
        figTitle = 'Fig.2b Joint Velocity Anomaly Analysis';
    end
    
    fig = figure('Position', [150, 100, 1600, 400 * ceil(numEvents/2)], ...
        'Name', figTitle);
    set(fig, 'Color', 'w');
    
    for eventIdx = 1:numEvents
        event = topEvents(eventIdx);
        subplot(ceil(numEvents/2), 2, eventIdx);
        
        AlgorithmVisualization.plotComparativeAnomalyDetail(...
            allData, allAnomalies, algorithms, event);
    end
    
    if strcmp(dataTypeFilter, 'angle')
        sgtitleText = 'Comparative Analysis of Joint Angle Anomalies (Most Severe per Joint)';
    else
        sgtitleText = 'Comparative Analysis of Joint Velocity Anomalies (Most Severe per Joint)';
    end
    
    sgtitle(sgtitleText, 'FontName', 'Times New Roman', ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    fprintf('展示了 %d 个%s异常事件（优先不同关节）\n', numEvents, dataTypeFilter);
end
        
        %% 按类型收集异常事件
        function allEvents = collectAnomalyEventsByType(allAnomalies, algorithms, dataTypeFilter)
            allEvents = [];
            
            for algoIdx = 1:length(algorithms)
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                
                % 只收集指定类型的异常
                if strcmp(dataTypeFilter, 'angle')
                    anomalyList = allAnomalies.(algoName).angleAnomalies;
                else
                    anomalyList = allAnomalies.(algoName).velocityAnomalies;
                end
                
                for joint = 1:8
                    if ~isempty(anomalyList{joint})
                        anomalyInfo = anomalyList{joint};
                        for i = 1:length(anomalyInfo.points)
                            event = struct(...
                                'algorithm', algorithms{algoIdx}, ...
                                'joint', joint, ...
                                'dataType', dataTypeFilter, ...
                                'point', anomalyInfo.points(i), ...
                                'value', anomalyInfo.values(i), ...
                                'slope', anomalyInfo.slopes(i), ...
                                'severity', anomalyInfo.severity(i));
                            allEvents = [allEvents; event];
                        end
                    end
                end
            end
        end
        
%% 选择最严重的异常事件（避免重复）
function topEvents = selectTopAnomalyEvents(allEvents, topN, excludeEvents)
    if isempty(allEvents)
        topEvents = [];
        return;
    end
    
    % 按严重度排序
    severities = [allEvents.severity];
    [~, sortIdx] = sort(severities, 'descend');
    sortedEvents = allEvents(sortIdx);
    
    % 过滤掉已经展示过的事件
    topEvents = [];
    if nargin >= 3 && ~isempty(excludeEvents)
        for i = 1:length(sortedEvents)
            event = sortedEvents(i);
            eventKey = sprintf('%s_J%d_%s_%d', event.algorithm, event.joint, event.dataType, event.point);
            
            % 检查是否在排除列表中
            if ~any(strcmp(excludeEvents, eventKey))
                topEvents = [topEvents; event];
                if length(topEvents) >= topN
                    break;
                end
            end
        end
    else
        % 如果没有排除列表，直接选择前topN个
        numEvents = min(topN, length(sortedEvents));
        topEvents = sortedEvents(1:numEvents);
    end
end
        
        %% 绘制对比式异常细节（修复版，无黑边框）
        function plotComparativeAnomalyDetail(allData, allAnomalies, algorithms, event)
            hold on;
            set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
            set(gca, 'LineWidth', 1.2);
            
            numAlgos = length(algorithms);
            colors = lines(numAlgos);
            
            centerPoint = event.point;
            windowSize = 40;
            startIdx = max(1, centerPoint - windowSize);
            endIdx = centerPoint + windowSize;
            
            if strcmp(event.dataType, 'angle')
                colName = sprintf('Joint%d', event.joint);
                yLabel = 'Angle (rad)';
                typeStr = 'Angle';
            else
                colName = sprintf('Vel%d', event.joint);
                yLabel = 'Velocity (rad/s)';
                typeStr = 'Velocity';
            end
            
            % 收集窗口内所有异常
            anomaliesInWindow = [];
            
            for algoIdx = 1:numAlgos
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                
                if strcmp(event.dataType, 'angle')
                    anomalies = allAnomalies.(algoName).angleAnomalies;
                else
                    anomalies = allAnomalies.(algoName).velocityAnomalies;
                end
                
                if ~isempty(anomalies{event.joint})
                    anomalyInfo = anomalies{event.joint};
                    inWindow = (anomalyInfo.points >= startIdx) & (anomalyInfo.points <= endIdx);
                    
                    if any(inWindow)
                        windowPoints = anomalyInfo.points(inWindow);
                        windowValues = anomalyInfo.values(inWindow);
                        windowSeverity = anomalyInfo.severity(inWindow);
                        
                        for i = 1:length(windowPoints)
                            anomaly = struct(...
                                'algorithm', algorithms{algoIdx}, ...
                                'algoIdx', algoIdx, ...
                                'point', windowPoints(i), ...
                                'value', windowValues(i), ...
                                'severity', windowSeverity(i), ...
                                'isPrimary', windowPoints(i) == centerPoint && ...
                                            strcmp(event.algorithm, algorithms{algoIdx}));
                            anomaliesInWindow = [anomaliesInWindow; anomaly];
                        end
                    end
                end
            end
            
            % 绘制轨迹
            legendEntries = {};
            hasData = false;
            
            for algoIdx = 1:numAlgos
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                algoData = allData.(algoName);
                
                if ~ismember(colName, algoData.Properties.VariableNames)
                    continue;
                end
                
                dataCol = algoData.(colName);
                actualEndIdx = min(length(dataCol), endIdx);
                actualStartIdx = max(1, startIdx);
                
                if actualEndIdx <= actualStartIdx
                    continue;
                end
                
                xData = actualStartIdx:actualEndIdx;
                yData = dataCol(actualStartIdx:actualEndIdx);
                
                isAnomalySource = strcmp(event.algorithm, algorithms{algoIdx});
                
                if isAnomalySource
                    lineStyle = '-';
                    lineWidth = 2.5;
                    alpha = 1.0;
                else
                    lineStyle = '--';
                    lineWidth = 1.5;
                    alpha = 0.7;
                end
                
                h = plot(xData, yData, 'LineStyle', lineStyle, ...
                    'Color', colors(algoIdx,:), 'LineWidth', lineWidth);
                h.Color(4) = alpha;
                
                legendEntries{end+1} = algorithms{algoIdx};
                if isAnomalySource
                    legendEntries{end} = [legendEntries{end} ' (Primary)'];
                end
                
                hasData = true;
            end
            
            if ~hasData
                text(0.5, 0.5, 'No Data Available', ...
                    'Units', 'normalized', 'HorizontalAlignment', 'center');
                return;
            end
            
            % 标记异常点
            markerStyles = {'o', 's', 'd', '^', 'v', '>', '<', 'p'};
            
            for i = 1:length(anomaliesInWindow)
                anomaly = anomaliesInWindow(i);
                markerIdx = mod(anomaly.algoIdx - 1, length(markerStyles)) + 1;
                
                if anomaly.isPrimary
                    % ★★★ 主要异常：大标记，无黑边框 ★★★
                    markerSize = 180;
                    faceAlpha = 1.0;
                    
                    scatter(anomaly.point, anomaly.value, markerSize, ...
                        colors(anomaly.algoIdx,:), 'filled', ...
                        'Marker', markerStyles{markerIdx}, ...
                        'LineWidth', 0.5, ...
                        'MarkerFaceAlpha', faceAlpha);
                    
                     text(anomaly.point, anomaly.value, ...
                        sprintf('  %.1f×', anomaly.severity), ...
                        'FontSize', 8, 'Color', colors(anomaly.algoIdx,:));
                    
                else
                    % 次要异常
                    markerSize = 120;
                    edgeWidth = 1.5;
                    faceAlpha = 0.6;
                    
                    scatter(anomaly.point, anomaly.value, markerSize, ...
                        colors(anomaly.algoIdx,:), 'filled', ...
                        'Marker', markerStyles{markerIdx}, ...
                        'MarkerEdgeColor', colors(anomaly.algoIdx,:), ...
                        'LineWidth', edgeWidth, ...
                        'MarkerFaceAlpha', faceAlpha);
                    
                    text(anomaly.point, anomaly.value, ...
                        sprintf('  %.1f×', anomaly.severity), ...
                        'FontSize', 8, 'Color', colors(anomaly.algoIdx,:));
                end
            end
            
            % 参考线
            yl = ylim;
            plot([centerPoint centerPoint], yl, 'r:', 'LineWidth', 2);
            
            % 标题
            numAnomaliesInWindow = length(anomaliesInWindow);
            titleStr = sprintf('[%s] Joint %d %s - Point %d\nSeverity: %.1f× | Anomalies in Window: %d', ...
                event.algorithm, event.joint, typeStr, event.point, event.severity, numAnomaliesInWindow);
            title(titleStr, 'FontSize', 11, 'FontWeight', 'bold');
            xlabel('Point Index', 'FontSize', 10);
            ylabel(yLabel, 'FontSize', 10);
            
            legend(legendEntries, 'Location', 'best', 'FontSize', 8);
            
            % 信息框
            infoStr = sprintf(['Window: [%d, %d]\n' ...
                               'Center: %d\n' ...
                               'Anomalies: %d'], ...
                actualStartIdx, actualEndIdx, centerPoint, numAnomaliesInWindow);
            
            text(0.02, 0.98, infoStr, 'Units', 'normalized', ...
                'FontSize', 8, 'VerticalAlignment', 'top', ...
                'BackgroundColor', [0.95 0.95 0.95 0.9], ...
                'EdgeColor', 'k', 'Margin', 3);
            
            grid on; box on;
            hold off;
        end
        
        %% 图3：算法对比（保持原逻辑）
        function createAlgorithmComparisonFigure(allAnomalies, algorithms)
            fig = figure('Position', [200, 200, 1400, 800], 'Name', 'Fig.3 Algorithm Comparison');
            set(fig, 'Color', 'w');
            
            %subplot(2, 2, 1);
            %AlgorithmVisualization.plotTotalAnomalyComparison(allAnomalies, algorithms);
            
            %subplot(2, 2, 2);
            %AlgorithmVisualization.plotSeverityComparison(allAnomalies, algorithms);
            
            %subplot(2, 2, [3,4]);
            AlgorithmVisualization.plotAnomalyHeatmapComparison(allAnomalies, algorithms);
        end
        
        function plotTotalAnomalyComparison(allAnomalies, algorithms)
            set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
            
            numAlgos = length(algorithms);
            angleCounts = zeros(numAlgos, 1);
            velCounts = zeros(numAlgos, 1);
            
            for algoIdx = 1:numAlgos
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                
                for joint = 1:8
                    if ~isempty(allAnomalies.(algoName).angleAnomalies{joint})
                        angleCounts(algoIdx) = angleCounts(algoIdx) + ...
                            length(allAnomalies.(algoName).angleAnomalies{joint}.points);
                    end
                    if ~isempty(allAnomalies.(algoName).velocityAnomalies{joint})
                        velCounts(algoIdx) = velCounts(algoIdx) + ...
                            length(allAnomalies.(algoName).velocityAnomalies{joint}.points);
                    end
                end
            end
            
            x = 1:numAlgos;
            width = 0.35;
            bar(x - width/2, angleCounts, width, 'FaceColor', [0.2 0.4 0.8]);
            hold on;
            bar(x + width/2, velCounts, width, 'FaceColor', [0.8 0.4 0.2]);
            
            title('(a) Total Anomaly Count', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Count', 'FontSize', 10);
            xticks(1:numAlgos);
            xticklabels(algorithms);
            xtickangle(15);
            legend({'Angle', 'Velocity'}, 'Location', 'best');
            grid on; box on;
            hold off;
        end
        
        function plotSeverityComparison(allAnomalies, algorithms)
            set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
            
            numAlgos = length(algorithms);
            avgSeverity = zeros(numAlgos, 2);
            
            for algoIdx = 1:numAlgos
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                
                angleSev = [];
                velSev = [];
                
                for joint = 1:8
                    if ~isempty(allAnomalies.(algoName).angleAnomalies{joint})
                        angleSev = [angleSev; allAnomalies.(algoName).angleAnomalies{joint}.severity(:)];
                    end
                    if ~isempty(allAnomalies.(algoName).velocityAnomalies{joint})
                        velSev = [velSev; allAnomalies.(algoName).velocityAnomalies{joint}.severity(:)];
                    end
                end
                
                if ~isempty(angleSev)
                    avgSeverity(algoIdx, 1) = mean(angleSev);
                end
                if ~isempty(velSev)
                    avgSeverity(algoIdx, 2) = mean(velSev);
                end
            end
            
            x = 1:numAlgos;
            width = 0.35;
            bar(x - width/2, avgSeverity(:,1), width, 'FaceColor', [0.2 0.4 0.8]);
            hold on;
            bar(x + width/2, avgSeverity(:,2), width, 'FaceColor', [0.8 0.4 0.2]);
            
            title('(b) Average Anomaly Severity', 'FontSize', 12, 'FontWeight', 'bold');
            ylabel('Severity (×)', 'FontSize', 10);
            xticks(1:numAlgos);
            xticklabels(algorithms);
            xtickangle(15);
            legend({'Angle', 'Velocity'}, 'Location', 'best');
            grid on; box on;
            hold off;
        end
        
        function plotAnomalyHeatmapComparison(allAnomalies, algorithms)
            set(gca, 'FontName', 'Times New Roman', 'FontSize', 10);
            
            numAlgos = length(algorithms);
            heatmapData = zeros(numAlgos, 8);
            
            for algoIdx = 1:numAlgos
                algoName = matlab.lang.makeValidName(algorithms{algoIdx});
                
                for joint = 1:8
                    angleCount = 0;
                    velCount = 0;
                    
                    if ~isempty(allAnomalies.(algoName).angleAnomalies{joint})
                        angleCount = length(allAnomalies.(algoName).angleAnomalies{joint}.points);
                    end
                    if ~isempty(allAnomalies.(algoName).velocityAnomalies{joint})
                        velCount = length(allAnomalies.(algoName).velocityAnomalies{joint}.points);
                    end
                    
                    heatmapData(algoIdx, joint) = angleCount + velCount;
                end
            end
            
            imagesc(heatmapData);
            colormap('hot');
            colorbar;
            
            title('Anomaly Distribution Heatmap', 'FontSize', 12, 'FontWeight', 'bold');
            xlabel('Joint Number', 'FontSize', 10);
            ylabel('Algorithm', 'FontSize', 10);
            xticks(1:8);
            xticklabels(arrayfun(@(x) sprintf('J%d', x), 1:8, 'UniformOutput', false));
            yticks(1:numAlgos);
            yticklabels(algorithms);
            
            for i = 1:numAlgos
                for j = 1:8
                    if heatmapData(i,j) > 0
                        text(j, i, sprintf('%d', heatmapData(i,j)), ...
                            'HorizontalAlignment', 'center', 'Color', 'w', ...
                            'FontWeight', 'bold', 'FontSize', 9);
                    end
                end
            end
        end
        
        function printAnomalyStatisticsTable(allAnomalies, algorithms)
    % 打印四种算法角度与速度异常点数及平均严重度统计表
    
    numAlgos = length(algorithms);
    
    angleAnomalyCounts = zeros(numAlgos,1);
    angleSeverityMeans = zeros(numAlgos,1);
    velocityAnomalyCounts = zeros(numAlgos,1);
    velocitySeverityMeans = zeros(numAlgos,1);
    
    for i = 1:numAlgos
        algoName = matlab.lang.makeValidName(algorithms{i});
        
        angleAnomalies = allAnomalies.(algoName).angleAnomalies;
        velocityAnomalies = allAnomalies.(algoName).velocityAnomalies;
        
        % 统计角度异常
        countAngle = 0;
        sevSumAngle = 0;
        sevNumAngle = 0;
        for j = 1:8
            if ~isempty(angleAnomalies{j})
                countAngle = countAngle + length(angleAnomalies{j}.points);
                sevSumAngle = sevSumAngle + sum(angleAnomalies{j}.severity);
                sevNumAngle = sevNumAngle + length(angleAnomalies{j}.severity);
            end
        end
        angleAnomalyCounts(i) = countAngle;
        if sevNumAngle > 0
            angleSeverityMeans(i) = sevSumAngle / sevNumAngle;
        else
            angleSeverityMeans(i) = NaN;
        end
        
        % 统计速度异常
        countVel = 0;
        sevSumVel = 0;
        sevNumVel = 0;
        for j = 1:8
            if ~isempty(velocityAnomalies{j})
                countVel = countVel + length(velocityAnomalies{j}.points);
                sevSumVel = sevSumVel + sum(velocityAnomalies{j}.severity);
                sevNumVel = sevNumVel + length(velocityAnomalies{j}.severity);
            end
        end
        velocityAnomalyCounts(i) = countVel;
        if sevNumVel > 0
            velocitySeverityMeans(i) = sevSumVel / sevNumVel;
        else
            velocitySeverityMeans(i) = NaN;
        end
    end
    
    % 打印格式化的LaTeX表格代码
    fprintf('\\begin{table}[htbp]\n');
    fprintf('\\centering\n');
    fprintf('\\caption{四种算法角度及速度异常点统计}\n');
    fprintf('\\label{tab:anomaly_statistics}\n');
    fprintf('\\begin{tabular}{lcccc}\n');
    fprintf('\\toprule\n');
    fprintf('算法名称 & 角度异常点数 & 角度平均严重度 & 速度异常点数 & 速度平均严重度 \\\\\n');
    fprintf('\\midrule\n');
    for i = 1:numAlgos
        fprintf('%s & %d & %.2f & %d & %.2f \\\\\n', ...
            algorithms{i}, ...
            angleAnomalyCounts(i), angleSeverityMeans(i), ...
            velocityAnomalyCounts(i), velocitySeverityMeans(i));
    end
    fprintf('\\bottomrule\n');
    fprintf('\\end{tabular}\n');
    fprintf('\\end{table}\n');
    

end
    end
end