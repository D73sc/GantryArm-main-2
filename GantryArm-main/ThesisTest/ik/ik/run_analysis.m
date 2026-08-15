% run_paper_figures.m
clear; clc; close all;

% 设置文件路径
baseFilename = 'multi_algorithm_comparison';

try
    % 加载数据
    timeSeriesData = readtable([baseFilename '_timeseries.csv']);
    mainData = readtable([baseFilename '_main.csv']);
    
    disp('=== 数据加载成功 ===');
    disp(['时间序列数据大小: ', num2str(size(timeSeriesData))]);
    disp(['主要数据大小: ', num2str(size(mainData))]);
    
    % 图1：主性能对比图
    disp('正在生成图1: 主性能对比图...');
    AlgorithmVisualization.plotMainPerformance(timeSeriesData);
    
    % 图2：智能关节异常展示
    AlgorithmVisualization.plotJointMotionAnalysis(timeSeriesData);
    
    disp('=== 所有图表生成完成 ===');

    
catch ME
    disp('错误信息:');
    disp(ME.message);
    disp('请检查:');
    disp('1. CSV文件是否存在');
    disp('2. 列名是否匹配');
    disp('3. 数据格式是否正确');
end