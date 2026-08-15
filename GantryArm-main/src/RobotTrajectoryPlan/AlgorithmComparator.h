#pragma once
#include "TrajectoryOptimizer.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <array>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <cmath>

class AlgorithmComparator {
public:
    // 性能指标结构
    struct PerformanceMetrics {
        std::string algorithm_name;
        double average_time = 0.0;
        double success_rate_5ms = 0.0;
        double success_rate_10ms = 0.0;

        std::array<double, 8> avg_joint_movement = {};
        std::array<int, 8> joint_limit_approaches = {};

        // 时间序列数据
        std::vector<double> computation_times;
        std::vector<std::array<double, 8>> joint_angles;
        std::vector<std::array<double, 8>> joint_velocities;
        std::vector<bool> optimization_success;

        int total_points = 0;
        int successful_points = 0;
        double avg_time_ms = 0.0;
        double rate_5ms = 0.0;  // <= 5ms比例
        double rate_10ms = 0.0; // <=10ms比例
        // 阈值设置
        std::array<double, 8> limit_cost_thresholds = {
        10, 0, 0,  // J1-J3: 10mm
        5 * M_PI / 180.0, 0 * M_PI / 180.0,  // J4-J5: 5度
        2 * M_PI / 180.0, 0 * M_PI / 180.0,  // J6-J7: 5度
        2 * M_PI / 180.0   // J8: 5度
        };
    };

private:
    RobotArm& arm_;
    CollisionSystem& collision_system_;
    std::vector<std::string> algorithm_types_;
    OptimizeParams params_;
    RobotArm::JointLimits joint_limits_;

public:
    AlgorithmComparator(RobotArm& arm, CollisionSystem& collision_system,
        const OptimizeParams& params = OptimizeParams());

    // 设置要比较的算法类型
    void setAlgorithmTypes(const std::vector<std::string>& types);

    // 比较多个算法性能
    std::vector<PerformanceMetrics> compareAlgorithms(const std::string& trajectory_file, bool isMm = true);
    AlgorithmComparator::PerformanceMetrics evaluateJointTrajectory(const std::string& alg_type, const std::vector<std::array<double, 8>>& joint_trajectory);
    AlgorithmComparator::PerformanceMetrics
        evaluateJointTrajectoryWithTimes(
            const std::string& alg_type,
            const std::vector<std::array<double, 8>>& joint_trajectory,
            const std::vector<double>& elapsed_times_ms);
    
    void exportMetricsToCSV(PerformanceMetrics metrics, const std::string& filename);
    void exportMetricsToCSV(std::vector<PerformanceMetrics> all_metrics, const std::string& filename);
    std::vector<PerformanceMetrics> compareAlgorithms(const std::vector<Eigen::Matrix4d>& trajectory);

    // 导出简化的比较表格
    void exportSimplifiedComparison(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "comparison.csv");

    // 获取当前设置的算法类型
    const std::vector<std::string>& getAlgorithmTypes() const;

private:
    // 计算关节限位成本
    double calcJointLimitCost(const std::array<double, 8>& q) const;

    // 计算单个关节的限位成本
    double calcSingleJointLimitCost(double joint_angle, int joint_index,
        const PerformanceMetrics& metrics) const;

    // 评估单个算法
    PerformanceMetrics evaluateAlgorithm(const std::string& alg_type,
        const std::vector<Eigen::Matrix4d>& trajectory);

    // 检查关节极限逼近
    void checkJointLimitApproaches(const std::array<double, 8>& joints,
        PerformanceMetrics& metrics);

    // 计算关节速度
    std::array<double, 8> calcJointVelocities(const std::array<double, 8>& joints,
        const std::array<double, 8>& ref_joints) const;

    // 计算关节突变次数
    int calculateJointSuddenChanges(const PerformanceMetrics& metrics) const;

    // 计算速度突变次数
    int calculateVelocitySuddenChanges(const PerformanceMetrics& metrics) const;

    // 打印指标摘要
    void printMetricsSummary(const PerformanceMetrics& metrics);

    // 计算关节突变次数（带详细统计）
    int calculateJointSuddenChangesDetailed(const PerformanceMetrics& metrics,
        std::array<int, 8>& per_joint_counts) const;

    // 计算速度突变次数（带详细统计）
    int calculateVelocitySuddenChangesDetailed(const PerformanceMetrics& metrics,
        std::array<int, 8>& per_joint_counts) const;

    std::array<double, 8> accel_thresholds = {
    50, 50, 50,  // J1-J3: 50mm/s²
    15 * M_PI / 180.0,20 * M_PI / 180.0,  // J4-J5: 10°/s²
    15 * M_PI / 180.0,15 * M_PI / 180.0,  // J6-J7: 10°/s²
    15 * M_PI / 180.0   // J8: 10°/s²
    };
    std::array<double, 8> thresholds = {
    80, 0, 0,  // J1-J3: 10mm
    15 * M_PI / 180.0, 12 * M_PI / 180.0,  // J4-J5: 5度
    15 * M_PI / 180.0, 15 * M_PI / 180.0,  // J6-J7: 5度
    15 * M_PI / 180.0   // J8: 5度
    };

};