#pragma once
#include "TrajectoryOptimizer.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <map>
#include <numeric>

class AlgorithmComparator {
public:
    // 轨迹点结构 - 对应CSV格式: x,y,z,qx,qy,qz,qw
    struct TrajectoryPoint {
        double x, y, z;
        double qx, qy, qz, qw;

        Eigen::Matrix4d toPose() const {
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

            Eigen::Quaterniond q(qw, qx, qy, qz);
            pose.block<3, 3>(0, 0) = q.toRotationMatrix();
            return pose;
        }
    };

    struct PerformanceMetrics {
        std::string algorithm_name;
        double average_time = 0.0;
        double success_rate_150ms = 0.0;
        std::array<double, 8> avg_joint_movement = {};
        std::array<int, 8> joint_limit_approaches = {};
        double avg_joint_limit_cost = 0.0;

        // 速度相关指标
        std::array<double, 8> avg_joint_velocity = {};
        std::array<double, 8> max_joint_velocity = {};
        std::array<double, 8> velocity_variance = {};
        std::array<double, 8> velocity_jerk = {};
        double overall_velocity_smoothness = 0.0;
        std::array<int, 8> velocity_limit_violations = {};

        // 时间序列数据（每个轨迹点的详细数据）
        std::vector<double> computation_times;          // 每个点的计算时间
        std::vector<double> joint_limit_costs;         // 每个点的关节限位成本
        std::vector<std::array<double, 8>> joint_angles; // 每个点的关节角度
        std::vector<std::array<double, 8>> joint_velocities; // 每个点的关节速度
        std::vector<bool> optimization_success;         // 每个点的优化成功状态

        int total_points = 0;
        int successful_points = 0;
        int points_within_150ms = 0;

        // 限制阈值
        std::array<double, 8> velocity_limits = { 50, 50, 50, 3, 3, 3, 3, 3 };
        std::array<double, 8> limit_cost_thresholds = { 20, 20, 50, 3 / 180.0 * M_PI, 0, 3 / 180.0 * M_PI, 2 / 180.0 * M_PI, 0 };
    };

    // 单次优化结果
    struct OptimizationResult {
        std::array<double, 8> joints;
        std::array<double, 8> velocities;
        double computation_time_ms;
        bool success;
        bool within_150ms;
        double joint_limit_cost;
        double velocity_smoothness;

    };

private:
    RobotArm& arm_;
    CollisionSystem& collision_system_;
    std::vector<std::string> algorithm_types_;
    OptimizeParams params_;
    RobotArm::JointLimits joint_limits_;

public:
    AlgorithmComparator(RobotArm& arm, CollisionSystem& collision_system,
        const OptimizeParams& params = OptimizeParams())
        : arm_(arm), collision_system_(collision_system), params_(params), joint_limits_(arm_.getJointLimits()) {

        // 默认比较算法列表
        algorithm_types_ = { "GA", "NSGA" };
    }

    // 设置要比较的算法类型
    void setAlgorithmTypes(const std::vector<std::string>& types) {
        algorithm_types_ = types;
    }

    // 设置关节极限逼近的成本阈值
    void setJointLimitThresholds(const std::array<double, 8>& thresholds) {
        // 可以在构造后调整阈值
    }

    //// 从CSV加载轨迹
    //std::vector<TrajectoryPoint> loadTrajectoryFromCSV(const std::string& filename) {
    //    std::vector<TrajectoryPoint> trajectory;
    //    std::ifstream file(filename);

    //    if (!file.is_open()) {
    //        throw std::runtime_error("无法打开轨迹文件: " + filename);
    //    }

    //    std::string line;
    //    bool first_line = true;

    //    while (std::getline(file, line)) {
    //        // 跳过标题行
    //        if (first_line) {
    //            first_line = false;
    //            if (line.find("x") != std::string::npos) continue;
    //        }

    //        std::stringstream ss(line);
    //        std::string cell;
    //        TrajectoryPoint point;

    //        try {
    //            std::getline(ss, cell, ','); point.x = std::stod(cell);
    //            std::getline(ss, cell, ','); point.y = std::stod(cell);
    //            std::getline(ss, cell, ','); point.z = std::stod(cell);
    //            std::getline(ss, cell, ','); point.qx = std::stod(cell);
    //            std::getline(ss, cell, ','); point.qy = std::stod(cell);
    //            std::getline(ss, cell, ','); point.qz = std::stod(cell);
    //            std::getline(ss, cell, ','); point.qw = std::stod(cell);

    //            trajectory.push_back(point);
    //        }
    //        catch (const std::exception& e) {
    //            std::cerr << "解析轨迹点失败: " << line << ", 错误: " << e.what() << std::endl;
    //        }
    //    }

    //    return trajectory;
    //}

    // 比较多个算法性能
    std::vector<PerformanceMetrics> compareAlgorithms(const std::string& trajectory_file,bool isMm=true) {
        if (isMm)
        {
            auto trajectory = loadTrajectoryFromCSV(trajectory_file,true);
            return compareAlgorithms(trajectory);
        }
        else
        {
            auto trajectory = loadTrajectoryFromCSV(trajectory_file);
            return compareAlgorithms(trajectory);
        }

    }

    std::vector<PerformanceMetrics> compareAlgorithms(const std::vector<Matrix4d>& trajectory) {
        std::vector<PerformanceMetrics> results;

        std::cout << "开始算法比较，轨迹点数: " << trajectory.size() << std::endl;

        for (const std::string& alg_type : algorithm_types_) {
            std::cout << "\n测试算法: " << alg_type << std::endl;

            auto metrics = evaluateAlgorithm(alg_type, trajectory);
            results.push_back(metrics);

            // 实时显示进度
            std::cout << "算法 " << alg_type << " 完成" << std::endl;
            printMetricsSummary(metrics);
        }

        return results;
    }

private:
    // 计算关节限位成本 - 参考你的实现
    double calcJointLimitCost(const std::array<double, 8>& q) const {
        double cost = 0.0;

        for (int i = 0; i < 8; i++) {
            double safe_range = 0.2 * (joint_limits_.max[i] - joint_limits_.min[i]);
            cost += params_.joint_limits_weights[i] *
                (2.0 / (1.0 + exp((q[i] - joint_limits_.min[i]) / safe_range)) +
                    2.0 / (1.0 + exp(-(q[i] - joint_limits_.max[i]) / safe_range)));
        }

        return cost;
    }

    // 计算单个关节的限位成本
    double calcSingleJointLimitCost(double joint_angle, int joint_index, PerformanceMetrics& metrics) const {


        // 计算安全边界（从极限值向内缩进的距离）
        double safe_margin = metrics.limit_cost_thresholds[joint_index];
        double safe_min, safe_max;
        if (joint_index < 3)
        {
            safe_min = joint_limits_.min[joint_index]  + safe_margin;
            safe_max = joint_limits_.max[joint_index]  - safe_margin;
        }
        else
        {
            // 计算安全范围的上下界
             safe_min = joint_limits_.min[joint_index]  + safe_margin;
             safe_max = joint_limits_.max[joint_index]  - safe_margin;
        }


        // 如果在安全范围内，成本为0
        if (joint_angle >= safe_min && joint_angle <= safe_max) {
            return 0.0;
        }

        // 计算距离安全限位的偏差
        double cost = 0.0;
        if (joint_angle < safe_min) {
            cost = safe_min - joint_angle;  // 超出下限的距离
        }
        else if (joint_angle > safe_max) {
            cost = joint_angle - safe_max;  // 超出上限的距离
        }

        return cost;
    }

    // 评估单个算法
    PerformanceMetrics evaluateAlgorithm(const std::string& alg_type,
        const std::vector<Matrix4d>& trajectory) {
        PerformanceMetrics metrics;
        metrics.algorithm_name = alg_type;
        metrics.total_points = trajectory.size();

        // 预留空间提高性能
        metrics.computation_times.reserve(trajectory.size());
        metrics.joint_limit_costs.reserve(trajectory.size());
        metrics.joint_angles.reserve(trajectory.size());
        metrics.joint_velocities.reserve(trajectory.size());
        metrics.optimization_success.reserve(trajectory.size());

        // 创建优化器
        auto optimizer = TrajectoryOptimizer::create(alg_type, arm_, collision_system_, params_);
        if (!optimizer) {
            throw std::runtime_error("无法创建优化器: " + alg_type);
        }

        // 初始化状态
        Eigen::Matrix4d prev_target_pose = Eigen::Matrix4d::Identity();
        std::array<double, 8> prev_joints = {};
        RobotArm::Solution prev_solution;
        std::vector<TrajectoryOptimizer::Individual> prev_population;

        std::vector<double> computation_times;
        std::vector<double> joint_limit_costs;
        std::array<double, 8> total_movements = {};
        std::vector<std::array<double, 8>> trajectory_velocities;

        for (size_t i = 0; i < trajectory.size(); ++i) {
            const auto target_pose = trajectory[i];

            // 显示进度
            if ((i + 1) % 100 == 0 || i == 0) {
                std::cout << "  " << alg_type << " - 进度: " << (i + 1) << "/" << trajectory.size() << std::endl;
            }

            auto start_time = std::chrono::high_resolution_clock::now();

            try {
                // 执行优化
                auto optimal_joints = optimizer->optimize(target_pose, prev_target_pose,
                    prev_solution, prev_population);

                auto end_time = std::chrono::high_resolution_clock::now();
                double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

                // 计算关节移动距离
                double total_movement = 0.0;
                for (int j = 0; j < 8; ++j) {
                    double movement = std::abs(optimal_joints[j] - prev_joints[j]);
                    total_movement += movement;
                    total_movements[j] += movement;
                }

                if (total_movement > 1e-10) {
                    computation_times.push_back(elapsed_ms);
                    metrics.successful_points++;

                    if (elapsed_ms <= 150) {
                        metrics.points_within_150ms++;
                    }

                    // 计算关节限位成本
                    double limit_cost = calcJointLimitCost(optimal_joints);
                    joint_limit_costs.push_back(limit_cost);

                    // 计算关节速度
                    auto velocities = calcJointVelocities(optimal_joints, prev_joints);
                    trajectory_velocities.push_back(velocities);

                    // 检查关节极限逼近
                    checkJointLimitApproaches(optimal_joints, metrics);

                    // 存储时间序列数据
                    metrics.computation_times.push_back(elapsed_ms);
                    metrics.joint_limit_costs.push_back(limit_cost);
                    metrics.joint_angles.push_back(optimal_joints);
                    metrics.joint_velocities.push_back(velocities);
                    metrics.optimization_success.push_back(true);
                }
                else {
                    // 无有效移动，记录为失败
                    metrics.computation_times.push_back(0);
                    metrics.joint_limit_costs.push_back(0);
                    metrics.joint_angles.push_back(prev_joints);
                    metrics.joint_velocities.push_back({});
                    metrics.optimization_success.push_back(false);
                }

                // 更新状态
                prev_joints = optimal_joints;
                prev_target_pose = target_pose;

            }
            catch (const std::exception& e) {
                std::cerr << alg_type << " 优化失败在点 " << i << ": " << e.what() << std::endl;

                // 记录失败数据
                metrics.computation_times.push_back(1000.0);
                metrics.joint_limit_costs.push_back(0.0);
                metrics.joint_angles.push_back(prev_joints);
                metrics.joint_velocities.push_back({});
                metrics.optimization_success.push_back(false);

                computation_times.push_back(1000.0);
                joint_limit_costs.push_back(0.0);
            }
        }

        // 计算最终指标
        if (!computation_times.empty()) {
            metrics.average_time = std::accumulate(computation_times.begin(),
                computation_times.end(), 0.0) / computation_times.size();
        }

        if (!joint_limit_costs.empty()) {
            metrics.avg_joint_limit_cost = std::accumulate(joint_limit_costs.begin(),
                joint_limit_costs.end(), 0.0) / joint_limit_costs.size();
        }

        metrics.success_rate_150ms = static_cast<double>(metrics.points_within_150ms) / metrics.total_points * 100.0;

        // 计算平均关节移动距离
        if (metrics.successful_points > 0) {
            for (int j = 0; j < 8; ++j) {
                metrics.avg_joint_movement[j] = total_movements[j] / metrics.successful_points;
            }
        }

        // 计算速度平滑性指标
        std::vector<std::vector<std::array<double, 8>>> all_trajectories = { trajectory_velocities };
        calculateVelocityMetrics(all_trajectories, metrics);

        return metrics;
    }

    // 检查关节极限逼近 - 基于你的限位成本计算
    void checkJointLimitApproaches(const std::array<double, 8>& joints, PerformanceMetrics& metrics) {
        for (int i = 0; i < 8; ++i) {
            // 计算单个关节的限位成本
            double joint_cost = calcSingleJointLimitCost(joints[i], i, metrics);

            // 如果单关节限位成本超过阈值，则认为逼近极限
            if (joint_cost > 0) {
                metrics.joint_limit_approaches[i]++;
            }
        }
    }

    void printMetricsSummary(const PerformanceMetrics& metrics) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  平均时间(AT): " << metrics.average_time << "ms" << std::endl;
        std::cout << "  10ms成功率(SR10): " << metrics.success_rate_150ms << "%" << std::endl;
        std::cout << "  平均关节限位成本: " << metrics.avg_joint_limit_cost << std::endl;
        std::cout << "  整体速度平滑度: " << metrics.overall_velocity_smoothness << std::endl;
        std::cout << "  成功点数: " << metrics.successful_points << "/" << metrics.total_points << std::endl;

        std::cout << "  关节平均移动距离(Mi): ";
        for (int i = 0; i < 8; ++i) {
            std::cout << "J" << (i + 1) << ":" << metrics.avg_joint_movement[i];
            if (i < 7) std::cout << ", ";
        }
        std::cout << std::endl;

        std::cout << "  关节平均速度(°或mm/s^2): ";
        for (int i = 0; i < 8; ++i) {
            if(i<3)
            std::cout << "J" << (i + 1) << ":" << metrics.avg_joint_velocity[i];
            else
                std::cout << "J" << (i + 1) << ":" << metrics.avg_joint_velocity[i]*180/PI;

            if (i < 7) std::cout << ", ";
        }
        std::cout << std::endl;

        std::cout << "  关节速度方差(平滑性°或mm/s^2): ";
        for (int i = 0; i < 8; ++i) {
            if (i < 3)
                std::cout << "J" << (i + 1) << ":" << metrics.velocity_variance[i];
            else
                std::cout << "J" << (i + 1) << ":" << metrics.velocity_variance[i] * 180 / PI;
            if (i < 7) std::cout << ", ";
        }
        std::cout << std::endl;

        std::cout << "  极限逼近次数(TCL): ";
        int total_approaches = 0;
        for (int i = 0; i < 8; ++i) {
            std::cout << "J" << (i + 1) << ":" << metrics.joint_limit_approaches[i];
            total_approaches += metrics.joint_limit_approaches[i];
            if (i < 7) std::cout << ", ";
        }
        std::cout << " (总计:" << total_approaches << ")" << std::endl;

        std::cout << "  速度限制违反次数: ";
        int total_violations = 0;
        for (int i = 0; i < 8; ++i) {
            std::cout << "J" << (i + 1) << ":" << metrics.velocity_limit_violations[i];
            total_violations += metrics.velocity_limit_violations[i];
            if (i < 7) std::cout << ", ";
        }
        std::cout << " (总计:" << total_violations << ")" << std::endl;
    }

    // 基于你的函数计算关节速度
    std::array<double, 8> calcJointVelocities(const std::array<double, 8>& joints,
        const std::array<double, 8>& ref_joints) const {
        std::array<double, 8> velocities = {};

        double sum_sq = 0.0;
        // 前3个关节除以10
        for (int i = 0; i < 3; i++) {
            sum_sq += pow((joints[i] - ref_joints[i]) / 10.0, 2);
        }
        // 后5个关节直接计算
        for (int i = 3; i < 8; i++) {
            sum_sq += pow(joints[i] - ref_joints[i], 2);
        }

        double T = sqrt(sum_sq) / params_.velocities;

        if (T > 1e-6) {  // 避免除零
            for (int i = 0; i < 3; i++) {
                velocities[i] = (joints[i] - ref_joints[i]) / 10.0 / T;
            }
            for (int i = 3; i < 8; i++) {
                velocities[i] = (joints[i] - ref_joints[i]) / T;
            }
        }

        return velocities;
    }

    // 计算速度平滑性指标
    void calculateVelocityMetrics(const std::vector<std::vector<std::array<double, 8>>>& all_velocities,
        PerformanceMetrics& metrics) {
        if (all_velocities.empty()) return;

        int total_valid_points = 0;
        std::array<double, 8> velocity_sums = {};
        std::array<double, 8> velocity_sq_sums = {};

        // 计算各关节速度统计
        for (const auto& trajectory_velocities : all_velocities) {
            for (const auto& velocities : trajectory_velocities) {
                total_valid_points++;

                for (int j = 0; j < 8; ++j) {
                    double vel_abs = std::abs(velocities[j]);
                    velocity_sums[j] += vel_abs;
                    velocity_sq_sums[j] += vel_abs * vel_abs;

                    // 更新最大速度
                    metrics.max_joint_velocity[j] = std::max(metrics.max_joint_velocity[j], vel_abs);

                    // 检查速度限制违反
                    if (vel_abs > metrics.velocity_limits[j]) {
                        metrics.velocity_limit_violations[j]++;
                    }
                }
            }
        }

        // 计算平均速度和方差
        if (total_valid_points > 0) {
            for (int j = 0; j < 8; ++j) {
                metrics.avg_joint_velocity[j] = velocity_sums[j] / total_valid_points;

                if (total_valid_points > 1) {
                    double mean_sq = velocity_sq_sums[j] / total_valid_points;
                    double mean = metrics.avg_joint_velocity[j];
                    metrics.velocity_variance[j] = mean_sq - mean * mean;
                }
            }
        }

        // 计算速度急变率(相邻点速度差的平均值)
        calculateVelocityJerk(all_velocities, metrics);

        // 计算整体平滑度(方差的加权和，方差越小越平滑)
        double smoothness_score = 0.0;
        for (int j = 0; j < 8; ++j) {
            smoothness_score += params_.velo_weights[j] * metrics.velocity_variance[j];
        }
        metrics.overall_velocity_smoothness = smoothness_score;
    }

    // 计算速度急变率(加速度的近似)
    void calculateVelocityJerk(const std::vector<std::vector<std::array<double, 8>>>& all_velocities,
        PerformanceMetrics& metrics) {
        std::array<double, 8> total_jerk = {};
        int jerk_samples = 0;

        for (const auto& trajectory_velocities : all_velocities) {
            if (trajectory_velocities.size() < 2) continue;

            for (size_t i = 1; i < trajectory_velocities.size(); ++i) {
                for (int j = 0; j < 8; ++j) {
                    double velocity_change = std::abs(trajectory_velocities[i][j] -
                        trajectory_velocities[i - 1][j]);
                    total_jerk[j] += velocity_change;
                }
                jerk_samples++;
            }
        }

        if (jerk_samples > 0) {
            for (int j = 0; j < 8; ++j) {
                metrics.velocity_jerk[j] = total_jerk[j] / jerk_samples;
            }
        }
    }

public:

    // 导出多算法比较结果到CSV
    void exportMultiAlgorithmComparison(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "multi_algorithm_comparison.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建CSV文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // CSV标题行 - 使用逗号分隔
        file << "Algorithm,AT(ms),SR150(%),JointLimitCost,VelocitySmoothness,"
            << "SuccessPoints,TotalPoints,SuccessRate(%),TotalLimitApproaches,TotalVelocityViolations";

        // 每个关节的详细指标
        for (int i = 1; i <= 8; ++i) {
            file << ",M" << i << ",AvgVel" << i << ",MaxVel" << i << ",VelVar" << i
                << ",VelJerk" << i << ",TCL" << i << ",VelViolation" << i;
        }
        file << "\n";

        // 写入每个算法的数据
        for (const auto& metrics : results) {
            int total_approaches = std::accumulate(metrics.joint_limit_approaches.begin(),
                metrics.joint_limit_approaches.end(), 0);
            int total_violations = std::accumulate(metrics.velocity_limit_violations.begin(),
                metrics.velocity_limit_violations.end(), 0);
            double success_rate = static_cast<double>(metrics.successful_points) / metrics.total_points * 100.0;

            file << metrics.algorithm_name << ","
                << metrics.average_time << ","
                << metrics.success_rate_150ms << ","
                << metrics.avg_joint_limit_cost << ","
                << metrics.overall_velocity_smoothness << ","
                << metrics.successful_points << ","
                << metrics.total_points << ","
                << success_rate << ","
                << total_approaches << ","
                << total_violations;

            // 每个关节的详细数据
            for (int i = 0; i < 8; ++i) {
                file << "," << metrics.avg_joint_movement[i]
                    << "," << metrics.avg_joint_velocity[i]
                    << "," << metrics.max_joint_velocity[i]
                    << "," << metrics.velocity_variance[i]
                    << "," << metrics.velocity_jerk[i]
                    << "," << metrics.joint_limit_approaches[i]
                    << "," << metrics.velocity_limit_violations[i];
            }
            file << "\n";
        }

        std::cout << "多算法比较结果已导出到: " << filename << std::endl;
    }


    // 导出时间序列数据到CSV
    void exportTimeSeriesData(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "time_series_data.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建时间序列CSV文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // CSV标题行
        file << "Algorithm,PointIndex,ComputationTime(ms),JointLimitCost,Success";

        // 关节角度和速度
        for (int i = 1; i <= 8; ++i) {
            file << ",Joint" << i << ",Vel" << i;
        }
        file << "\n";

        // 写入每个算法的每个轨迹点数据
        for (const auto& metrics : results) {
            for (size_t i = 0; i < metrics.computation_times.size(); ++i) {
                file << metrics.algorithm_name << ","
                    << i << ","
                    << metrics.computation_times[i] << ","
                    << metrics.joint_limit_costs[i] << ","
                    << (metrics.optimization_success[i] ? "1" : "0");

                // 关节角度和速度
                for (int j = 0; j < 8; ++j) {
                    file << "," << metrics.joint_angles[i][j]
                        << "," << (i < metrics.joint_velocities.size() ? metrics.joint_velocities[i][j] : 0.0);
                }
                file << "\n";
            }
        }
    }


    // 导出统计汇总表到CSV
    void exportSummaryStatistics(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "summary_statistics.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建汇总统计CSV文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // 汇总表头
        file << "Metric";
        for (const auto& metrics : results) {
            file << "," << metrics.algorithm_name;
        }
        file << "\n";

        // 主要性能指标
        std::vector<std::pair<std::string, std::function<double(const PerformanceMetrics&)>>> metrics_list = {
            {"AverageTime(ms)", [](const auto& m) { return m.average_time; }},
            {"SuccessRate150(%)", [](const auto& m) { return m.success_rate_150ms; }},
            {"JointLimitCost", [](const auto& m) { return m.avg_joint_limit_cost; }},
            {"VelocitySmoothness", [](const auto& m) { return m.overall_velocity_smoothness; }},
            {"OverallSuccessRate(%)", [](const auto& m) {
                return static_cast<double>(m.successful_points) / m.total_points * 100.0;
            }},
            {"TotalLimitApproaches", [](const auto& m) {
                return std::accumulate(m.joint_limit_approaches.begin(), m.joint_limit_approaches.end(), 0);
            }},
            {"TotalVelocityViolations", [](const auto& m) {
                return std::accumulate(m.velocity_limit_violations.begin(), m.velocity_limit_violations.end(), 0);
            }}
        };

        for (const auto& [name, func] : metrics_list) {
            file << name;
            for (const auto& metrics : results) {
                file << "," << func(metrics);
            }
            file << "\n";
        }

        // 关节相关指标的汇总
        for (int i = 0; i < 8; ++i) {
            file << "Joint" << (i + 1) << "_AvgMovement";
            for (const auto& metrics : results) {
                file << "," << metrics.avg_joint_movement[i];
            }
            file << "\n";

            file << "Joint" << (i + 1) << "_AvgVelocity";
            for (const auto& metrics : results) {
                file << "," << metrics.avg_joint_velocity[i];
            }
            file << "\n";

            file << "Joint" << (i + 1) << "_LimitApproaches";
            for (const auto& metrics : results) {
                file << "," << metrics.joint_limit_approaches[i];
            }
            file << "\n";

            file << "Joint" << (i + 1) << "_VelocityViolations";
            for (const auto& metrics : results) {
                file << "," << metrics.velocity_limit_violations[i];
            }
            file << "\n";
        }
    }

    // 导出速度分析报告到CSV
    void exportVelocityAnalysis(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "velocity_analysis.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建速度分析CSV文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // 标题行
        file << "Algorithm,Joint,AvgVelocity,VelocityLimit,MaxVelocity,VelocityVariance,"
            << "VelocityJerk,ViolationCount,OverallSmoothness,TotalViolations\n";

        // 写入每个算法的每个关节数据
        for (const auto& metrics : results) {
            int total_violations = std::accumulate(metrics.velocity_limit_violations.begin(),
                metrics.velocity_limit_violations.end(), 0);

            for (int i = 0; i < 8; ++i) {
                file << metrics.algorithm_name << ","
                    << "Joint" << (i + 1) << ","
                    << metrics.avg_joint_velocity[i] << ","
                    << metrics.velocity_limits[i] << ","
                    << metrics.max_joint_velocity[i] << ","
                    << metrics.velocity_variance[i] << ","
                    << metrics.velocity_jerk[i] << ","
                    << metrics.velocity_limit_violations[i] << ","
                    << metrics.overall_velocity_smoothness << ","
                    << total_violations << "\n";
            }
        }

        std::cout << "速度分析已导出到: " << filename << std::endl;
    }

    // 导出详细分析报告到CSV
    void exportDetailedAnalysis(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "detailed_analysis.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建详细分析CSV文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // 第一部分：整体性能指标
        file << "Algorithm,AverageTime(ms),SR150(%),OverallSuccessRate(%),AvgJointLimitCost,"
            << "SuccessfulPoints,TotalPoints,TotalLimitApproaches,TotalVelocityViolations\n";

        for (const auto& metrics : results) {
            int total_approaches = std::accumulate(metrics.joint_limit_approaches.begin(),
                metrics.joint_limit_approaches.end(), 0);
            int total_violations = std::accumulate(metrics.velocity_limit_violations.begin(),
                metrics.velocity_limit_violations.end(), 0);
            double success_rate = static_cast<double>(metrics.successful_points) / metrics.total_points * 100.0;

            file << metrics.algorithm_name << ","
                << metrics.average_time << ","
                << metrics.success_rate_150ms << ","
                << success_rate << ","
                << metrics.avg_joint_limit_cost << ","
                << metrics.successful_points << ","
                << metrics.total_points << ","
                << total_approaches << ","
                << total_violations << "\n";
        }

        file << "\n";  // 空行分隔

        // 第二部分：各关节平均移动距离
        file << "Algorithm,Joint,AvgMovement(rad),AvgVelocity,MaxVelocity,LimitApproaches,VelocityViolations\n";

        for (const auto& metrics : results) {
            for (int i = 0; i < 8; ++i) {
                file << metrics.algorithm_name << ","
                    << "Joint" << (i + 1) << ","
                    << metrics.avg_joint_movement[i] << ","
                    << metrics.avg_joint_velocity[i] << ","
                    << metrics.max_joint_velocity[i] << ","
                    << metrics.joint_limit_approaches[i] << ","
                    << metrics.velocity_limit_violations[i] << "\n";
            }
        }

        std::cout << "详细分析已导出到: " << filename << std::endl;
    }

    // 可选：导出算法对比汇总到CSV（更简洁的版本）
    void exportAlgorithmComparison(const std::vector<PerformanceMetrics>& results,
        const std::string& filename = "algorithm_comparison.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建算法对比CSV文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // 标题行
        file << "Algorithm,AvgTime(ms),SR150(%),SuccessRate(%),JointLimitCost,VelocitySmoothness,"
            << "TotalViolations,ViolationRate(%)\n";

        for (const auto& metrics : results) {
            int total_violations = std::accumulate(metrics.velocity_limit_violations.begin(),
                metrics.velocity_limit_violations.end(), 0);
            double success_rate = static_cast<double>(metrics.successful_points) / metrics.total_points * 100.0;
            double violation_rate = static_cast<double>(total_violations) / metrics.total_points * 100.0;

            file << metrics.algorithm_name << ","
                << metrics.average_time << ","
                << metrics.success_rate_150ms << ","
                << success_rate << ","
                << metrics.avg_joint_limit_cost << ","
                << metrics.overall_velocity_smoothness << ","
                << total_violations << ","
                << violation_rate << "\n";
        }

        std::cout << "算法对比已导出到: " << filename << std::endl;
    }

    // 完整的分析导出函数（全部CSV格式）
    void exportCompleteAnalysis(const std::vector<PerformanceMetrics>& results,
        const std::string& base_filename = "algorithm_analysis") {

        std::cout << "\n=== 开始导出多算法分析数据 ===" << std::endl;

        try {
            // 1. CSV格式的主要比较结果
            exportMultiAlgorithmComparison(results, base_filename + "_main.csv");

            // 2. CSV格式的汇总统计
            exportSummaryStatistics(results, base_filename + "_summary.csv");

            // 3. CSV格式的时间序列数据
            if (!results.empty() && !results[0].computation_times.empty()) {
                exportTimeSeriesData(results, base_filename + "_timeseries.csv");
            }

            // 4. CSV格式的详细分析报告
            exportDetailedAnalysis(results, base_filename + "_detailed.csv");

            // 5. CSV格式的速度分析报告
            exportVelocityAnalysis(results, base_filename + "_velocity.csv");

            // 6. CSV格式的简洁算法对比
            exportAlgorithmComparison(results, base_filename + "_comparison.csv");

            std::cout << "=== 多算法分析数据导出完成 ===" << std::endl;
            std::cout << "生成的文件:" << std::endl;
            std::cout << "  - " << base_filename << "_main.csv (完整主要结果)" << std::endl;
            std::cout << "  - " << base_filename << "_summary.csv (汇总统计)" << std::endl;
            std::cout << "  - " << base_filename << "_timeseries.csv (时间序列数据)" << std::endl;
            std::cout << "  - " << base_filename << "_detailed.csv (详细分析)" << std::endl;
            std::cout << "  - " << base_filename << "_velocity.csv (速度分析)" << std::endl;
            std::cout << "  - " << base_filename << "_comparison.csv (算法对比)" << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "导出过程中出现错误: " << e.what() << std::endl;
        }
    }
    public:
        // 获取当前设置的算法类型
        const std::vector<std::string>& getAlgorithmTypes() const {
            return algorithm_types_;
        }

private:
    void exportJointTrajectoryWithCosts(const std::vector<std::array<double, 8>>& trajectory,
        const std::vector<double>& limit_costs,
        const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法创建关节轨迹文件: " + filename);
        }

        file << std::fixed << std::setprecision(6);

        // 写入标题行
        file << "Index,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7,Joint8,LimitCost\n";

        // 写入数据
        for (size_t i = 0; i < trajectory.size(); ++i) {
            file << i;
            for (const auto& joint : trajectory[i]) {
                file << "," << joint;
            }
            file << "," << (i < limit_costs.size() ? limit_costs[i] : 0.0);
            file << "\n";
        }
    }

};

// 使用示例
class Example {
public:
    static void demonstrateComparison() {
        RobotArm arm;
        CollisionSystem collision_system(arm);
        OptimizeParams params;

        AlgorithmComparator comparator(arm, collision_system, params);

        // 设置要比较的多个算法
        //comparator.setAlgorithmTypes({ "GA", "ISGA", "DE", "MOEAD", "NSGA" });
        //comparator.setAlgorithmTypes({ "GA", "ISGA",  "MOEAD", "NSGA" });
        comparator.setAlgorithmTypes({ "GA"});


        std::cout << "开始比较 " << comparator.getAlgorithmTypes().size() << " 个算法..." << std::endl;

        try {
            // 执行多算法比较
            //auto results = comparator.compareAlgorithms("2025-09-09-15-02-upperpoint.csv");
            auto results = comparator.compareAlgorithms("curve.csv");

            std::cout << "\n=== 多算法比较完成 ===" << std::endl;
            std::cout << "比较的算法: ";
            for (const auto& result : results) {
                std::cout << result.algorithm_name << " ";
            }
            std::cout << std::endl;

            // 导出完整的分析数据
            //comparator.exportCompleteAnalysis(results, "multi_algorithm_comparison");
            comparator.exportCompleteAnalysis(results, "curve");

            // 打印简要汇总
            std::cout << "\n=== 算法性能排名 ===" << std::endl;
            std::vector<std::pair<std::string, double>> ranking;
            for (const auto& result : results) {
                // 综合评分（时间权重0.4，成功率权重0.3，平滑度权重0.2，限位成本权重0.1）
                double score = (1.0 / result.average_time) * 0.4 +
                    result.success_rate_150ms * 0.3 +
                    (1.0 / (result.overall_velocity_smoothness + 1e-6)) * 0.2 +
                    (1.0 / (result.avg_joint_limit_cost + 1e-6)) * 0.1;
                ranking.emplace_back(result.algorithm_name, score);
            }

            // 按评分排序
            std::sort(ranking.begin(), ranking.end(),
                [](const auto& a, const auto& b) { return a.second > b.second; });

            for (size_t i = 0; i < ranking.size(); ++i) {
                std::cout << i + 1 << ". " << ranking[i].first
                    << " (评分: " << ranking[i].second << ")" << std::endl;
            }

        }
        catch (const std::exception& e) {
            std::cerr << "多算法比较过程中出现错误: " << e.what() << std::endl;
        }
    }
};