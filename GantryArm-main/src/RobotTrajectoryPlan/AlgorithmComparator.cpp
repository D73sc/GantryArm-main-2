#include "AlgorithmComparator.h"
#include <iostream>
#include <iomanip>

AlgorithmComparator::AlgorithmComparator(RobotArm& arm, CollisionSystem& collision_system,
    const OptimizeParams& params)
    : arm_(arm), collision_system_(collision_system), params_(params),
    joint_limits_(arm_.getJointLimits()) {
    // 默认比较算法列表
    algorithm_types_ = { "GA", "NSGA" };
}

void AlgorithmComparator::setAlgorithmTypes(const std::vector<std::string>& types) {
    algorithm_types_ = types;
}

const std::vector<std::string>& AlgorithmComparator::getAlgorithmTypes() const {
    return algorithm_types_;
}

std::vector<AlgorithmComparator::PerformanceMetrics>
AlgorithmComparator::compareAlgorithms(const std::string& trajectory_file, bool isMm) {
    if (isMm) {
        auto trajectory = loadTrajectoryFromCSV(trajectory_file,true);
        return compareAlgorithms(trajectory);
    }
    else {
        auto trajectory = loadTrajectoryFromCSV(trajectory_file);
        return compareAlgorithms(trajectory);
    }
}

// 直接评估关节角序列（无时间统计）
AlgorithmComparator::PerformanceMetrics
AlgorithmComparator::evaluateJointTrajectory(
    const std::string& alg_type,
    const std::vector<std::array<double, 8>>& joint_trajectory)
{
    PerformanceMetrics metrics;
    metrics.algorithm_name = alg_type;
    metrics.total_points = joint_trajectory.size();

    // 预分配内存
    metrics.joint_angles.reserve(joint_trajectory.size());
    metrics.joint_velocities.reserve(joint_trajectory.size());
    metrics.optimization_success.reserve(joint_trajectory.size());

    // 初始化
    std::array<double, 8> prev_joints = { 0, 0, 0, 0, 0, 0, 0, 0 };
    std::array<double, 8> total_movements = { 0, 0, 0, 0, 0, 0, 0, 0 };

    for (size_t i = 0; i < joint_trajectory.size(); ++i) {
        const auto& current_joints = joint_trajectory[i];

        // 进度显示
        if ((i + 1) % 100 == 0 || i == 0) {
            std::cout << "  " << alg_type << " - 进度: " << (i + 1) << "/"
                << joint_trajectory.size() << std::endl;
        }

        try {
            // 计算关节运动量
            double total_movement = 0.0;
            for (int j = 0; j < 8; ++j) {
                double movement = std::abs(current_joints[j] - prev_joints[j]);
                total_movement += movement;
                total_movements[j] += movement;
            }

            // 判断是否为有效运动点
            if (total_movement > 1e-10) {
                metrics.successful_points++;

                // 计算关节速度
                auto velocities = calcJointVelocities(current_joints, prev_joints);

                // 检查关节极限接近度
                checkJointLimitApproaches(current_joints, metrics);

                // 记录指标
                metrics.joint_angles.push_back(current_joints);
                metrics.joint_velocities.push_back(velocities);
                metrics.optimization_success.push_back(true);
            }
            else {
                // 静止点处理
                metrics.joint_angles.push_back(prev_joints);
                metrics.joint_velocities.push_back({});
                metrics.optimization_success.push_back(false);
            }

            prev_joints = current_joints;

        }
        catch (const std::exception& e) {
            std::cerr << alg_type << " 评估失败在点 " << i << ": " << e.what() << std::endl;

            metrics.joint_angles.push_back(prev_joints);
            metrics.joint_velocities.push_back({});
            metrics.optimization_success.push_back(false);
        }
    }

    // 统计平均关节运动量
    if (metrics.successful_points > 0) {
        for (int j = 0; j < 8; ++j) {
            metrics.avg_joint_movement[j] = total_movements[j] / metrics.successful_points;
        }
    }
    printMetricsSummary(metrics);

    //exportMetricsToCSV(metrics, alg_type+".csv");
    return metrics;
}

AlgorithmComparator::PerformanceMetrics AlgorithmComparator::evaluateJointTrajectoryWithTimes(const std::string& alg_type, const std::vector<std::array<double, 8>>& joint_trajectory, const std::vector<double>& elapsed_times_ms)
{
    auto metrics = evaluateJointTrajectory(alg_type, joint_trajectory);


    // 判断elapsed_times_ms大小是否和轨迹点数匹配
    size_t N = elapsed_times_ms.size();
    if (N != metrics.total_points) {
        std::cerr << "警告：时间数组长度与轨迹点数不匹配！" << std::endl;
    }

    // 统计小于阈值的点数
    int points_within_5ms = 0;
    int points_within_10ms = 0;
    double sum_time = 0.0;

    for (size_t i = 0; i < N; ++i) {
        double t = elapsed_times_ms[i];
        sum_time += t;
        if (t <= 5.0) ++points_within_5ms;
        if (t <= 10.0) ++points_within_10ms;
    }

    double avg_time = (N > 0) ? (sum_time / N) : 0.0;

    // 写入metrics
    metrics.avg_time_ms = avg_time;
    metrics.success_rate_5ms = (N > 0) ? (100.0 * points_within_5ms / N) : 0.0;
    metrics.success_rate_10ms = (N > 0) ? (100.0 * points_within_10ms / N) : 0.0;


    return metrics;
}

// ========== CSV导出函数 ==========
void AlgorithmComparator::exportMetricsToCSV(PerformanceMetrics metrics,
    const std::string& filename = "algorithm_comparison.csv")
{
    std::ofstream csv_file(filename);

    if (!csv_file.is_open()) {
        std::cerr << "无法创建CSV文件: " << filename << std::endl;
        return;
    }

    // 写入表头
    csv_file << "算法名称,AT(ms),SR5(%),SR10(%),";
    csv_file << "J1(mm),J2(mm),J3(mm),J4(rad),";
    csv_file << "J5(rad),J6(rad),J7(rad),J8(rad),";
    csv_file << "关节限位次数,关节突变次数,速度突变次数\n";

    // 写入数据
        csv_file << std::fixed << std::setprecision(4);

        csv_file << metrics.algorithm_name << ",";
        csv_file << metrics.average_time << ",";
        csv_file << metrics.success_rate_5ms << ",";
        csv_file << metrics.success_rate_10ms << ",";

        for (int i = 0; i < 8; ++i) {
            csv_file << metrics.avg_joint_movement[i];
            if (i < 7) csv_file << ",";
        }
        csv_file << ",";
        int total_approaches = 0;
        for (int i = 0; i < 8; ++i) {
            total_approaches += metrics.joint_limit_approaches[i];
        }

        std::array<int, 8> joint_sudden_changes_per_joint = {};
        int total_joint_changes = calculateJointSuddenChangesDetailed(metrics, joint_sudden_changes_per_joint);

        std::array<int, 8> velocity_sudden_changes_per_joint = {};
        int total_velocity_changes = calculateVelocitySuddenChangesDetailed(metrics, velocity_sudden_changes_per_joint);

        csv_file << total_approaches << ",";
        csv_file << total_joint_changes << ",";
        csv_file << total_velocity_changes << "\n";
    

    csv_file.close();
    std::cout << "CSV文件已保存: " << filename << std::endl;
}

// ========== CSV导出函数 ==========
void AlgorithmComparator::exportMetricsToCSV(std::vector<PerformanceMetrics> all_metrics,
    const std::string& filename = "algorithm_comparison.csv")
{
    std::ofstream csv_file(filename);

    if (!csv_file.is_open()) {
        std::cerr << "无法创建CSV文件: " << filename << std::endl;
        return;
    }

    // 写入表头
    csv_file << "算法名称,AT(ms),SR5(%),SR10(%),";
    csv_file << "J1(mm),J2(mm),J3(mm),J4(°),";
    csv_file << "J5(°),J6(°),J7(°),J8(°),";
    csv_file << "关节限位次数,关节突变次数,速度突变次数,\n";
    for (auto metrics : all_metrics)
    {
        // 写入数据
        csv_file << std::fixed << std::setprecision(4);

        csv_file << metrics.algorithm_name << ",";
        csv_file << metrics.avg_time_ms << ",";
        csv_file << metrics.success_rate_5ms << ",";
        csv_file << metrics.success_rate_10ms << ",";

        for (int i = 0; i < 8; ++i) {
            if (i < 3)
                csv_file << metrics.avg_joint_movement[i];
            else
                csv_file << metrics.avg_joint_movement[i] * 180 / M_PI;            
            if (i < 7) csv_file << ",";
        }
        csv_file << ",";
        int total_approaches = 0;
        for (int i = 0; i < 8; ++i) {
            total_approaches += metrics.joint_limit_approaches[i];
        }

        std::array<int, 8> joint_sudden_changes_per_joint = {};
        int total_joint_changes = calculateJointSuddenChangesDetailed(metrics, joint_sudden_changes_per_joint);

        std::array<int, 8> velocity_sudden_changes_per_joint = {};
        int total_velocity_changes = calculateVelocitySuddenChangesDetailed(metrics, velocity_sudden_changes_per_joint);

        csv_file << total_approaches << ",";
        csv_file << total_joint_changes << ",";
        csv_file << total_velocity_changes << "\n";
    }

    csv_file.close();
    std::cout << "CSV文件已保存: " << filename << std::endl;
}


std::vector<AlgorithmComparator::PerformanceMetrics>
AlgorithmComparator::compareAlgorithms(const std::vector<Eigen::Matrix4d>& trajectory) {
    std::vector<PerformanceMetrics> results;

    std::cout << "开始算法比较,轨迹点数: " << trajectory.size() << std::endl;

    for (const std::string& alg_type : algorithm_types_) {
        std::cout << "\n测试算法: " << alg_type << std::endl;

        auto metrics = evaluateAlgorithm(alg_type, arm_.tcpToFlange( trajectory));
        results.push_back(metrics);

        std::cout << "算法 " << alg_type << " 完成" << std::endl;
        TrajectoryOptimizer::exportJointTrajectory(metrics.joint_angles, alg_type+".csv");
        printMetricsSummary(metrics);
    }

    return results;
}

double AlgorithmComparator::calcJointLimitCost(const std::array<double, 8>& q) const {
    double cost = 0.0;

    for (int i = 0; i < 8; i++) {
        double safe_range = 0.2 * (joint_limits_.max[i] - joint_limits_.min[i]);
        cost += params_.joint_limits_weights[i] *
            (2.0 / (1.0 + exp((q[i] - joint_limits_.min[i]) / safe_range)) +
                2.0 / (1.0 + exp(-(q[i] - joint_limits_.max[i]) / safe_range)));
    }

    return cost;
}

double AlgorithmComparator::calcSingleJointLimitCost(double joint_angle, int joint_index,
    const PerformanceMetrics& metrics) const {

    double safe_margin = metrics.limit_cost_thresholds[joint_index];
    double safe_min, safe_max;

    if (joint_index < 3) {
        safe_min = joint_limits_.min[joint_index] + safe_margin;
        safe_max = joint_limits_.max[joint_index] - safe_margin;
    }
    else {
        safe_min = joint_limits_.min[joint_index] + safe_margin;
        safe_max = joint_limits_.max[joint_index] - safe_margin;
    }

    if (joint_angle >= safe_min && joint_angle <= safe_max) {
        return 0.0;
    }

    double cost = 0.0;
    if (joint_angle < safe_min) {
        cost = safe_min - joint_angle;
    }
    else if (joint_angle > safe_max) {
        cost = joint_angle - safe_max;
    }

    return cost;
}

AlgorithmComparator::PerformanceMetrics
AlgorithmComparator::evaluateAlgorithm(const std::string& alg_type,
    const std::vector<Eigen::Matrix4d>& trajectory) {

    PerformanceMetrics metrics;
    metrics.algorithm_name = alg_type;
    metrics.total_points = trajectory.size();

    metrics.computation_times.reserve(trajectory.size());
    metrics.joint_angles.reserve(trajectory.size());
    metrics.joint_velocities.reserve(trajectory.size());
    metrics.optimization_success.reserve(trajectory.size());

    auto optimizer = TrajectoryOptimizer::create(alg_type, arm_, collision_system_, params_);
    if (!optimizer) {
        throw std::runtime_error("无法创建优化器: " + alg_type);
    }

    Eigen::Matrix4d prev_target_pose = Eigen::Matrix4d::Identity();
    std::array<double, 8> prev_joints = { 0	,0	,0,	0,	0,	0,	0,	0};
    RobotArm::Solution prev_solution;
    std::vector<TrajectoryOptimizer::Individual> prev_population;

    std::vector<double> computation_times;
    std::array<double, 8> total_movements = {};
    int points_within_5ms = 0;
    int points_within_10ms = 0;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto target_pose = trajectory[i];

        if ((i + 1) % 100 == 0 || i == 0) {
            std::cout << "  " << alg_type << " - 进度: " << (i + 1) << "/"
                << trajectory.size() << std::endl;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            auto optimal_joints = optimizer->optimize(target_pose, prev_target_pose,
                prev_solution, prev_population);

            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(
                end_time - start_time).count();

            double total_movement = 0.0;
            for (int j = 0; j < 8; ++j) {
                double movement = std::abs(optimal_joints[j] - prev_joints[j]);
                total_movement += movement;
                total_movements[j] += movement;
            }

            if (total_movement > 1e-10) {
                computation_times.push_back(elapsed_ms);
                metrics.successful_points++;

                if (elapsed_ms <= 5.0) {
                    points_within_5ms++;
                }
                if (elapsed_ms <= 10.0) {
                    points_within_10ms++;
                }
                auto velocities = calcJointVelocities(optimal_joints, prev_joints);
                checkJointLimitApproaches(optimal_joints, metrics);

                metrics.computation_times.push_back(elapsed_ms);
                metrics.joint_angles.push_back(optimal_joints);
                metrics.joint_velocities.push_back(velocities);
                metrics.optimization_success.push_back(true);
            }
            else {
                metrics.computation_times.push_back(0);
                metrics.joint_angles.push_back(prev_joints);
                metrics.joint_velocities.push_back({});
                metrics.optimization_success.push_back(false);
            }

            prev_joints = optimal_joints;
            prev_target_pose = target_pose;

        }
        catch (const std::exception& e) {
            std::cerr << alg_type << " 优化失败在点 " << i << ": " << e.what() << std::endl;

            metrics.computation_times.push_back(1000.0);
            metrics.joint_angles.push_back(prev_joints);
            metrics.joint_velocities.push_back({});
            metrics.optimization_success.push_back(false);

            computation_times.push_back(1000.0);
        }
    }

    if (!computation_times.empty()) {
        metrics.average_time = std::accumulate(computation_times.begin(),
            computation_times.end(), 0.0) / computation_times.size();
    }

    metrics.success_rate_5ms = (metrics.total_points > 0) ?
        (static_cast<double>(points_within_5ms) / metrics.total_points * 100.0) : 0.0;
    metrics.success_rate_10ms = (metrics.total_points > 0) ?
        (static_cast<double>(points_within_10ms) / metrics.total_points * 100.0) : 0.0;
    if (metrics.successful_points > 0) {
        for (int j = 0; j < 8; ++j) {
            metrics.avg_joint_movement[j] = total_movements[j] / metrics.successful_points;
        }
    }

    return metrics;
}

void AlgorithmComparator::checkJointLimitApproaches(const std::array<double, 8>& joints,
    PerformanceMetrics& metrics) {
    for (int i = 0; i < 8; ++i) {
        double joint_cost = calcSingleJointLimitCost(joints[i], i, metrics);
        if (joint_cost > 0) {
            metrics.joint_limit_approaches[i]++;
        }
    }
}

std::array<double, 8> AlgorithmComparator::calcJointVelocities(
    const std::array<double, 8>& joints,
    const std::array<double, 8>& ref_joints) const {

    std::array<double, 8> velocities = {};

    double sum_sq = 0.0;
    for (int i = 0; i < 3; i++) {
        sum_sq += pow((joints[i] - ref_joints[i]) / 10.0, 2);
    }
    for (int i = 3; i < 8; i++) {
        sum_sq += pow(joints[i] - ref_joints[i], 2);
    }

    double T = sqrt(sum_sq) / params_.velocities;

    if (T > 1e-6) {
        for (int i = 0; i < 3; i++) {
            velocities[i] = (joints[i] - ref_joints[i]) / 10.0 / T;
        }
        for (int i = 3; i < 8; i++) {
            velocities[i] = (joints[i] - ref_joints[i]) / T;
        }
    }

    return velocities;
}

int AlgorithmComparator::calculateJointSuddenChanges(const PerformanceMetrics& metrics) const {
    int sudden_changes = 0;
    for (size_t i = 1; i < metrics.joint_angles.size(); ++i) {
        if (!metrics.optimization_success[i] || !metrics.optimization_success[i - 1]) {
            continue;
        }

        for (int j = 0; j < 8; ++j) {
            double change = std::abs(metrics.joint_angles[i][j] -
                metrics.joint_angles[i - 1][j]);
            if (change > thresholds[j]) {
                sudden_changes++;
                break;
            }
        }
    }
    return sudden_changes;
}

int AlgorithmComparator::calculateVelocitySuddenChanges(const PerformanceMetrics& metrics) const {
    int sudden_changes = 0;

    for (size_t i = 1; i < metrics.joint_velocities.size(); ++i) {
        if (!metrics.optimization_success[i] || !metrics.optimization_success[i - 1]) {
            continue;
        }

        for (int j = 0; j < 8; ++j) {
            double vel_change = std::abs(metrics.joint_velocities[i][j] -
                metrics.joint_velocities[i - 1][j]);
            if (vel_change > accel_thresholds[j]) {
                sudden_changes++;
                break;
            }
        }
    }

    return sudden_changes;
}

void AlgorithmComparator::exportSimplifiedComparison(
    const std::vector<PerformanceMetrics>& results,
    const std::string& filename) {

    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法创建CSV文件: " + filename);
    }

    file << std::fixed << std::setprecision(3);

    // 标题行
    file << "算法名称,AT(ms),SR5(%),SR10(%)";
    for (int i = 1; i <= 3; ++i) {
        file << ",J" << i << "(mm)";
    }
    for (int i = 4; i <= 8; ++i) {
        file << ",J" << i << "(°)";
    }
    file << ",极限逼近次数,关节突变次数,速度突变次数\n";

    // 数据行
    for (const auto& metrics : results) {
        file << metrics.algorithm_name << ","
            << metrics.average_time << ","
            << metrics.success_rate_5ms << ","
            << metrics.success_rate_10ms << ",";

        // 前3个关节(mm)
        for (int i = 0; i < 3; ++i) {
            file << metrics.avg_joint_movement[i];
            if (i < 2) file << ",";
        }
        file << ",";

        // 后5个关节(度)
        for (int i = 3; i < 8; ++i) {
            file << (metrics.avg_joint_movement[i] * 180.0 / M_PI);
            if (i < 7) file << ",";
        }
        file << ",";

        // 极限逼近次数
        int total_limit_approaches = std::accumulate(
            metrics.joint_limit_approaches.begin(),
            metrics.joint_limit_approaches.end(), 0);
        file << total_limit_approaches << ",";

        // 关节突变和速度突变
        int joint_sudden_changes = calculateJointSuddenChanges(metrics);
        int velocity_sudden_changes = calculateVelocitySuddenChanges(metrics);

        file << joint_sudden_changes << ","
            << velocity_sudden_changes << "\n";
    }

    std::cout << "简化比较表格已导出到: " << filename << std::endl;
}

void AlgorithmComparator::printMetricsSummary(const PerformanceMetrics& metrics) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  平均时间(AT): " << metrics.average_time << "ms" << std::endl;
    std::cout << "  5ms成功率(SR5): " << metrics.success_rate_5ms << "%" << std::endl;
    std::cout << "  10ms成功率(SR10): " << metrics.success_rate_10ms << "%" << std::endl;
    std::cout << "  成功点数: " << metrics.successful_points << "/"
        << metrics.total_points << std::endl;

    std::cout << "  关节平均移动: ";
    for (int i = 0; i < 3; ++i) {
        std::cout << "J" << (i + 1) << ":" << metrics.avg_joint_movement[i] << "mm ";
    }
    for (int i = 3; i < 8; ++i) {
        std::cout << "J" << (i + 1) << ":"
            << (metrics.avg_joint_movement[i] * 180.0 / M_PI) << "° ";
    }
    std::cout << std::endl;

    // 详细的极限逼近统计
    std::cout << "  极限逼近次数详情:" << std::endl;
    int total_approaches = 0;
    for (int i = 0; i < 8; ++i) {
        if (metrics.joint_limit_approaches[i] > 0) {
            std::cout << "    J" << (i + 1) << ": "
                << metrics.joint_limit_approaches[i] << "次";
            if (i < 3) {
                std::cout << " (mm)";
            }
            else {
                std::cout << " (°)";
            }
            std::cout << std::endl;
        }
        total_approaches += metrics.joint_limit_approaches[i];
    }
    std::cout << "    总计: " << total_approaches << "次" << std::endl;

    // 详细的关节突变统计
    std::cout << "  关节突变次数详情:" << std::endl;
    std::array<int, 8> joint_sudden_changes_per_joint = {};
    int total_joint_changes = calculateJointSuddenChangesDetailed(metrics, joint_sudden_changes_per_joint);

    for (int i = 0; i < 8; ++i) {
        if (joint_sudden_changes_per_joint[i] > 0) {
            std::cout << "    J" << (i + 1) << ": "
                << joint_sudden_changes_per_joint[i] << "次";
            if (i < 3) {
                std::cout << " (阈值:10mm)";
            }
            else {
                std::cout << " (阈值:5°)";
            }
            std::cout << std::endl;
        }
    }
    std::cout << "    总计: " << total_joint_changes << "次" << std::endl;

    // 详细的速度突变统计
    std::cout << "  速度突变次数详情:" << std::endl;
    std::array<int, 8> velocity_sudden_changes_per_joint = {};
    int total_velocity_changes = calculateVelocitySuddenChangesDetailed(metrics, velocity_sudden_changes_per_joint);

    for (int i = 0; i < 8; ++i) {
        if (velocity_sudden_changes_per_joint[i] > 0) {
            std::cout << "    J" << (i + 1) << ": "
                << velocity_sudden_changes_per_joint[i] << "次";
            if (i < 3) {
                std::cout << " (阈值:100mm/s²)";
            }
            else {
                std::cout << " (阈值:10°/s²)";
            }
            std::cout << std::endl;
        }
    }
    std::cout << "    总计: " << total_velocity_changes << "次" << std::endl;
}

int AlgorithmComparator::calculateJointSuddenChangesDetailed(
    const PerformanceMetrics& metrics,
    std::array<int, 8>& per_joint_counts) const {

    int sudden_changes = 0;
    per_joint_counts.fill(0);

    for (size_t i = 1; i < metrics.joint_angles.size(); ++i) {
        if (!metrics.optimization_success[i] || !metrics.optimization_success[i - 1]) {
            continue;
        }

        bool has_sudden_change = false;
        for (int j = 0; j < 8; ++j) {
            double change = std::abs(metrics.joint_angles[i][j] -
                metrics.joint_angles[i - 1][j]);
            if (change > thresholds[j]) {
                per_joint_counts[j]++;
                has_sudden_change = true;
            }
        }

        if (has_sudden_change) {
            sudden_changes++;
        }
    }

    return sudden_changes;
}

int AlgorithmComparator::calculateVelocitySuddenChangesDetailed(
    const PerformanceMetrics& metrics,
    std::array<int, 8>& per_joint_counts) const {

    int sudden_changes = 0;
    per_joint_counts.fill(0);

    for (size_t i = 1; i < metrics.joint_velocities.size(); ++i) {
        if (!metrics.optimization_success[i] || !metrics.optimization_success[i - 1]) {
            continue;
        }

        bool has_sudden_change = false;
        for (int j = 0; j < 8; ++j) {
            double vel_change = std::abs(metrics.joint_velocities[i][j] -
                metrics.joint_velocities[i - 1][j]);
            if (vel_change > accel_thresholds[j]) {
                per_joint_counts[j]++;
                has_sudden_change = true;
            }
        }

        if (has_sudden_change) {
            sudden_changes++;
        }
    }

    return sudden_changes;
}