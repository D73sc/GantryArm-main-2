#include "TrajectoryOptimizer.h"
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace std;

std::array<double, 8> TrajectoryOptimizer::optimize(const Eigen::Matrix4d& target_pose, Eigen::Matrix4d& prev_target_pose, RobotArm::Solution& prev_solution, std::vector<Individual>& prev_population)
{

    if (!prev_solution.valid) {
        prev_solution.joints = { 0, 0, 0, 0, 0, 0, 0, 0 };
        prev_solution.velocities = { 0, 0, 0, 0, 0, 0, 0, 0 };
    }
    const RobotArm::CoreParams cp = arm_.extractCoreParams(target_pose);

    RobotArm::Solution solution = prev_solution;
    updateGeneJoints(cp);

    // 确定搜索半径
    current_search_radius_ = determineSearchRadius(target_pose, prev_solution.joints, search_radius_);

    // 初始化种群
    std::vector<Individual> population;

    // 处理种群迁移
    if (!prev_population.empty()) {
        population = handleMigration(prev_population, target_pose, prev_target_pose);
    }

    // 生成剩余个体
    while ((int)population.size() < params_.population_size) {
        Individual ind = generateIndividualWithRadius(prev_solution.joints, current_search_radius_);
        population.push_back(ind);
    }

    Individual best_individual;
    best_individual.valid = false;
    best_individual.fitness = INFINITY;
    //double prev_best_fitness = INFINITY;
    //int stagnation_count = 0;

    // 主循环
    for (int generation = 0; generation < params_.max_generations; ++generation) {
        double best_fitness = evaluatePopulation(population, prev_solution, target_pose, gene_joint1_, gene_joint2_);


        // 获取当前最佳个体
        auto current_best = getBestValidIndividual(population);
        if (current_best.valid && current_best.fitness < best_individual.fitness) {
            //cout << "gen:" << generation << " best_fitness:" << best_fitness<<" continuityCost:"<< current_best.continuityCost<<" jointCost:" << current_best.jointCost << endl;

            best_individual = current_best;
            //更新记录用于实验
            last_best_fitness_ = current_best.fitness;

        }

        // 检查收敛条件
        if (best_fitness < 0.25) {
            if (best_individual.valid) {
                solution.joints = best_individual.joints;
                calcVelocities(solution, prev_solution.joints);
                prev_solution = solution;
                prev_target_pose = target_pose;
                prev_population = population;
                break;
            }
        }

        //// 新增：早期退出条件
        //if (generation >= params_.max_generations / 2) {
        //    // 检查适应度是否停滞
        //    if (std::abs(best_fitness - prev_best_fitness) < 0.1) {
        //        stagnation_count++;
        //    }
        //    else {
        //        stagnation_count = 0; // 重置计数器
        //    }

        //    // 如果连续多次停滞，直接退出
        //    if (stagnation_count >= params_.max_generations/10) {
        //        //cout << "Early termination at generation " << generation << " due to stagnation" << endl;
        //        if (best_individual.valid) {
        //            solution.joints = best_individual.joints;
        //            calcVelocities(solution, prev_solution.joints);
        //            prev_solution = solution;
        //            prev_target_pose = target_pose;
        //            prev_population = population;
        //        }
        //        break;
        //    }
        //}
        //prev_best_fitness = best_fitness;

        // 选择
        auto selected = select(population);
        if (selected.empty()) {
            // 重新初始化种群
            population.clear();
            while ((int)population.size() < params_.population_size) {
                Individual ind = generateIndividualWithRadius(prev_solution.joints, current_search_radius_);
                population.push_back(ind);
            }
            continue;
        }

        // 繁殖
        population = reproduction(selected);
    }

    // 最终评估
    evaluatePopulation(population, prev_solution, target_pose, gene_joint1_, gene_joint2_);
    auto final_best = getBestValidIndividual(population);

    if (final_best.valid) {
        solution.joints = final_best.joints;
        solution.valid = true;
        calcVelocities(solution, prev_solution.joints);
        prev_solution = solution;
        prev_target_pose = target_pose;
        prev_population = population;
        //更新记录用于实验
        last_best_fitness_ = final_best.fitness;
        return solution.joints;
    }

    return prev_solution.joints;
}

// 工厂方法实现
std::unique_ptr<TrajectoryOptimizer> TrajectoryOptimizer::create(
    const std::string& type,
    RobotArm& arm,
    CollisionSystem& collision_system,
    const OptimizeParams& params) {

    if (type == "GA" || type == "genetic") {
        return std::make_unique<GeneticOptimizer>(arm, collision_system, params);
    }
    else if (type == "NSGA") {
        return std::make_unique<NSGAOptimizer>(arm, collision_system, params);
    }
    else if (type == "MOEAD" || type == "moea/d") {
        return std::make_unique<MOEADOptimizer>(arm, collision_system, params);
    }
    else if (type == "ISGA" || type == "isga-bt") {
        return std::make_unique<ISGAOptimizer>(arm, collision_system, params);
    }
    else if (type == "DE" || type == "differential") {
        return std::make_unique<DifferentialOptimizer>(arm, collision_system, params);
    }

    throw std::invalid_argument("Unknown optimizer type: " + type);
}

std::array<double, 8> TrajectoryOptimizer::optimizeSinglePoint(const Eigen::Matrix4d& target_pose)
{
    RobotArm::Solution prev_solution;
    Eigen::Matrix4d prev_pose;
    std::vector<Individual> prev_population;
    if (!warmupSolution.valid) {
        prev_solution.joints.fill(0);
        prev_solution.velocities.fill(0);
        prev_pose = Eigen::Matrix4d::Identity();
    }
    else
    {
        prev_solution = warmupSolution;
        prev_population = warmupPopulation_;
        prev_pose = warmupTargetPoses_;
    }
    auto joints = optimize(arm_.tcpToFlange(target_pose), prev_pose, prev_solution, prev_population);

    // 更新缓存，支持下次热启动
    warmupTargetPoses_ =target_pose;
    warmupPopulation_ = prev_population;
    warmupSolution = prev_solution;
    return joints;
}

std::vector<std::array<double, 8>> TrajectoryOptimizer::optimizeTrajectory(const std::vector<Eigen::Matrix4d>& trajectory, std::function<void(const std::string&)> progress_callback)
{

    return optimizeTrajectoryInternal(arm_.tcpToFlange(trajectory), nullptr, progress_callback);
}

AsyncOptimizeTask TrajectoryOptimizer::optimizeTrajectoryAsync(const std::vector<Eigen::Matrix4d>& trajectory, std::function<void(const std::string&)> progress_callback)
{

    auto cancel_flag = std::make_shared<std::atomic<bool>>(false);

    auto future = std::async(std::launch::async, [=]() mutable {
        return optimizeTrajectoryInternal(arm_.tcpToFlange(trajectory), cancel_flag, progress_callback);
        });

    return AsyncOptimizeTask(std::move(future), cancel_flag);
}

std::vector<std::array<double, 8>> TrajectoryOptimizer::optimizeFromCSV(const std::string& csv_file, std::function<void(const std::string&)> progress_callback)
{

    auto trajectory = loadTrajectoryFromCSV(csv_file);
    if (trajectory.empty()) {
        throw std::runtime_error("轨迹数据加载失败: " + csv_file);
    }
    return optimizeTrajectory(trajectory, progress_callback);
}

AsyncOptimizeTask TrajectoryOptimizer::optimizeFromCSVAsync(const std::string& csv_file, std::function<void(const std::string&)> progress_callback)
{

    auto trajectory = loadTrajectoryFromCSV(csv_file);
    if (trajectory.empty()) {
        throw std::runtime_error("轨迹数据加载失败: " + csv_file);
    }
    return optimizeTrajectoryAsync(trajectory, progress_callback);
}

void TrajectoryOptimizer::exportJointTrajectory(const std::vector<std::array<double, 8>>& joint_trajectory, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法创建输出文件");
    }
    file << "Index,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7,Joint8\n";

    // 设置高精度输出（参考机械臂精度需求）
    file << std::fixed << std::setprecision(6);
    int index = 1;
    for (const auto& joint : joint_trajectory) {
        file << index++<<",";
        for (size_t i = 0; i < joint.size(); ++i) {
            file << joint[i];
            if (i != joint.size() - 1) file << ","; // 最后一位不加逗号
        }
        file << "\n";
    }
}

// 将外部状态导入为热启动状态
void TrajectoryOptimizer::setWarmupState(
    const std::array<double, 8>& joints)
{
    warmupSolution.joints = joints;
    warmupSolution.valid = true;
    warmupSolution.velocities.fill(0);
    warmupTargetPoses_ = arm_.getTCPTransform(joints);
}

// 清除缓存状态，强制冷启动
void TrajectoryOptimizer::clearWarmupState()
{
    warmupSolution.valid = false;
}

std::vector<std::array<double, 8>> TrajectoryOptimizer::optimizeTrajectoryInternal(
    const std::vector<Eigen::Matrix4d>& trajectory,
    std::shared_ptr<std::atomic<bool>> cancel_flag,
    std::function<void(const std::string&)> progress_callback)
{
    if (trajectory.empty()) throw std::runtime_error("Trajectory is empty");

    per_point_elapsed_ms_.clear();
    per_point_elapsed_ms_.reserve(trajectory.size());

    std::vector<std::array<double, 8>> joint_trajectory;
    joint_trajectory.reserve(trajectory.size());

    auto start_time = std::chrono::high_resolution_clock::now();

    // Initialize progress callback
    if (progress_callback) {
        progress_callback("Starting trajectory optimization, total " + std::to_string(trajectory.size()) + " points...");
    }

    Eigen::Matrix4d prev_target_pose = Eigen::Matrix4d::Identity();
    RobotArm::Solution prev_solution;
    std::vector<Individual> prev_population;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        if (cancel_flag && cancel_flag->load()) {
            if (progress_callback) progress_callback("Optimization cancelled by user");
            throw std::runtime_error("Optimization cancelled by user");
        }

        auto start = std::chrono::high_resolution_clock::now();

        auto joints = optimize(trajectory[i], prev_target_pose, prev_solution, prev_population);

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

        per_point_elapsed_ms_.push_back(elapsed_ms);

        joint_trajectory.push_back(joints);
        prev_target_pose = trajectory[i];
    }

    if (progress_callback) {
        progress_callback("Optimization completed, total " + std::to_string(trajectory.size()) + " points");
    }

    return joint_trajectory;
}

// 通用方法实现
double TrajectoryOptimizer::evaluatePopulation(
    std::vector<Individual>& population,
    const RobotArm::Solution& ref_solution,
    const Eigen::Matrix4d& target,
    int gene_joint1, int gene_joint2) {

    double best_fitness = INFINITY;

    for (auto& ind : population) {
        double fitness = calculateFitness(ind, ref_solution, target, gene_joint1, gene_joint2);
        best_fitness = std::min(best_fitness, fitness);
    }

    return best_fitness;
}

double TrajectoryOptimizer::calculateFitness(
    Individual& ind,
    const RobotArm::Solution& ref_solution,
    const Eigen::Matrix4d& target,
    int gene_joint1, int gene_joint2) {

    const std::array<double, 2> known_angles = { ind.theta[gene_joint1], ind.theta[gene_joint2] };
    auto solutions = arm_.inverseKinematics(target, known_angles);

    if (solutions.empty()) {
        ind.valid = false;
        ind.fitness = INFINITY;
        return INFINITY;
    }

    ind.valid = true;
    double min_fitness = INFINITY;

    for (auto& sol : solutions) {
        calcVelocities(sol, ref_solution.joints);

        const double continuity = calcContinuityCost(sol, ref_solution);
        const double jointlimits= calcJointLimitCost(sol.joints);
        const double fitness = params_.task_weights[0] * continuity+ params_.task_weights[1] * jointlimits;
        if (fitness < min_fitness) {
            if (!collision_system_.CheckCollision(sol.joints)) {
                min_fitness = fitness;
                ind.joints = sol.joints;
                ind.continuityCost = continuity;
                ind.jointCost = jointlimits;
            }

        }
    }

    if (min_fitness != INFINITY) {
        ind.valid = true;
        ind.fitness = min_fitness;  
    }
    else {
        ind.valid = false;
        ind.fitness = INFINITY;
        ind.continuityCost = INFINITY;
        ind.jointCost = INFINITY;
    }

    return ind.fitness;
}

double TrajectoryOptimizer::calcContinuityCost(
    const RobotArm::Solution& sol,
    const RobotArm::Solution& ref) const {

    std::array<double, 8> zeroArray = {};
    if (ref.joints == zeroArray) {
        return 0;
    }

    const double sigma_c = PI / 18;
    const double sigma_dc = 30;
    double cost = 0.0;

    for (int i = 0; i < 3; i++) {
        double delta = fabs(sol.joints[i] - ref.joints[i]);
        if (is_continuity_) {
            cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_dc, 2)));
        }
        else {
            cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_dc, -2)));
        }
    }

    for (int i = 3; i < 8; i++) {
        double delta = fabs(sol.joints[i] - ref.joints[i]);
        if (i == 3) {
            if (is_continuity_) {
                cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / (sigma_c / 10), 3)));
            }
            else {
                cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_c, -2)));
            }
        }
        else {
            if (is_continuity_) {
                cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_c, 2)));
            }
            else {
                cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_c, -2)));
            }
        }
    }

    const double delta = 3;  // 速度死区阈值

    for (int i = 0; i < 8; i++) {
        double vel_diff = fabs(sol.velocities[i] - ref.velocities[i]);
        double vel_penalty = (std::max(0.0, vel_diff - delta));
        cost += params_.velo_weights[i] * vel_penalty;
    }

    return cost;
}

double TrajectoryOptimizer::calcJointLimitCost(const std::array<double, 8>& q) const {
    double cost = 0.0;

    for (int i = 0; i < 8; i++) {
        double safe_range = 0.2 * (limits_.max[i] - limits_.min[i]);
        cost += params_.joint_limits_weights[i] *
            (2.0 / (1.0 + exp((q[i] - limits_.min[i]) / safe_range)) +
                2.0 / (1.0 + exp(-(q[i] - limits_.max[i]) / safe_range)));
    }

    return cost;
}

void TrajectoryOptimizer::calcVelocities(
    RobotArm::Solution& sol,
    const std::array<double, 8>& ref_joints) {

    double sum_sq = 0.0;
    for (int i = 0; i < 3; i++) {
        sum_sq += pow((sol.joints[i] - ref_joints[i]) / 10.0, 2);
    }
    for (int i = 3; i < 8; i++) {
        sum_sq += pow(sol.joints[i] - ref_joints[i], 2);
    }

    double T = sqrt(sum_sq) / params_.velocities;

    for (int i = 0; i < 3; i++) {
        sol.velocities[i] = (sol.joints[i] - ref_joints[i]) / 10.0 / T;
    }
    for (int i = 3; i < 8; i++) {
        sol.velocities[i] = (sol.joints[i] - ref_joints[i]) / T;
    }
}

std::vector<TrajectoryOptimizer::Individual> GeneticOptimizer::select(
    const std::vector<Individual>& population) {

    std::vector<const Individual*> valid_individuals;
    double max_fitness = -INFINITY;

    for (const auto& ind : population) {
        if (ind.valid) {
            valid_individuals.push_back(&ind);
            max_fitness = std::max(max_fitness, ind.fitness);
        }
    }

    if (valid_individuals.empty()) {
        return {};
    }

    double sum = 0.0;
    std::vector<double> probs;
    for (const auto ind : valid_individuals) {
        const double f_prime = max_fitness - ind->fitness + 1e-6;
        probs.push_back(f_prime);
        sum += f_prime;
    }

    std::vector<double> wheel;
    double cum = 0.0;
    for (const auto p : probs) {
        cum += p / sum;
        wheel.push_back(cum);
    }

    std::vector<Individual> selected;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (size_t i = 0; i < population.size(); i++) {
        const double r = dist(rng_);
        auto it = std::lower_bound(wheel.begin(), wheel.end(), r);
        const int index = std::distance(wheel.begin(),
            (it == wheel.end()) ? wheel.end() - 1 : it);
        selected.push_back(*valid_individuals[index]);
    }

    return selected;
}

TrajectoryOptimizer::Individual TrajectoryOptimizer::getBestValidIndividual(
    const std::vector<Individual>& population) {

    auto valid_individuals = getValidIndividuals(population);
    if (valid_individuals.empty()) {
        Individual invalid;
        invalid.valid = false;
        invalid.fitness = INFINITY;
        return invalid;
    }

    return *std::min_element(valid_individuals.begin(), valid_individuals.end(),
        [](const Individual& a, const Individual& b) {
            return a.fitness < b.fitness;
        });
}

std::vector<TrajectoryOptimizer::Individual> TrajectoryOptimizer::getValidIndividuals(
    const std::vector<Individual>& population) {
    std::vector<Individual> valid_inds;
    for (const auto& ind : population) {
        if (ind.valid) {
            valid_inds.push_back(ind);
        }
    }
    return valid_inds;
}

std::vector<TrajectoryOptimizer::Individual> TrajectoryOptimizer::initializePopulation(
    const Eigen::Matrix4d& target_pose,
    const Eigen::Matrix4d& prev_target_pose,
    const RobotArm::Solution& prev_solution,
    const std::vector<Individual>& prev_population) {

    std::vector<Individual> population;
    population.reserve(params_.population_size);

    if (is_continuity_ && !prev_population.empty()) {
        for (const auto& prev_ind : prev_population) {
            population.push_back(prev_ind);
        }
    }

    while ((int)population.size() < params_.population_size) {
        Individual ind = generateIndividualWithRadius(prev_solution.joints, current_search_radius_);
        population.push_back(ind);
    }

    return population;
}

TrajectoryOptimizer::Individual TrajectoryOptimizer::generateIndividualWithRadius(
    const std::array<double, 8>& ref_joints,
    double radius) {

    Individual ind;
    const int j1 = gene_joint1_;
    const int j2 = gene_joint2_;

    // 为关键关节生成角度
    std::uniform_real_distribution<double> dist_radius(-radius, radius);

    const double min_j1 = std::max(limits_.min[j1], ref_joints[j1] - radius);
    const double max_j1 = std::min(limits_.max[j1], ref_joints[j1] + radius);
    std::uniform_real_distribution<double> dist_j1(min_j1, max_j1);

    const double min_j2 = std::max(limits_.min[j2], ref_joints[j2] - radius);
    const double max_j2 = std::min(limits_.max[j2], ref_joints[j2] + radius);
    std::uniform_real_distribution<double> dist_j2(min_j2, max_j2);

    ind.theta.fill(0.0);
    ind.theta[j1] = dist_j1(rng_);
    ind.theta[j2] = dist_j2(rng_);

    return ind;
}

double TrajectoryOptimizer::determineSearchRadius(
    const Eigen::Matrix4d& target,
    const std::array<double, 8>& ref_joints,
    double init_radius) {

    limits_ = arm_.getJointLimits();
    const int param_joint1 = gene_joint1_;
    const int param_joint2 = gene_joint2_;

    double limits_joint1 = (limits_.max[param_joint1] - limits_.min[param_joint1]) / 2.0;
    double limits_joint2 = (limits_.max[param_joint2] - limits_.min[param_joint2]) / 2.0;

    const double full_search_space = std::max(limits_joint1, limits_joint2);

    double best_radius = init_radius;
    int max_valid_count = 0;

    for (double s = init_radius; s <= full_search_space; s *= 1.5) {
        const double axis1_range = std::min(s * limits_joint1 / (limits_joint1 + limits_joint2), limits_joint1);
        const double axis2_range = std::min(s * limits_joint2 / (limits_joint1 + limits_joint2), limits_joint2);

        int valid_count = 0;
        for (int i = 0; i < 20; i++) {
            std::uniform_real_distribution<double> dist1(-axis1_range, axis1_range);
            std::uniform_real_distribution<double> dist2(-axis2_range, axis2_range);

            const double theta_joint1 = ref_joints[param_joint1] + dist1(rng_);
            const double theta_joint2 = ref_joints[param_joint2] + dist2(rng_);

            auto solutions = arm_.inverseKinematics(target, { theta_joint1, theta_joint2 });
            if (!solutions.empty()) {
                valid_count++;
            }
        }

        if (valid_count * 1.2 > max_valid_count || max_valid_count == 0) {
            best_radius = s;
            max_valid_count = valid_count;
        }
        else {
            break;
        }
    }

    return best_radius;
}

void TrajectoryOptimizer::updateGeneJoints(const RobotArm::CoreParams& cp) {
    // 示例: 根据目标位置决定基因关节索引, 这里只是示范
    string actual_method = arm_.getAutoMethod(cp);
    if (actual_method == "45")
    {
        gene_joint2_ = 5;
    }
    else
        gene_joint2_ = 7;
}

//std::vector<TrajectoryOptimizer::Individual> TrajectoryOptimizer::handleMigration(
//    const std::vector<Individual>& prev_population,
//    const Eigen::Matrix4d& target_pose,
//    const Eigen::Matrix4d& prev_target_pose) {
//
//    std::vector<Individual> population;
//
//    // 计算旋转差异
//    const Eigen::Matrix3d prev_rot = prev_target_pose.block<3, 3>(0, 0);
//    const Eigen::Matrix3d curr_rot = target_pose.block<3, 3>(0, 0);
//    const double rotation_diff = (curr_rot - prev_rot).norm();
//
//    if (rotation_diff < 0.3) {
//        const int migrant_num = static_cast<int>(params_.population_size * params_.migration_rate);
//
//        for (int i = 0; i < migrant_num && i < (int)prev_population.size(); i++) {
//            Individual migrant = prev_population[i];
//            migrant.valid = false;
//            migrant.fitness = INFINITY;
//            migrant.joints.fill(0);
//            population.push_back(migrant);
//        }
//    }
//
//    return population;
//}
std::vector<TrajectoryOptimizer::Individual> TrajectoryOptimizer::handleMigration(
const std::vector<TrajectoryOptimizer::Individual>& prev_population,
const Eigen::Matrix4d& target_pose,
const Eigen::Matrix4d& prev_target_pose) {

    std::vector<Individual> population;

    // 提取位置部分
    Eigen::Vector3d prev_pos = prev_target_pose.block<3, 1>(0, 3);
    Eigen::Vector3d curr_pos = target_pose.block<3, 1>(0, 3);

    // 提取旋转部分转换为四元数
    Eigen::Quaterniond prev_quat(prev_target_pose.block<3, 3>(0, 0));
    Eigen::Quaterniond curr_quat(target_pose.block<3, 3>(0, 0));

    // 计算位置差异
    double pos_diff = (curr_pos - prev_pos).norm();

    // 计算旋转差异（弧度）
    Eigen::Quaterniond delta_q = prev_quat.conjugate() * curr_quat;
    delta_q.normalize();
    double rot_diff = 2.0 * std::acos(std::abs(delta_q.w()));

    const double pos_threshold = 1000;  // 50mm
    const double rot_threshold = 0.2;   // 0.2 rad

    //std::cout << "pos_diff*a+rot_diff*b:" << pos_diff <<"+"<< rot_diff<<std::endl;

    if (pos_diff < pos_threshold && rot_diff < rot_threshold) {
        const int migrant_num = static_cast<int>(params_.population_size * params_.migration_rate);

        for (int i = 0; i < migrant_num && i < static_cast<int>(prev_population.size()); i++) {
            Individual migrant = prev_population[i];
            migrant.valid = false;
            migrant.fitness = std::numeric_limits<double>::infinity();
            migrant.joints.fill(0);
            population.push_back(migrant);
        }
    }

    return population;
}
// ==================== 参数空间扫描实现 ====================

double TrajectoryOptimizer::runSingleTest(
    const Eigen::Matrix4d& test_pose,
    const RobotArm::Solution& warmup_solution,
    const std::vector<Individual>& warmup_population,
    int timeout_ms)
{
    auto start = std::chrono::high_resolution_clock::now();

    try {
        last_best_fitness_ = INFINITY;

        Eigen::Matrix4d prev_pose = Eigen::Matrix4d::Identity();
        RobotArm::Solution prev_solution = warmup_solution;
        std::vector<Individual> prev_population = warmup_population;

        optimize(test_pose, prev_pose, prev_solution, prev_population);

    }
    catch (const std::exception& e) {
        return INFINITY;
    }

    auto end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (elapsed_ms > timeout_ms) {
        return INFINITY;
    }

    return last_best_fitness_;
}

std::vector<TrajectoryOptimizer::ExperimentResult>
TrajectoryOptimizer::scanParameterSpace(
    const Eigen::Matrix4d& warmup_pose,      // 第一个位姿
    const Eigen::Matrix4d& test_pose,        // 第二个位姿
    const Eigen::Matrix4d& actual_test_pose, // 第三个位姿（实际测试用）
    int pop_size_divisions,
    int migration_rate_divisions,
    int timeout_ms,
    std::function<void(const std::string&)> progress)
{
    std::vector<ExperimentResult> results;

    // 临时保存原始参数
    int orig_pop = params_.population_size;
    double orig_mig = params_.migration_rate;
    int orig_gen = params_.max_generations;

    params_.max_generations = 1;

    // ===== 阶段1：第一次热启动（初始 → warmup_pose） =====
    if (progress) {
        progress("\n========== 阶段1: 第一次热启动 (初始→warmup) ==========");
    }

    params_.population_size = 100;
    params_.migration_rate = 0;

    Eigen::Matrix4d prev_pose_1 = Eigen::Matrix4d::Identity();
    RobotArm::Solution prev_solution_1;
    prev_solution_1.valid = false;
    prev_solution_1.joints = { 0, 0, 0, 0, 0, 0, 0, 0 };
    prev_solution_1.velocities = { 0, 0, 0, 0, 0, 0, 0, 0 };
    std::vector<Individual> prev_population_1;

    optimize(warmup_pose, prev_pose_1, prev_solution_1, prev_population_1);

    if (progress) {
        progress("✓ 第一次热启动完成！population size: " +
            std::to_string(prev_population_1.size()));
    }

    // ===== 阶段2：第二次热启动（warmup_pose → test_pose） =====
    if (progress) {
        progress("\n========== 阶段2: 第二次热启动 (warmup→test) ==========");
    }

    Eigen::Matrix4d prev_pose_2 = prev_pose_1;          // 现在是warmup_pose
    RobotArm::Solution prev_solution_2 = prev_solution_1;
    std::vector<Individual> prev_population_2 = prev_population_1;

    optimize(test_pose, prev_pose_2, prev_solution_2, prev_population_2);

    if (progress) {
        progress("✓ 第二次热启动完成！population size: " +
            std::to_string(prev_population_2.size()));
    }

    // ===== 阶段3：参数空间扫描（test_pose → actual_test_pose） =====
    if (progress) {
        progress("\n========== 阶段3: 参数空间扫描 (test→actual_test) ==========");
        progress("种群规模: 1-" + std::to_string(pop_size_divisions) +
            " (" + std::to_string(pop_size_divisions) + "份)");
        progress("迁移比例: 0-1.0 (" + std::to_string(migration_rate_divisions) + "份)");
        progress("总参数组合数: " + std::to_string(pop_size_divisions * migration_rate_divisions) + "\n");
    }

    int total_combinations = pop_size_divisions * migration_rate_divisions;
    int current_combination = 0;

    for (int pop_idx = 0; pop_idx < pop_size_divisions; ++pop_idx) {
        int pop_size = (pop_idx+1);
   

        for (int mig_idx = 0; mig_idx < migration_rate_divisions; ++mig_idx) {
            current_combination++;

            double mig_rate = static_cast<double>(mig_idx) / (migration_rate_divisions );

            params_.population_size = pop_size;
            params_.migration_rate = mig_rate;

            if (progress && current_combination % 1000 == 0) {
                std::string msg = "Progress: " + std::to_string(current_combination) + "/" +
                    std::to_string(total_combinations) +
                    " | PopSize=" + std::to_string(pop_size) +
                    ", MigRate=" + std::to_string(mig_rate);
                progress(msg);
            }

            // 使用第二次热启动的结果，测试第三个位姿
            double fitness = runSingleTest(
                actual_test_pose,    // 第三个位姿！
                prev_solution_2,     // 从test_pose的解开始
                prev_population_2,   // 从test_pose的种群开始
                timeout_ms);

            ExperimentResult result;
            result.population_size = pop_size;
            result.migration_rate = mig_rate;
            result.fitness = fitness;
            results.push_back(result);
        }
    }

    if (progress) {
        progress("\n✓ 参数空间扫描完成！共" + std::to_string(total_combinations) +
            "个数据点");
    }

    // 恢复原始参数
    params_.population_size = orig_pop;
    params_.migration_rate = orig_mig;
    params_.max_generations = orig_gen;

    return results;
}

void TrajectoryOptimizer::exportExperimentResults(
    const std::vector<ExperimentResult>& results,
    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    file << "PopulationSize,MigrationRate,Fitness\n";

    for (const auto& result : results) {
        file << result.population_size << ","
            << std::fixed << std::setprecision(4) << result.migration_rate << ","
            << std::scientific << std::setprecision(6) << result.fitness << "\n";
    }

    file.close();
    std::cout << "\n✓ Results exported to: " << filename << std::endl;
}

void TrajectoryOptimizer::exportJointTrajectoryWithTime(const std::vector<std::array<double, 8>>& joint_trajectory, const std::vector<double>& elapsed_times_ms, const std::string& filename)
{
    if (joint_trajectory.size() != elapsed_times_ms.size())
        throw std::runtime_error("Joint trajectory and elapsed times size mismatch");

    std::ofstream file(filename);
    if (!file.is_open()) throw std::runtime_error("无法创建输出文件");

    file << "Index,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7,Joint8,Elapsed_ms\n";
    file << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < joint_trajectory.size(); ++i) {
        file << (i + 1) << ",";
        for (size_t j = 0; j < 8; ++j) {
            file << joint_trajectory[i][j];
            if (j != 7) file << ",";
        }
        file << "," << std::setprecision(3) << elapsed_times_ms[i] << "\n";
    }
}

// =====================================================================
//GA
std::vector<TrajectoryOptimizer::Individual> GeneticOptimizer::reproduction(
    const std::vector<Individual>& selected) {

    std::vector<Individual> offspring;
    offspring.reserve(selected.size());

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (size_t i = 0; i + 1 < selected.size(); i += 2) {
        const Individual& parent1 = selected[i];
        const Individual& parent2 = selected[i + 1];

        Individual child1 , child2;
        if (dist(rng_) < params_.crossover_rate) {
            applySBX(parent1, parent2, child1, child2);
        }

        applyMutation(child1);
        applyMutation(child2);

        offspring.push_back(child1);
        offspring.push_back(child2);
    }

    return offspring;
}

void GeneticOptimizer::applySBX(
    const Individual& parent1, const Individual& parent2,
    Individual& child1, Individual& child2) {

    const double eta_c = 2.0;
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < 8; ++i) {
        double u = dist(rng_);
        double beta;
        if (u <= 0.5) {
            beta = pow(2 * u, 1.0 / (eta_c + 1));
        }
        else {
            beta = pow(1 / (2 * (1 - u)), 1.0 / (eta_c + 1));
        }
        double p1 = parent1.theta[i];
        double p2 = parent2.theta[i];
        child1.theta[i] = 0.5 * ((1 + beta) * p1 + (1 - beta) * p2);
        child2.theta[i] = 0.5 * ((1 - beta) * p1 + (1 + beta) * p2);
    }
}

void GeneticOptimizer::applyMutation(Individual& ind) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::normal_distribution<double> norm_dist(0.0, 0.1);

    for (int i = 0; i < 8; ++i) {
        if (dist(rng_) < params_.mutation_rate) {
            ind.theta[i] += norm_dist(rng_);
        }
    }
}


//NSGA
std::vector<TrajectoryOptimizer::Individual> NSGAOptimizer::select(
    const std::vector<Individual>& population) {

    std::vector<Individual> pop_copy = population;

    // 快速非支配排序
    fastNonDominatedSort(pop_copy);

    // 锦标赛选择
    std::vector<Individual> selected;
    tournamentSelectionNSGA(pop_copy, selected);

    return selected;
}

void NSGAOptimizer::fastNonDominatedSort(std::vector<Individual>& population) {
    std::vector<DominationInfo> dom_info(population.size());
    std::vector<std::vector<int>> fronts;
    std::vector<int> current_front;

    // 计算支配关系
    for (int i = 0; i < population.size(); ++i) {
        for (int j = 0; j < population.size(); ++j) {
            if (i != j) {
                if (dominates(population[i], population[j])) {
                    dom_info[i].dominated_solutions.push_back(j);
                }
                else if (dominates(population[j], population[i])) {
                    dom_info[i].domination_count++;
                }
            }
        }

        if (dom_info[i].domination_count == 0) {
            population[i].rank = 0;
            current_front.push_back(i);
        }
    }

    fronts.push_back(current_front);

    // 生成后续前沿
    int front_index = 0;
    while (!fronts[front_index].empty()) {
        std::vector<int> next_front;

        for (int i : fronts[front_index]) {
            for (int j : dom_info[i].dominated_solutions) {
                dom_info[j].domination_count--;
                if (dom_info[j].domination_count == 0) {
                    population[j].rank = front_index + 1;
                    next_front.push_back(j);
                }
            }
        }

        front_index++;
        if (!next_front.empty()) {
            fronts.push_back(next_front);
        }
        else {
            break;
        }
    }

    // 计算拥挤距离
    for (const auto& front : fronts) {
        if (front.size() > 2) {
            std::vector<Individual> front_individuals;
            for (int idx : front) {
                front_individuals.push_back(population[idx]);
            }
            calculateCrowdingDistance(front_individuals);

            for (int i = 0; i < front.size(); ++i) {
                population[front[i]].crowding_distance = front_individuals[i].crowding_distance;
            }
        }
    }
}

bool NSGAOptimizer::dominates(const Individual& a, const Individual& b) {
    if (!a.valid && !b.valid) return false;
    if (!a.valid) return false;
    if (!b.valid) return true;

    // 多目标比较：连续性、关节限制
    bool better_in_any = false;
    bool worse_in_any = false;

    // 目标1：连续性成本（越小越好）
    if (a.continuityCost <= b.continuityCost) better_in_any = true;
    else  worse_in_any = true;

    // 目标2：关节限制成本（越小越好）
    if (a.jointCost < b.jointCost) better_in_any = true;
    else  worse_in_any = true;

    return better_in_any && !worse_in_any;
}

std::vector<double> NSGAOptimizer::calculateObjectives(const Individual& ind)
{
    std::vector<double> objectives(3);

    if (!ind.valid || ind.fitness == INFINITY) {
        objectives[0] = std::numeric_limits<double>::infinity(); // fitness (综合成本)
        objectives[1] = std::numeric_limits<double>::infinity(); // 连续性成本
        objectives[2] = std::numeric_limits<double>::infinity(); // 关节限制成本
        return objectives;
    }

    // 目标1：综合fitness (不能为INFINITY)
    objectives[0] = ind.fitness;

    // 目标2：连续性成本 (已在calculateFitness中计算)
    objectives[1] = ind.continuityCost;

    // 目标3：关节限制成本 (已在calculateFitness中计算)
    objectives[2] = ind.jointCost;

    return objectives;
}

std::vector<TrajectoryOptimizer::Individual> NSGAOptimizer::reproduction(
    const std::vector<Individual>& selected) {

    std::vector<Individual> offspring;
    offspring.reserve(selected.size());

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (size_t i = 0; i + 1 < selected.size(); i += 2) {
        Individual child1 = selected[i];
        Individual child2 = selected[i + 1];

        // SBX交叉
        if (dist(rng_) < params_.crossover_rate) {
            const double eta_c = 20.0;  // NSGA-II推荐值

            for (int j = 0; j < 8; ++j) {
                double u = dist(rng_);
                double beta;
                if (u <= 0.5) {
                    beta = pow(2 * u, 1.0 / (eta_c + 1));
                }
                else {
                    beta = pow(1 / (2 * (1 - u)), 1.0 / (eta_c + 1));
                }

                double p1 = selected[i].theta[j];
                double p2 = selected[i + 1].theta[j];
                child1.theta[j] = 0.5 * ((1 + beta) * p1 + (1 - beta) * p2);
                child2.theta[j] = 0.5 * ((1 - beta) * p1 + (1 + beta) * p2);
            }
        }

        // 多项式变异
        const double eta_m = 20.0;
        for (int j = 0; j < 8; ++j) {
            if (dist(rng_) < params_.mutation_rate) {
                double delta;
                double u = dist(rng_);
                if (u < 0.5) {
                    delta = pow(2 * u, 1.0 / (eta_m + 1)) - 1;
                }
                else {
                    delta = 1 - pow(2 * (1 - u), 1.0 / (eta_m + 1));
                }
                child1.theta[j] += delta * 0.1;
                child2.theta[j] += delta * 0.1;
            }
        }

        offspring.push_back(child1);
        offspring.push_back(child2);
    }

    return offspring;
}
void NSGAOptimizer::calculateCrowdingDistance(std::vector<Individual>& front) {
    if (front.size() <= 2) {
        for (auto& ind : front) {
            ind.crowding_distance = std::numeric_limits<double>::infinity();
        }
        return;
    }

    // 初始化拥挤距离
    for (auto& ind : front) {
        ind.crowding_distance = 0.0;
    }

    // 对每个目标函数计算拥挤距离
    const int num_objectives = 3; // 连续性、关节限制、速度平滑性

    for (int obj = 0; obj < num_objectives; ++obj) {
        // 按目标函数值排序
        std::sort(front.begin(), front.end(), [obj, this](const Individual& a, const Individual& b) {
            std::vector<double> obj_a = calculateObjectives(a);
            std::vector<double> obj_b = calculateObjectives(b);
            return obj_a[obj] < obj_b[obj];
            });

        // 边界个体设置为无穷大
        front[0].crowding_distance = std::numeric_limits<double>::infinity();
        front.back().crowding_distance = std::numeric_limits<double>::infinity();

        // 计算目标函数的范围
        std::vector<double> first_obj = calculateObjectives(front[0]);
        std::vector<double> last_obj = calculateObjectives(front.back());
        double obj_range = last_obj[obj] - first_obj[obj];

        if (obj_range > 1e-10) { // 避免除零
            // 计算中间个体的拥挤距离
            for (int i = 1; i < front.size() - 1; ++i) {
                std::vector<double> prev_obj = calculateObjectives(front[i - 1]);
                std::vector<double> next_obj = calculateObjectives(front[i + 1]);

                front[i].crowding_distance += (next_obj[obj] - prev_obj[obj]) / obj_range;
            }
        }
    }
}

void NSGAOptimizer::tournamentSelectionNSGA(const std::vector<Individual>& population,
    std::vector<Individual>& selected) {
    selected.clear();
    selected.reserve(population.size());

    std::uniform_int_distribution<int> dist(0, population.size() - 1);

    for (int i = 0; i < population.size(); ++i) {
        // 锦标赛大小为2
        int idx1 = dist(rng_);
        int idx2 = dist(rng_);

        const Individual& ind1 = population[idx1];
        const Individual& ind2 = population[idx2];

        // NSGA-II选择规则：
        // 1. 优先选择rank更小的
        // 2. rank相同时选择拥挤距离更大的
        if (ind1.rank < ind2.rank ||
            (ind1.rank == ind2.rank && ind1.crowding_distance > ind2.crowding_distance)) {
            selected.push_back(ind1);
        }
        else {
            selected.push_back(ind2);
        }
    }
}

// MOEA/D 实现
std::vector<TrajectoryOptimizer::Individual> MOEADOptimizer::select(
    const std::vector<Individual>& population) {

    if (weight_vectors_.empty()) {
        initializeWeights();
        findNeighbors();
    }

    std::vector<Individual> selected;
    selected.reserve(population.size());

    // 对每个子问题进行选择
    for (size_t i = 0; i < std::min(weight_vectors_.size(), population.size()); ++i) {
        double best_value = std::numeric_limits<double>::infinity();
        Individual best_individual;
        bool found = false;

        // 在邻域中寻找最优解
        for (int neighbor_idx : weight_vectors_[i % weight_vectors_.size()].neighbors) {
            if (neighbor_idx < static_cast<int>(population.size())) {
                double value = scalarizeObjective(population[neighbor_idx],
                    weight_vectors_[i % weight_vectors_.size()]);
                if (value < best_value) {
                    best_value = value;
                    best_individual = population[neighbor_idx];
                    found = true;
                }
            }
        }

        if (found) {
            selected.push_back(best_individual);
        }
        else {
            selected.push_back(population[i % population.size()]);
        }
    }

    // 确保选择的个体数量与原种群相同
    while (selected.size() < population.size()) {
        selected.push_back(population[selected.size() % population.size()]);
    }

    return selected;
}

std::vector<TrajectoryOptimizer::Individual> MOEADOptimizer::reproduction(
    const std::vector<Individual>& selected) {

    std::vector<Individual> offspring;
    offspring.reserve(selected.size());

    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<int> int_dist(0, selected.size() - 1);

    for (size_t i = 0; i < selected.size(); ++i) {
        const Individual& parent = selected[i];

        // DE/rand/1变异策略
        int r1, r2, r3;
        do { r1 = int_dist(rng_); } while (r1 == static_cast<int>(i));
        do { r2 = int_dist(rng_); } while (r2 == static_cast<int>(i) || r2 == r1);
        do { r3 = int_dist(rng_); } while (r3 == static_cast<int>(i) || r3 == r1 || r3 == r2);

        Individual trial = parent;

        // 对 theta 数组进行变异和交叉
        for (int j = 0; j < 8; ++j) {
            if (dist(rng_) < params_.crossover_rate) { // 使用参数中的交叉率
                trial.theta[j] = selected[r1].theta[j] +
                    0.5 * (selected[r2].theta[j] - selected[r3].theta[j]);
            }
        }

        // 重置适应度信息，等待重新计算
        trial.fitness = INFINITY;
        trial.valid = false;
        trial.continuityCost = 0.0;
        trial.jointCost = 0.0;
        trial.rank = 0;
        trial.crowding_distance = 0.0;

        offspring.push_back(trial);
    }

    return offspring;
}

void MOEADOptimizer::initializeWeights() {
    weight_vectors_.clear();
    int H = 10; // 划分数

    // 生成均匀分布的权重向量
    for (int i = 0; i <= H; ++i) {
        for (int j = 0; j <= H - i; ++j) {
            int k = H - i - j;
            WeightVector weight;
            weight.weights.push_back(static_cast<double>(i) / H);
            weight.weights.push_back(static_cast<double>(j) / H);
            weight.weights.push_back(static_cast<double>(k) / H);
            weight_vectors_.push_back(weight);
        }
    }
}

void MOEADOptimizer::findNeighbors() {
    for (int i = 0; i < weight_vectors_.size(); ++i) {
        std::vector<std::pair<double, int>> distances;

        for (int j = 0; j < weight_vectors_.size(); ++j) {
            if (i != j) {
                double dist = 0.0;
                for (int k = 0; k < 3; ++k) {
                    double diff = weight_vectors_[i].weights[k] - weight_vectors_[j].weights[k];
                    dist += diff * diff;
                }
                distances.push_back({ sqrt(dist), j });
            }
        }

        std::sort(distances.begin(), distances.end());

        weight_vectors_[i].neighbors.clear();
        weight_vectors_[i].neighbors.push_back(i); // 包含自己
        for (int j = 0; j < std::min(T - 1, static_cast<int>(distances.size())); ++j) {
            weight_vectors_[i].neighbors.push_back(distances[j].second);
        }
    }
}

double MOEADOptimizer::scalarizeObjective(const Individual& ind, const WeightVector& weight) {
    std::vector<double> objectives = calculateObjectives(ind);

    // Tchebycheff方法
    double max_value = 0.0;
    for (int i = 0; i < objectives.size(); ++i) {
        double value = weight.weights[i] * std::abs(objectives[i] - 0.0); // 理想点为0
        max_value = std::max(max_value, value);
    }

    return max_value;
}

std::vector<double> MOEADOptimizer::calculateObjectives(const Individual& ind) {
    std::vector<double> objectives(3);

    if (!ind.valid || ind.fitness == INFINITY) {
        objectives[0] = std::numeric_limits<double>::infinity();
        objectives[1] = std::numeric_limits<double>::infinity();
        objectives[2] = std::numeric_limits<double>::infinity();
        return objectives;
    }

    objectives[0] = ind.fitness;
    objectives[1] = ind.continuityCost;
    objectives[2] = ind.jointCost;

    return objectives;
}



// ISGA-BT 实现
std::vector<TrajectoryOptimizer::Individual> ISGAOptimizer::select(
    const std::vector<Individual>& population) {

    if (islands_.empty()) {
        initializeIslands(population);
    }

    // 进化各个岛屿
    evolveIslands();

    // 执行迁移（根据参数中的 migration_rate）
    static int generation_counter = 0;
    generation_counter++;
    if (generation_counter % migration_interval == 0) {
        migration();
    }

    // 收集所有岛屿的个体
    std::vector<Individual> selected;
    for (const auto& island : islands_) {
        for (const auto& ind : island.population) {
            selected.push_back(ind);
        }
    }

    // 如果个体数量不够，进行精英选择
    if (selected.size() > population.size()) {
        std::sort(selected.begin(), selected.end()); // 使用 Individual 的 operator<
        selected.resize(population.size());
    }
    else if (selected.size() < population.size()) {
        // 复制最优个体直到达到所需大小
        while (selected.size() < population.size()) {
            selected.push_back(selected[0]); // 复制最优个体
        }
    }

    return selected;
}

void ISGAOptimizer::initializeIslands(const std::vector<Individual>& population) {
    islands_.clear();
    islands_.resize(num_islands);

    int island_size = population.size() / num_islands;

    for (int i = 0; i < num_islands; ++i) {
        int start_idx = i * island_size;
        int end_idx = (i == num_islands - 1) ?
            static_cast<int>(population.size()) : (i + 1) * island_size;

        islands_[i].population.assign(population.begin() + start_idx,
            population.begin() + end_idx);
        islands_[i].migration_generation = 0;
    }
}

void ISGAOptimizer::evolveIslands() {
    for (auto& island : islands_) {
        // 对每个岛屿进行一代进化
        std::vector<Individual> selected;
        binaryTournament(island.population, selected);

        // 交叉和变异（类似遗传算法）
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::vector<Individual> offspring;

        for (size_t i = 0; i + 1 < selected.size(); i += 2) {
            Individual child1 = selected[i];
            Individual child2 = selected[i + 1];

            // 交叉
            if (dist(rng_) < params_.crossover_rate) {
                for (int j = 0; j < 8; ++j) {
                    if (dist(rng_) < 0.5) {
                        std::swap(child1.theta[j], child2.theta[j]);
                    }
                }
            }

            // 变异
            for (int j = 0; j < 8; ++j) {
                if (dist(rng_) < params_.mutation_rate) {
                    child1.theta[j] += dist(rng_) * 0.1 - 0.05;
                    child2.theta[j] += dist(rng_) * 0.1 - 0.05;
                }
            }

            offspring.push_back(child1);
            offspring.push_back(child2);
        }

        // 精英选择
        island.population.insert(island.population.end(), offspring.begin(), offspring.end());
        eliteSelection(island.population);
    }
}

void ISGAOptimizer::migration() {
    int migration_count = static_cast<int>(params_.population_size * migration_rate / num_islands);

    for (int i = 0; i < num_islands; ++i) {
        if (islands_[i].population.empty()) continue;

        // 排序找到最优个体
        std::sort(islands_[i].population.begin(), islands_[i].population.end());

        // 迁移到下一个岛屿
        int target_island = (i + 1) % num_islands;
        for (int j = 0; j < migration_count && j < static_cast<int>(islands_[i].population.size()); ++j) {
            islands_[target_island].population.push_back(islands_[i].population[j]);
        }
    }

    // 保持岛屿大小
    for (auto& island : islands_) {
        if (island.population.size() > params_.population_size / num_islands) {
            std::sort(island.population.begin(), island.population.end());
            island.population.resize(params_.population_size / num_islands);
        }
    }
}

std::vector<TrajectoryOptimizer::Individual> ISGAOptimizer::reproduction(
    const std::vector<Individual>& selected) {

    std::vector<Individual> offspring;
    offspring.reserve(selected.size());

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (size_t i = 0; i + 1 < selected.size(); i += 2) {
        Individual child1 = selected[i];
        Individual child2 = selected[i + 1];

        // 使用参数中的交叉率
        if (dist(rng_) < params_.crossover_rate) {
            // 均匀交叉
            for (int j = 0; j < 8; ++j) {
                if (dist(rng_) < 0.5) {
                    std::swap(child1.theta[j], child2.theta[j]);
                }
            }
        }

        // 使用参数中的变异率
        std::normal_distribution<double> gauss_dist(0.0, 0.1);
        for (int j = 0; j < 8; ++j) {
            if (dist(rng_) < params_.mutation_rate) {
                child1.theta[j] += gauss_dist(rng_);
                child2.theta[j] += gauss_dist(rng_);
            }
        }

        // 重置适应度信息
        child1.fitness = INFINITY;
        child1.valid = false;
        child1.continuityCost = 0.0;
        child1.jointCost = 0.0;
        child1.rank = 0;
        child1.crowding_distance = 0.0;

        child2.fitness = INFINITY;
        child2.valid = false;
        child2.continuityCost = 0.0;
        child2.jointCost = 0.0;
        child2.rank = 0;
        child2.crowding_distance = 0.0;

        offspring.push_back(child1);
        offspring.push_back(child2);
    }

    // 如果 selected 的大小是奇数，处理最后一个个体
    if (selected.size() % 2 == 1) {
        Individual child = selected.back();

        // 变异
        std::normal_distribution<double> gauss_dist(0.0, 0.1);
        for (int j = 0; j < 8; ++j) {
            if (dist(rng_) < params_.mutation_rate) {
                child.theta[j] += gauss_dist(rng_);
            }
        }

        // 重置适应度信息
        child.fitness = INFINITY;
        child.valid = false;
        child.continuityCost = 0.0;
        child.jointCost = 0.0;
        child.rank = 0;
        child.crowding_distance = 0.0;

        offspring.push_back(child);
    }

    return offspring;
}

void ISGAOptimizer::binaryTournament(const std::vector<Individual>& population,
    std::vector<Individual>& selected) {
    selected.clear();
    selected.reserve(population.size());

    std::uniform_int_distribution<int> dist(0, population.size() - 1);

    for (int i = 0; i < population.size(); ++i) {
        int idx1 = dist(rng_);
        int idx2 = dist(rng_);

        if (population[idx1].fitness <= population[idx2].fitness) {
            selected.push_back(population[idx1]);
        }
        else {
            selected.push_back(population[idx2]);
        }
    }
}

void ISGAOptimizer::eliteSelection(std::vector<Individual>& population) {
    std::sort(population.begin(), population.end(),
        [](const Individual& a, const Individual& b) {
            return a.fitness < b.fitness;
        });

    // 保持原始大小
    int original_size = population.size() / 2; // 假设目标大小为一半
    population.resize(original_size);
}

// Differential Evolution 实现
std::vector<TrajectoryOptimizer::Individual> DifferentialOptimizer::select(
    const std::vector<Individual>& population) {

    // DE中选择就是保留当前种群
    return population;
}

std::vector<TrajectoryOptimizer::Individual> DifferentialOptimizer::reproduction(
    const std::vector<Individual>& selected) {

    std::vector<Individual> offspring;
    offspring.reserve(selected.size());

    for (size_t i = 0; i < selected.size(); ++i) {
        // DE变异
        Individual mutant = mutateDE(selected, static_cast<int>(i));

        // DE交叉
        Individual trial = crossoverDE(selected[i], mutant);

        // 边界约束
        boundaryConstraint(trial);

        // 重置适应度信息
        trial.fitness = INFINITY;
        trial.valid = false;
        trial.continuityCost = 0.0;
        trial.jointCost = 0.0;
        trial.rank = 0;
        trial.crowding_distance = 0.0;

        offspring.push_back(trial);
    }

    return offspring;
}

TrajectoryOptimizer::Individual DifferentialOptimizer::mutateDE(
    const std::vector<Individual>& population, int target_idx) {

    std::uniform_int_distribution<int> dist(0, population.size() - 1);

    // 选择三个不同的随机个体
    int r1, r2, r3;
    do { r1 = dist(rng_); } while (r1 == target_idx);
    do { r2 = dist(rng_); } while (r2 == target_idx || r2 == r1);
    do { r3 = dist(rng_); } while (r3 == target_idx || r3 == r1 || r3 == r2);

    Individual mutant = population[target_idx];

    // DE/rand/1: V = X_r1 + F * (X_r2 - X_r3)
    for (int j = 0; j < 8; ++j) {
        mutant.theta[j] = population[r1].theta[j] +
            F * (population[r2].theta[j] - population[r3].theta[j]);
    }

    return mutant;
}

TrajectoryOptimizer::Individual DifferentialOptimizer::crossoverDE(
    const Individual& target, const Individual& mutant) {

    Individual trial = target;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<int> j_rand(0, 7);

    int j_r = j_rand(rng_); // 确保至少有一个参数来自变异向量

    for (int j = 0; j < 8; ++j) {
        if (dist(rng_) <= CR || j == j_r) {
            trial.theta[j] = mutant.theta[j];
        }
    }

    return trial;
}

void DifferentialOptimizer::boundaryConstraint(Individual& individual) {
    // 根据关节限制权重来约束关节角度
    for (int i = 0; i < 8; ++i) {
        // 假设关节角度范围基于权重进行调整
        double limit_max = limits_.max[i] * (1-params_.joint_limits_weights[i]);
        double limit_min = limits_.min[i] * (1 - params_.joint_limits_weights[i]);

        if (individual.theta[i] > limit_max) {
            individual.theta[i] = limit_max;
        }
        else if (individual.theta[i] < -limit_min) {
            individual.theta[i] = -limit_min;
        }
    }
}
