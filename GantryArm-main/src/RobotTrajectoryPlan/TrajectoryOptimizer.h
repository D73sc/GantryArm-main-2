#pragma once
#include "RobotArm.h"
#include "CollisionChecker.h"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <memory>
#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <future>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>  // For std::mutex and std::lock_guard
// 进度信息结构
struct ProgressInfo {
    size_t current_point = 0;
    size_t total_points = 0;
    double current_fitness = 0.0;
    std::string status = "Initializing";
    double elapsed_seconds = 0.0;
    bool is_completed = false;
    bool is_cancelled = false;
    std::string error_message;
};
// 异步优化任务句柄
class AsyncOptimizeTask {
private:
    std::future<std::vector<std::array<double, 8>>> future_;
    std::shared_ptr<std::atomic<bool>> cancel_flag_;

public:
    AsyncOptimizeTask(std::future<std::vector<std::array<double, 8>>>&& fut,
        std::shared_ptr<std::atomic<bool>> cancel_flag)
        : future_(std::move(fut)), cancel_flag_(cancel_flag) {}
    // 默认构造函数（用于 unique_ptr 初始为空，支持 Qt 成员）
    AsyncOptimizeTask() : future_(), cancel_flag_(nullptr) {}

    // 移动语义（推荐）
    AsyncOptimizeTask(AsyncOptimizeTask&&) = default;
    AsyncOptimizeTask& operator=(AsyncOptimizeTask&&) = default;
    AsyncOptimizeTask(const AsyncOptimizeTask&) = delete;
    AsyncOptimizeTask& operator=(const AsyncOptimizeTask&) = delete;

    bool isValid() const {
        return future_.valid();
    }
    // 检查是否完成
    bool isReady() const {
        if (!future_.valid()) {
            return false;
        }
        return future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    // 取消任务
    void cancel() {
        if (!future_.valid()) {
            return;  // 无效任务，无需操作
        }
        if (cancel_flag_) {
            cancel_flag_->store(true);
        }
    }

    // 等待完成并获取结果
    std::vector<std::array<double, 8>> get() {
        if (!future_.valid()) {
            throw std::invalid_argument("Invalid AsyncOptimizeTask: cannot get result from invalid task");
        }
        return future_.get();  // 正常执行，可能抛 runtime_error（取消/异常）
    }

    // 等待一段时间
    std::future_status wait_for(const std::chrono::milliseconds& timeout) {
        if (!future_.valid()) {
            throw std::invalid_argument("Invalid AsyncOptimizeTask: cannot wait on invalid task");
        }
        return future_.wait_for(timeout);
    }
};
struct OptimizeParams {
    int population_size = 100; 
    double migration_rate = 0.8;
    int max_generations = 200;
    double velocities = 10.0;
    double crossover_rate = 0.8;
    double mutation_rate = 0.3;
    std::array<double, 8> joint_weights = { {0.008, 0.008, 0.01, 0.5, 0.1, 0.1, 0.2, 0.2} };
    std::array<double, 8> velo_weights = { {1, 1, 1, 0.5, 1, 0.2, 0.3, 0.3} };
    std::array<double, 8> joint_limits_weights = { {0.008, 0.008, 0.01, 0.2, 0.3, 0.2, 0.2, 0.2} };
    std::array<double, 2> task_weights = { {0.3, 0.1} };
};
class TrajectoryOptimizer {
public:
    struct Individual {
        std::array<double, 8> theta;
        std::array<double, 8> joints;

        // 适应度相关
        double fitness = INFINITY;             // 主要适应度
        bool valid = false;
        double continuityCost = 0.0;
        double jointCost = 0.0;

        // NSGA-II 相关
        int rank = 0;                         // 非支配排序等级
        double crowding_distance = 0.0;       // 拥挤距离

        bool operator<(const Individual& other) const {
            return fitness < other.fitness;
        }
    };



protected:
    RobotArm& arm_;
    CollisionSystem& collision_system_;
    OptimizeParams params_;
    std::mt19937 rng_;
    bool is_continuity_ = true;
    double search_radius_ = PI / 5.0;
    int gene_joint1_ = 4;
    int gene_joint2_ = 5;
    double current_search_radius_=0;
    RobotArm::JointLimits limits_;

public:
    TrajectoryOptimizer(RobotArm& arm, CollisionSystem& collision_system,
        const OptimizeParams& params = OptimizeParams())
        : arm_(arm), collision_system_(collision_system), params_(params), rng_(std::random_device{}()), limits_(arm_.getJointLimits()) {}

    virtual ~TrajectoryOptimizer() = default;

    // 主接口
    virtual std::array<double, 8> optimize(
        const Eigen::Matrix4d& target_pose,
        Eigen::Matrix4d& prev_target_pose,
        RobotArm::Solution& prev_solution,
        std::vector<Individual>& prev_population);

    void setContinuity(bool enable) { is_continuity_ = enable; }
    void setNotContinuity() { is_continuity_ = false; }

    // 工厂方法
    static std::unique_ptr<TrajectoryOptimizer> create(
        const std::string& type,
        RobotArm& arm,
        CollisionSystem& collision_system,
        const OptimizeParams& params = OptimizeParams());


    // 单点优化（同步，无回调）
    std::array<double, 8> optimizeSinglePoint(const Eigen::Matrix4d& target_pose);

    // 批量轨迹优化（同步，带字符串回调）
    std::vector<std::array<double, 8>> optimizeTrajectory(
        const std::vector<Eigen::Matrix4d>& trajectory,
        std::function<void(const std::string&)> progress_callback = nullptr);

    // 异步批量轨迹优化（带字符串回调，回调在后台线程执行）
    AsyncOptimizeTask optimizeTrajectoryAsync(
        const std::vector<Eigen::Matrix4d>& trajectory,
        std::function<void(const std::string&)> progress_callback = nullptr);

    // 从CSV同步优化（带字符串回调）
    std::vector<std::array<double, 8>> optimizeFromCSV(
        const std::string& csv_file,
        std::function<void(const std::string&)> progress_callback = nullptr);

    // 从CSV异步优化（带字符串回调）
    AsyncOptimizeTask optimizeFromCSVAsync(
        const std::string& csv_file,
        std::function<void(const std::string&)> progress_callback = nullptr);

        // 新增提供获取耗时的接口
    const std::vector<double>& getPerPointElapsedTimes() const { return per_point_elapsed_ms_; }

    // 新增清空耗时缓存函数
    void clearPerPointElapsedTimes() { per_point_elapsed_ms_.clear(); }

    static void exportJointTrajectory(
        const std::vector<std::array<double, 8>>& joint_trajectory,
        const std::string& filename = "optimized_joints.csv");

    // 热启动状态接口
    void setWarmupState(
        const std::array<double, 8>& joints);

    void clearWarmupState();
private:
    Eigen::Matrix4d warmupTargetPoses_;
    std::vector<Individual> warmupPopulation_;
    RobotArm::Solution warmupSolution;

    // 内部优化实现，支持异步取消和字符串进度回调
    std::vector<std::array<double, 8>> optimizeTrajectoryInternal(
        const std::vector<Eigen::Matrix4d>& trajectory,
        std::shared_ptr<std::atomic<bool>> cancel_flag,
        std::function<void(const std::string&)> progress_callback);
    std::vector<double> per_point_elapsed_ms_;

protected:
    //记录最后一次优化的fitness
    double last_best_fitness_ = INFINITY;

    // 计算适应度函数
    double evaluatePopulation(std::vector<Individual>& population,
        const RobotArm::Solution& ref_solution,
        const Eigen::Matrix4d& target,
        int gene_joint1, int gene_joint2);
    //计算适应度
    double calculateFitness(Individual& ind, const RobotArm::Solution& ref_solution,
        const Eigen::Matrix4d& target, int gene_joint1, int gene_joint2);
    //计算连续性
    double calcContinuityCost(const RobotArm::Solution& sol, const RobotArm::Solution& ref) const;
    //计算关节限位
    double calcJointLimitCost(const std::array<double, 8>& q) const;
    //计算速度
    void calcVelocities(RobotArm::Solution& sol, const std::array<double, 8>& ref_joints);
    //选择
    virtual std::vector<Individual> select(const std::vector<Individual>& population) { return std::vector<Individual>{}; };
    //选择最优解
    Individual getBestValidIndividual(const std::vector<Individual>& population);
    //获取有效解
    std::vector<Individual> getValidIndividuals(const std::vector<Individual>& population);
    //初始化种群
    std::vector<Individual> initializePopulation(
        const Eigen::Matrix4d& target_pose,
        const Eigen::Matrix4d& prev_target_pose,
        const RobotArm::Solution& prev_solution,
        const std::vector<Individual>& prev_population);
    //带半径的初始化种群
    Individual generateIndividualWithRadius(
        const std::array<double, 8>& ref_joints,
        double radius);
    //确定搜索半径
    double determineSearchRadius(
        const Eigen::Matrix4d& target,
        const std::array<double, 8>& ref_joints,
        double init_radius);
    //获取关节解
    void updateGeneJoints(const RobotArm::CoreParams& cp);
    //判断是否迁移
    std::vector<Individual> handleMigration(
        const std::vector<Individual>& prev_population,
        const Eigen::Matrix4d& target_pose,
        const Eigen::Matrix4d& prev_target_pose);
    //交叉变异
    virtual std::vector<Individual> reproduction(const std::vector<Individual>& selected) { return std::vector<Individual>{}; };

    //////////////实验测试
 public:
        // ============ 新增：参数空间扫描接口 ============
        struct ExperimentResult {
            int population_size;
            double migration_rate;
            double fitness;  // 单次结果
        };

        // 参数空间扫描：100×100 = 10000个参数组合
        std::vector<TrajectoryOptimizer::ExperimentResult>
            scanParameterSpace(
                const Eigen::Matrix4d& warmup_pose,      // 第一个位姿
                const Eigen::Matrix4d& test_pose,        // 第二个位姿
                const Eigen::Matrix4d& actual_test_pose, // 第三个位姿（实际测试用）
                int pop_size_divisions,
                int migration_rate_divisions,
                int timeout_ms,
                std::function<void(const std::string&)> progress);

        // 导出结果到CSV
        static void exportExperimentResults(
            const std::vector<ExperimentResult>& results,
            const std::string& filename = "experiment_results.csv");
        // 在TrajectoryOptimizer类里新增：
        static void exportJointTrajectoryWithTime(
            const std::vector<std::array<double, 8>>& joint_trajectory,
            const std::vector<double>& elapsed_times_ms,
            const std::string& filename = "optimized_joints_with_time.csv");

        private:
            double runSingleTest(
                const Eigen::Matrix4d& test_pose,
                const RobotArm::Solution& warmup_solution,
                const std::vector<Individual>& warmup_population,
                int timeout_ms = 5);
};

// 遗传算法实现
class GeneticOptimizer : public TrajectoryOptimizer {
public:
    using TrajectoryOptimizer::TrajectoryOptimizer;
private:
    //交叉
    void applySBX(const Individual& parent1, const Individual& parent2,
        Individual& child1, Individual& child2);
    //变异
    void applyMutation(Individual& ind);
    //选择
    std::vector<Individual> select(const std::vector<Individual>& population) override;
    //二次处理
    std::vector<Individual> reproduction(const std::vector<Individual>& selected) override;

};

class NSGAOptimizer : public TrajectoryOptimizer {
private:
    struct DominationInfo {
        int domination_count = 0;
        std::vector<int> dominated_solutions;
        int rank = 0;
        double crowding_distance = 0.0;
    };

public:
    using TrajectoryOptimizer::TrajectoryOptimizer;

    std::vector<Individual> select(const std::vector<Individual>& population) override;
    std::vector<Individual> reproduction(const std::vector<Individual>& selected) override;

private:
    void fastNonDominatedSort(std::vector<Individual>& population);
    void calculateCrowdingDistance(std::vector<Individual>& front);
    bool dominates(const Individual& a, const Individual& b);
    std::vector<double> calculateObjectives(const Individual& ind);
    void tournamentSelectionNSGA(const std::vector<Individual>& population,
        std::vector<Individual>& selected);
};
// MOEA/D 优化器
class MOEADOptimizer : public TrajectoryOptimizer {
private:
    struct WeightVector {
        std::vector<double> weights;
        std::vector<int> neighbors;
    };
 
public:
    using TrajectoryOptimizer::TrajectoryOptimizer;
 
    std::vector<Individual> select(const std::vector<Individual>& population) override;
    std::vector<Individual> reproduction(const std::vector<Individual>& selected) override;
 
private:
    void initializeWeights();
    void findNeighbors();
    double scalarizeObjective(const Individual& ind, const WeightVector& weight);
    std::vector<double> calculateObjectives(const Individual& ind);
    void updateNeighbors(const Individual& offspring, int subproblem_idx);
    
private:
    std::vector<WeightVector> weight_vectors_;
    std::vector<Individual> ideal_point_;
    static constexpr int T = 20; // 邻域大小
    static constexpr double delta = 0.9; // 从邻域选择的概率
};
 
// ISGA-BT 优化器
class ISGAOptimizer : public TrajectoryOptimizer {
private:
    struct IslandInfo {
        std::vector<Individual> population;
        int migration_generation = 0;
    };
 
public:
    using TrajectoryOptimizer::TrajectoryOptimizer;
 
    std::vector<Individual> select(const std::vector<Individual>& population) override;
    std::vector<Individual> reproduction(const std::vector<Individual>& selected) override;
 
private:
    void initializeIslands(const std::vector<Individual>& population);
    void evolveIslands();
    void migration();
    void binaryTournament(const std::vector<Individual>& population, std::vector<Individual>& selected);
    void eliteSelection(std::vector<Individual>& population);
    
private:
    std::vector<IslandInfo> islands_;
    static constexpr int num_islands = 4;
    static constexpr int migration_interval = 10;
    static constexpr double migration_rate = 0.1;
};
 
// Differential Evolution 优化器
class DifferentialOptimizer : public TrajectoryOptimizer {
public:
    using TrajectoryOptimizer::TrajectoryOptimizer;
 
    std::vector<Individual> select(const std::vector<Individual>& population) override;
    std::vector<Individual> reproduction(const std::vector<Individual>& selected) override;
 
private:
    Individual mutateDE(const std::vector<Individual>& population, int target_idx);
    Individual crossoverDE(const Individual& target, const Individual& mutant);
    void boundaryConstraint(Individual& individual);
    
private:
    double F = 0.5;  // 变异因子
    double CR = 0.9; // 交叉概率
};
