// TrajectoryGA.h
#pragma once
#include "RobotArm.h"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <random>
#include <algorithm>
#include "CollisionChecker.h"
class TrajectoryGA {
    
public:

struct GAParams {
    // 成员变量声明（不进行类内初始化）
    int population_size;
    double migration_rate;
    int max_generations;
    double velocities;
    double crossover_rate;
    double mutation_rate;
    std::array<double, 8> joint_weights;
    std::array<double, 8> velo_weights;

    std::array<double, 8> joint_limits_weights;
    std::array<double, 2> task_weights;

    // 显式默认构造函数（统一初始化所有成员）
    GAParams() :
        population_size(200),
        migration_rate(0.9),
        max_generations(50),
        velocities(10.0),

        crossover_rate(0.6),
        mutation_rate(0.1),
        joint_weights({ {0.08, 0.08, 0.1, 1, 0.3, 0.2, 0.1, 0.1} }), // 双括号
        velo_weights({ {1, 1, 1, 0.5, 1, 0.2, 0.2, 0.2} }), // 双括号
        joint_limits_weights({{0.01, 0.01, 0.3, 0.1, 0.1, 0.1, 0.1, 0.1}}),
        task_weights({{0.8, 0.3}}) {}
};

    struct Individual {
        std::array<double, 8> theta;          
        std::array<double, 8> joints;

        double fitness = INFINITY;
        bool valid = false;

        bool operator<(const Individual& other) const {
            return fitness < other.fitness;
        }
    };


    explicit TrajectoryGA(RobotArm& arm,CollisionSystem& collision_system, const GAParams& params = GAParams());

    std::array<double, 8> optimize(
        const Eigen::Matrix4d& target_pose,
         Eigen::Matrix4d& prev_target_pose,
         RobotArm::Solution& prev_solution,
         std::vector<Individual>& prev_population);

    void SetNotContinuity() { IsContinuity = false; }
    void SetContinuity() { IsContinuity = true; }

private:
    RobotArm& arm_;
    CollisionSystem& collision_system_;
    GAParams params_;
    std::mt19937 rng_{ std::random_device{}() };
    // ����ʱ״̬
    std::vector<Individual> population_;
    Eigen::Matrix4d current_target_;
    double current_search_radius_;
    double search_radius = PI / 5.0;
    int gene_joint1 = 4;  // theta4 ������
    int gene_joint2 = 7;  // ��ʼ��Ϊ theta5���� optimize ��̬����
    RobotArm::JointLimits limits;
    bool IsContinuity=true;//calculate continuity
    // �������������뾶Լ���ĸ�������
    Individual generateIndividualWithRadius(
        const std::array<double, 8>& ref_joints,
        double radius);

    // ������������̬��������ȷ��
    double determineSearchRadius(
        const Eigen::Matrix4d& target,
        const std::array<double, 8>& ref_joints,
        double init_radius);

    // ���̶�ѡ��ʵ��
    std::vector<Individual> rouletteWheelSelection(
        const std::vector<Individual>& population);







    double evaluate_fitness(
        std::vector<Individual>& population,
        RobotArm::Solution& ref_joints);

    void applySBX(Individual& parent1, Individual& parent2, Individual& child1, Individual& child2);
    void applyMutation(Individual& ind);

    double calc_continuity_cost(
        RobotArm::Solution& sol,const RobotArm::Solution& ref) const;



    double calc_joint_limit_cost(
        const std::array<double, 8>& q) const;

    void calc_velo(RobotArm::Solution& sol, const std::array<double, 8>& ref_joints);

    /**
     * @brief 计算八自由度机械臂的可操作性度量
     * @param J 6x8的雅可比矩阵
     * @param sigma_m 控制曲线下降速率的参数
     * @return 可操作性度量值(0-1)
     */
    double calculateManipulability(const std::array<double, 8>& q, double sigma_m = 1.0);
};
