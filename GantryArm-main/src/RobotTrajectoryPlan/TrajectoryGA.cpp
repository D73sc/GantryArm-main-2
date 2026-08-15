// TrajectoryGA.cpp
#include "TrajectoryGA.h"
#include <cmath>
#include <iostream>

TrajectoryGA::TrajectoryGA(RobotArm &arm, CollisionSystem &collision_system, const GAParams &params)
    : arm_(arm), collision_system_(collision_system), params_(params), rng_(std::random_device{}()) {}

std::array<double, 8> TrajectoryGA::optimize(
    const Eigen::Matrix4d &target_pose,
    Eigen::Matrix4d &prev_target_pose, // ������������һĿ��λ��
    RobotArm::Solution &prev_solution,
    std::vector<Individual> &prev_population) // ������������������뾶
{
    if (!prev_solution.valid)
    {
        prev_solution.joints = { 0,0,0,0,0,0,0,0 };
        prev_solution.velocities = { 0,0,0,0,0,0,0,0 };
    }
    RobotArm::Solution solution = prev_solution;
    double bestfitness;
    current_target_ = target_pose;
    current_search_radius_ = search_radius;
    const RobotArm::CoreParams cp = arm_.extractCoreParams(target_pose);
    // ȷ����������ַ���
    string actual_method = arm_.getAutoMethod(cp);
    if (actual_method == "45")
    {
        gene_joint2 = 5;
    }
    else
        gene_joint2 = 7;

    // 1. ��Ⱥ��ʼ��
    std::vector<Individual> population;
    // ���û����һ���Ͳ��̳�
    if (!prev_population.empty())
    {
        // 0������ת�������
        const Eigen::Matrix3d prev_rot = prev_target_pose.block<3, 3>(0, 0);
        const Eigen::Matrix3d curr_rot = target_pose.block<3, 3>(0, 0);
        const double rotation_diff = (curr_rot - prev_rot).norm(); // ����������
        // ��̬Ǩ�Ʊ���
        double migration_rate;
        if (rotation_diff < 0.3)
        {
             //std::cout << "migration_rate" << rotation_diff << "\n";
            migration_rate = params_.migration_rate; // С�������Ǩ��
            // Ǩ����һ����Ⱥ
            const int migrant_num = params_.population_size * migration_rate;
            // std::copy_n(prev_population.begin(),
            //     std::min(migrant_num, (int)prev_population.size()),
            //     std::back_inserter(population));
            for (int i = 0; i < migrant_num; i++)
            {
                Individual migrant = prev_population[i];
                migrant.valid = false;      // ����Ϊ��Ч
                migrant.fitness = INFINITY; // ������Ӧ��
                migrant.joints.fill(0);     // ��չؽڽ�
                population.push_back(migrant);
            }
            // cout << "migration";
        }
    }

    // 1.2 ��̬ȷ����������
    const double final_radius = determineSearchRadius(target_pose, prev_solution.joints, current_search_radius_);
    // search_radius = final_radius; // �����������
    //  std::cout << final_radius<<"\n";
    //  1.3 �����¸���
    while (population.size() < params_.population_size)
    {
        Individual ind = generateIndividualWithRadius(prev_solution.joints, final_radius);
        population.push_back(ind);
    }
    // cout << "population";

    Individual best_it{};
    // 2. ����ѭ��
    for (int gen = 0; gen < params_.max_generations; gen++)
    {
        // 2.1 ��Ӧ������
        bestfitness = evaluate_fitness(population, prev_solution);
        //cout << "gen:" << gen << "bestfitness" << bestfitness << endl;
        // 第一步：筛选出所有 valid=true 的个体
        std::vector<Individual> valid_individuals;
        std::copy_if(population.begin(), population.end(), std::back_inserter(valid_individuals),
                     [](const Individual &ind)
                     { return ind.valid; });

        // 第二步：在有效个体中找 fitness 最小
        if (!valid_individuals.empty())
        {
            auto it = std::min_element(valid_individuals.begin(), valid_individuals.end(),
                                       [](const Individual &a, const Individual &b)
                                       { return a.fitness < b.fitness; });
            best_it = *it;
            //cout << best_it.fitness<<endl;
        }

        if (bestfitness < 0.15)
        {
            if (best_it.valid)
            { // ȷ����Ч��Ÿ��½�
                solution.joints = best_it.joints;
                calc_velo(solution, prev_solution.joints);
                prev_solution = solution;
                prev_target_pose = target_pose;
                prev_population = population;
                // found_valid_solution = true;
                break;
            }
        }

        auto selected = rouletteWheelSelection(population);
        if (selected.empty())
        {
            population.clear();
            while (population.size() < params_.population_size)
            {
                Individual ind = generateIndividualWithRadius(prev_solution.joints, final_radius);
                population.push_back(ind);
            }
            continue;
        }

        std::vector<Individual> new_population;
        for (size_t i = 0; i < selected.size(); i += 2)
        {
            if (i + 1 >= selected.size())
            { // ����������һ������
                new_population.push_back(selected[i]);
                continue;
            }

            Individual child1, child2;
            applySBX(selected[i], selected[i + 1], child1, child2);
            applyMutation(child1);
            applyMutation(child2);
            new_population.push_back(child1);
            new_population.push_back(child2);
        }
        population = std::move(new_population);
    }

    // 2.1 ��Ӧ������
    bestfitness = evaluate_fitness(population, prev_solution);

    // 第一步：筛选出所有 valid=true 的个体
    std::vector<Individual> valid_individuals;
    std::copy_if(population.begin(), population.end(), std::back_inserter(valid_individuals),
                 [](const Individual &ind)
                 { return ind.valid; });

    // 第二步：在有效个体中找 fitness 最小
    if (!valid_individuals.empty())
    {
        auto it = std::min_element(valid_individuals.begin(), valid_individuals.end(),
                                   [](const Individual &a, const Individual &b)
                                   {
            if(b.valid)
                return a.fitness < b.fitness; });
        best_it = *it;
    }

        // ȷ����Ч��Ÿ��½�
        solution .joints= best_it.joints;
        solution.valid = true;
        calc_velo(solution, prev_solution.joints);
        prev_solution = solution;
        prev_target_pose = target_pose;
        prev_population = population;
        return solution.joints;

}

TrajectoryGA::Individual TrajectoryGA::generateIndividualWithRadius(const std::array<double, 8> &ref_joints, double radius)
{
    Individual ind;
    const auto &limits = arm_.getJointLimits();
    const int j1 = gene_joint1;
    const int j2 = gene_joint2;

    // 1. ����������λ�ĹؽڽǶȣ����� clamp��
    std::uniform_real_distribution<double> dist_radius(-radius, radius);

    // ����Ϸ����ɷ�Χ
    const double min_j1 = std::max(limits.min[j1], ref_joints[j1] - radius);
    const double max_j1 = std::min(limits.max[j1], ref_joints[j1] + radius);
    std::uniform_real_distribution<double> dist_j1(min_j1, max_j1);

    const double min_j2 = std::max(limits.min[j2], ref_joints[j2] - radius);
    const double max_j2 = std::min(limits.max[j2], ref_joints[j2] + radius);
    std::uniform_real_distribution<double> dist_j2(min_j2, max_j2);

    // ����������λ�Ĳ���ֵ
    ind.theta.fill(0.0);
    ind.theta[j1] = dist_j1(rng_);
    ind.theta[j2] = dist_j2(rng_);

    return ind;
}

// ������������̬��������ȷ��
double TrajectoryGA::determineSearchRadius(
    const Eigen::Matrix4d &target,
    const std::array<double, 8> &ref_joints,
    double init_radius)
{
    // 1. ��ȡ�ؽ���λ��Ϣ����λ�����ȣ�
    limits = arm_.getJointLimits();
    const int param_joint1 = gene_joint1; // �������ؽ�4����
    const int param_joint2 = gene_joint2; // �������ؽ�6����
    double limits_joint1 = (limits.max[param_joint1] - limits.min[param_joint1]) / 2.0;
    double limits_joint2 = (limits.max[param_joint1] - limits.min[param_joint1]) / 2.0;

    // 2. ������������ռ����Χ
    const double full_search_space = std::max(
        limits_joint1, limits_joint2);

    // 3. ��̬��������
    double best_radius = init_radius;
    int max_valid_count = 0;

    // 4. �ֽ׶ξ�������
    for (double s = init_radius; s <= full_search_space; s *= 1.5)
    {
        // 4.1 �������ʵ��������Χ
        const double axis1_range = std::min(s * limits_joint1 / (limits_joint1 + limits_joint2), limits_joint1);
        const double axis2_range = std::min(s * limits_joint2 / (limits_joint1 + limits_joint2), limits_joint2);

        // 4.2 Ԥ��������
        int valid_count = 0;
        for (int i = 0; i < 20; i++)
        {
            // �ھ��η�Χ�����������
            std::uniform_real_distribution<double> dist1(-axis1_range, axis1_range);
            std::uniform_real_distribution<double> dist2(-axis2_range, axis2_range);

            const double theta_joint1 = ref_joints[param_joint1] + dist1(rng_);
            const double theta_joint2 = ref_joints[param_joint2] + dist2(rng_);

            // �����֤
            auto solutions = arm_.inverseKinematics(target, {theta_joint1, theta_joint2});
            if (!solutions.empty())
                valid_count++;
        }

        // 4.3 ��̬�������ԣ�������Ч���ܶ�׼��
        if (valid_count * 1.2 > max_valid_count || max_valid_count == 0)
        {
            best_radius = s;
            max_valid_count = valid_count;
        }
        else
        {
            break; // ������������ֹͣ����
        }
    }

    return best_radius;
}

// ���̶�ѡ��ʵ��
std::vector<TrajectoryGA::Individual> TrajectoryGA::rouletteWheelSelection(
    const std::vector<TrajectoryGA::Individual> &population)
{
    std::vector<const TrajectoryGA::Individual *> valid_individuals;
    double max_fitness = -INFINITY;

    // 步骤 1: 预处理有效个体
    for (const auto &ind : population)
    {
        if (ind.valid)
        {
            valid_individuals.push_back(&ind);
            max_fitness = std::max(max_fitness, ind.fitness);
        }
    }

    // 如果没有有效个体，返回空列表（避免选择无效个体）
    if (valid_individuals.empty())
    {
        return {};
    }

    // 步骤 2: 计算适应度修正值和总概率
    double sum = 0.0;
    std::vector<double> probs;
    for (const auto ind : valid_individuals)
    {
        const double f_prime = max_fitness - ind->fitness + 1e-6; // 避免负值或零
        probs.push_back(f_prime);
        sum += f_prime;
    }

    // 步骤 3: 构建轮盘赌概率
    std::vector<double> wheel;
    double cum = 0.0;
    for (const auto p : probs)
    {
        cum += p / sum;
        wheel.push_back(cum);
    }

    // 步骤 4: 执行选择
    std::vector<TrajectoryGA::Individual> selected;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (size_t i = 0; i < population.size(); i++)
    {
        const double r = dist(TrajectoryGA::rng_);
        auto it = std::lower_bound(wheel.begin(), wheel.end(), r);
        const int index = std::distance(wheel.begin(),
                                        (it == wheel.end()) ? wheel.end() - 1 : it);
        selected.push_back(*valid_individuals[index]);
    }

    return selected;
}

double TrajectoryGA::evaluate_fitness(
    std::vector<Individual> &population,
    RobotArm::Solution &ref_joints)
{
    double best_fitness = INFINITY;

    for (auto &ind : population)
    {

        const int j1 = gene_joint1;
        const int j2 = gene_joint2;
        const std::array<double, 2> known_angles = {ind.theta[j1], ind.theta[j2]};

        // ��һ�����������˶�ѧ
        auto solutions = arm_.inverseKinematics(current_target_, known_angles);

        if (solutions.empty())
        {
            ind.valid = false;
            ind.fitness = INFINITY;
            continue;
        }
        ind.valid = true;

        // ����������Ч��
        double min_fitness = INFINITY;
        std::array<double, 8> best_joints;

        for (auto &sol : solutions)
        {
            //calculate velocities
            calc_velo(sol, ref_joints.joints);

            // ��Ŀ����Ӧ�ȼ���
            const double continuity = calc_continuity_cost(sol, ref_joints);

            //const double limits = calc_joint_limit_cost(sol.joints);
            
            const double fitness = params_.task_weights[0] * continuity;
                //+ params_.task_weights[1] * limits;

            if (fitness < min_fitness)
            {
                if (!collision_system_.CheckCollision(sol.joints))
                {

                        min_fitness = fitness;
                        best_joints = sol.joints;


                }
            }
        }

        // ���¸�����Ϣ
        if (min_fitness != INFINITY)
        {
            ind.valid=true;
            ind.fitness = min_fitness;
            ind.joints = best_joints;
            best_fitness = std::min(best_fitness, min_fitness);
        }
        else
        {
            ind.valid = false;
            ind.fitness = INFINITY;
        }
    }
     //cout<<best_fitness<<endl;
    return best_fitness;
}

// �������������㽻�棨SBX ���棩
void TrajectoryGA::applySBX(Individual &parent1, Individual &parent2, Individual &child1, Individual &child2)
{
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    const double eta_c = 20.0; // ���ƽ���ֲ���Խ���Ӵ�Խ�ӽ�������

    // ֻ�Բ������ؽڣ�theta4 ��thetaX�����н���
    for (int joint : {gene_joint1, gene_joint2})
    {
        double min = limits.min[joint];
        double max = limits.max[joint];

        // ��һ����[0,1]����
        double x1 = (parent1.theta[joint] - min) / (max - min);
        double x2 = (parent2.theta[joint] - min) / (max - min);

        if (dist(rng_) > params_.crossover_rate)
        {
            // ����������
            child1.theta[joint] = parent1.theta[joint];
            child2.theta[joint] = parent2.theta[joint];
            continue;
        }

        double beta;
        double u = dist(rng_);
        if (u <= 0.5)
            beta = std::pow(2 * u, 1.0 / (eta_c + 1));
        else
            beta = std::pow(0.5 / (1 - u), 1.0 / (eta_c + 1));

        // �����Ӵ��Ĺ�һ��ֵ��ȷ����Խ�磩
        double c1 = 0.5 * (x1 + x2 - beta * std::abs(x2 - x1));
        double c2 = 0.5 * (x1 + x2 + beta * std::abs(x2 - x1));
        c1 = std::max(0.0, std::min(1.0, c1)); // ȷ���ڹ�һ����Χ��
        c2 = std::max(0.0, std::min(1.0, c2));

        // ����һ��
        child1.theta[joint] = c1 * (max - min) + min;
        child2.theta[joint] = c2 * (max - min) + min;
    }
}

// �������������죨����λԼ���Ķ���ʽ���죩
void TrajectoryGA::applyMutation(Individual &ind)
{
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    const double eta_m = 20.0; // ���Ʊ���ǿ��

    for (int joint : {gene_joint1, gene_joint2})
    {
        if (dist(rng_) > params_.mutation_rate)
            continue;

        double min = limits.min[joint];
        double max = limits.max[joint];
        // ��һ����ǰֵ��[0,1]
        double x = (ind.theta[joint] - min) / (max - min);
        double delta = 0.0;

        if (dist(rng_) < 0.5)
        {
            delta = std::pow(2 * dist(rng_), 1.0 / (eta_m + 1)) - 1;
        }
        else
        {
            delta = 1 - std::pow(2 * (1 - dist(rng_)), 1.0 / (eta_m + 1));
        }

        // ���������ֵ������һ��
        x = x + delta;
        x = std::max(0.0, std::min(1.0, x)); // ȷ���ںϷ���Χ��
        ind.theta[joint] = x * (max - min) + min;
    }
}

double TrajectoryGA::calc_continuity_cost(
    RobotArm::Solution& sol, const RobotArm::Solution& ref) const
{
    std::array<double, 8> zeroArray = {};
    if (ref.joints == zeroArray)
    {
        return 0;
    }
    const double sigma_c = PI / 18;
    const double sigma_dc = 30;

    double cost = 0.0;
    
    for (int i = 0; i < 3; i++)
    {
        double delta = fabs(sol.joints[i] - ref.joints[i]);
        if(IsContinuity)    cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_dc, 2)));
        else    cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_dc, -2)));

    }
    for (int i = 3; i < 8; i++)
    {
        double delta = fabs(sol.joints[i] - ref.joints[i]);
        if(i==3)
        {
            if (IsContinuity)   cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / (sigma_c/10), 3)));
            else    cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_c, -2)));
        }
        else
        {
            if (IsContinuity)   cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_c, 2)));
            else    cost += params_.joint_weights[i] * (1.0 - exp(-pow(delta / sigma_c, -2)));
        }


    }

    const double delta =3;  // 速度死区阈值 (delta*10mm/s 度/s)3

    // 2.2 带死区的速度平滑性计算
    for (int i = 0; i < 8; i++)
    {

        double vel_diff = fabs(sol.velocities[i] - ref.velocities[i]);
        double vel_penalty = (std::max(0.0, vel_diff - delta));
        cost += params_.velo_weights[i] * vel_penalty;
    }

    return cost;
}





double TrajectoryGA::calc_joint_limit_cost(const std::array<double, 8> &q) const
{
    const auto &limits = arm_.getJointLimits();
    double cost = 0.0;
    for (int i = 0; i < 8; i++)
    {
        double safe_range = 0.2 * (limits.max[i] - limits.min[i]);
        cost += params_.joint_limits_weights[i] * (2.0 / (1.0 + exp((q[i] - limits.min[i]) / safe_range)) + 2.0 / (1.0 + exp(-(q[i] - limits.max[i]) / safe_range)));
    }
    return cost;
}

void TrajectoryGA::calc_velo(RobotArm::Solution& sol, const std::array<double, 8>& ref_joints)
{
    //calculate velocities
    double sum_sq = 0.0;
    for (int i = 0; i < 3; i++) {
        sum_sq += pow((sol.joints[i] - ref_joints[i])/10.0, 2);
    }
    for (int i = 3; i < 8; i++) {
        sum_sq += pow(sol.joints[i] - ref_joints[i], 2);
    }
    double T = sqrt(sum_sq) / params_.velocities;

    for (int i = 0; i < 3; i++) {
        sol.velocities[i] = (sol.joints[i] - ref_joints[i])/10.0 / T;
    }
    for (int i = 3; i < 8; i++) {
        sol.velocities[i] = (sol.joints[i] - ref_joints[i])  / T;
    }
}

double TrajectoryGA::calculateManipulability(const std::array<double, 8>& q, double sigma_m)
{
    sigma_m = 1000000;
    Eigen::Matrix<double, 6, 8> J = arm_.computeJacobian(q);
    std::cout << "Jacobian Matrix:\n" << J << std::endl;  // 打印雅可比矩阵
    // 计算JJᵀ
    Eigen::Matrix<double, 6, 6> JJT = J * J.transpose();

    // 使用PartialPivLU分解计算行列式（数值更稳定）
    Eigen::PartialPivLU<Eigen::Matrix<double, 6, 6>> lu(JJT);
    double det = lu.determinant();

    // 确保行列式非负
    det = std::max(det, 0.0);

    // 计算可操作性度量
    return std::exp(-std::sqrt(det) / sigma_m);
}

