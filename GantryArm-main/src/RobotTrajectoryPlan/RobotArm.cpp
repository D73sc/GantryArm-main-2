// RobotArm.cpp
#include "RobotArm.h"
#include <cmath>
#include <stdexcept>

constexpr double EPS = 1e-10;

Eigen::Matrix4d RobotArm::forwardKinematics(const std::array<double, 8>& q) const {
    Matrix4d T = Matrix4d::Identity();

    for (size_t i = 0; i < mdh_.size(); ++i) {
        T = T * computeTransform(mdh_[i], q[i]);
    }

    return T;
}

std::vector<Eigen::Matrix4d> RobotArm::computeJointTransforms(
    const std::array<double, 8>& q
) const {
    std::vector<Eigen::Matrix4d> jointTransforms;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < mdh_.size(); ++i) {
        T = computeTransform(mdh_[i], q[i]);
        jointTransforms.push_back(T); // 存储每个关节的变换矩阵
    }

    return jointTransforms;
}


std::vector<Eigen::Matrix4d> RobotArm::computeAllJointTransforms(
    const std::array<double, 8>& q
) const {
    std::vector<Eigen::Matrix4d> jointTransforms;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < mdh_.size(); ++i) {
        T = T * computeTransform(mdh_[i], q[i]);
        jointTransforms.push_back(T); // 存储累积的变换矩阵
    }

    return jointTransforms;
}

Eigen::Matrix4d RobotArm::computeToolTransform(const Eigen::Matrix4d& end_effector_transform, const Eigen::Vector3d& tool_offset)
{
    // 创建工具坐标系的局部变换矩阵
    Eigen::Matrix4d tool_local_transform = Eigen::Matrix4d::Identity();

    // 设置位置偏移部分
    tool_local_transform.block<3, 1>(0, 3) = tool_offset;

    // 将局部工具变换应用到末端执行器变换上
    // 注意乘法顺序：T_tool = T_end_effector * T_tool_local
    Eigen::Matrix4d tool_transform = end_effector_transform * tool_local_transform;

    return tool_transform;
}


Eigen::Matrix4d RobotArm::getTCPTransform(const std::array<double, 8>& q) {
    Eigen::Matrix4d T_end = forwardKinematics(q);
    Eigen::Vector3d tool_offset(tcp_.x, tcp_.y, tcp_.z);
    return computeToolTransform(T_end, tool_offset);  // 复用！
}

bool RobotArm::modifyMDHParam(int joint_index, double a, double alpha, double d, double theta)
{
    // 检查关节索引是否有效
    if (joint_index < 0 || joint_index >= mdh_.size())
    {
        return false;
    }

    // 修改指定关节的MDH参数
    mdh_[joint_index].a += a;
    mdh_[joint_index].alpha += alpha;
    if (mdh_[joint_index].is_revolute)
    {
        mdh_[joint_index].axis_offset += theta;
        mdh_[joint_index].d_base += d;
    }
    else
    {
        mdh_[joint_index].axis_offset += d;
    }

    return true;
}

string RobotArm::getAutoMethod(const RobotArm::CoreParams cp) const
{
    const double az = abs(cp.a().z());
    const double nz = abs(cp.n().z());
    const double oz = abs(cp.o().z());

    // 当 az 分量占主导时选择 47 策略，否则选择 45 策略
    string actual_method = (az > 1.6 * nz && az > 1.6 * oz) ? "47" : "45";
    return actual_method;
}

// vector<Matrix4d> RobotArm::loadTrajectoryFromCSV(const string& filename)
// {
//     vector<Matrix4d> trajectory;
//     ifstream file(filename);
//     string line;
//     // 跳过标题行
//     getline(file, line);

//     while (getline(file, line)) {
//         // 替换逗号分隔符为空格
//         replace(line.begin(), line.end(), ',', ' ');

//         istringstream iss(line);
//         double x, y, z, qx, qy, qz, qw;

//         // 解析CSV格式数据
//         if (iss >> x >> y >> z >> qx >> qy >> qz >> qw) {
//             trajectory.push_back(poseToTransformMatrix(x * 1000, y * 1000, z * 1000 - 2358, qx, qy, qz, qw));
//         }
//         else {
//             cerr << "格式错误: " << line << endl;
//         }
//     }

//     return trajectory;
// }

// vector<Matrix4d> RobotArm::loadmmTrajectoryFromCSV(const string& filename)
// {
//     vector<Matrix4d> trajectory;
//     ifstream file(filename);
//     string line;
//     // 跳过标题行
//     getline(file, line);

//     while (getline(file, line)) {
//         // 替换逗号分隔符为空格
//         replace(line.begin(), line.end(), ',', ' ');

//         istringstream iss(line);
//         double x, y, z, qx, qy, qz, qw;

//         // 解析CSV格式数据
//         if (iss >> x >> y >> z >> qx >> qy >> qz >> qw) {
//             trajectory.push_back(poseToTransformMatrix(x , y , z, qx, qy, qz, qw));
//         }
//         else {
//             cerr << "格式错误: " << line << endl;
//         }
//     }

//     return trajectory;
// }

std::vector<std::array<double, 8> > RobotArm::readJointTrajectoryFromCSV(const string& filename)
{
    std::vector<std::array<double, 8>> joint_trajectory;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return joint_trajectory;
    }

    std::string line;
    // Skip header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::array<double, 8> joints;

        // Read and discard the index column
        std::getline(iss, token, ',');

        // Read the 8 joint values
        for (int i = 0; i < 8; ++i) {
            if (!std::getline(iss, token, ',')) {
                std::cerr << "Error parsing line: " << line << std::endl;
                break;
            }
            try {
                joints[i] = std::stod(token);
            }
            catch (const std::exception& e) {
                std::cerr << "Error converting to double: " << token << std::endl;
                joints[i] = 0.0;
            }
        }

        joint_trajectory.push_back(joints);
    }

    return joint_trajectory;
}

RobotArm::SolutionDiversity RobotArm::gridSearchInverseSolutions(
    const Matrix4d& T_goal,
    const string& method,
    int grid_steps,
    const string& output_csv,
    bool print_result)
{
    // ========== 开始计时 ==========
    auto start_time = chrono::high_resolution_clock::now();

    double axis1_min, axis1_max, axis2_min, axis2_max;
    if (method == "auto")
    {
        axis1_min = 0 * PI / 180;
        axis1_max = 85 * PI / 180;
        axis2_min = -PI;
        axis2_max = PI;
    }
    else
    {
        int first_digit = method[0] - '0';
        axis1_min = limits_.min[first_digit];
        axis1_max = limits_.max[first_digit];

        int second_digit = method[1] - '0';
        axis2_min = limits_.min[second_digit];
        axis2_max = limits_.max[second_digit];
    }

    double axis1_step = (axis1_max - axis1_min) / grid_steps;
    double axis2_step = (axis2_max - axis2_min) / grid_steps;

    ofstream csv_file(output_csv);
    csv_file << "axis1,axis2,has_valid_solution,solutions_count\n";

    vector<int> solutions_per_point;
    int total_valid = 0;
    int total_solutions = 0;
    int total_search_points = grid_steps * grid_steps;

    for (int i = 1; i <= grid_steps; ++i) {
        const double axis1 = axis1_min + i * axis1_step;

        for (int j = 1; j <= grid_steps; ++j) {
            const double axis2 = axis2_min + j * axis2_step;

            array<double, 2> known_angles = { axis1, axis2 };
            const auto solutions = inverseKinematics(T_goal, known_angles, method);

            const bool has_valid = !solutions.empty();
            const int count = solutions.size();

            if (has_valid) {
                total_valid++;
                total_solutions += count;
                solutions_per_point.push_back(count);
            }

            csv_file << axis1 << "," << axis2 << "," << has_valid << "," << count << "\n";
        }
    }

    csv_file.close();

    // ========== 结束计时 ==========
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    double elapsed_ms = duration.count();

    SolutionDiversity diversity;
    diversity.success_rate = (total_valid / (double)total_search_points) * 100.0;
    diversity.elapsed_time_ms = elapsed_ms;
    diversity.time_per_point_ms = elapsed_ms / total_search_points;
    diversity.throughput = (total_search_points / (elapsed_ms / 1000.0));  // 点数/秒

    if (total_valid == 0) {
        diversity.mean_solutions = 0;
        diversity.cv = 0;
        diversity.concentration = 100.0;
        diversity.diversity_ratio = 0;
        diversity.distribution = "无解";
    }
    else {
        diversity.mean_solutions = total_solutions / (double)total_valid;

        double sum_sq_diff = 0;
        for (int sol : solutions_per_point) {
            double diff = sol - diversity.mean_solutions;
            sum_sq_diff += diff * diff;
        }
        double std_dev = sqrt(sum_sq_diff / total_valid);
        diversity.cv = (diversity.mean_solutions > 0.001) ?
            (std_dev / diversity.mean_solutions) * 100.0 : 0;

        sort(solutions_per_point.begin(), solutions_per_point.end());

        int count_1sol = 0;
        int count_2plus = 0;
        for (int sol : solutions_per_point) {
            if (sol == 1) count_1sol++;
            else if (sol >= 2) count_2plus++;
        }

        diversity.concentration = (count_1sol / (double)total_valid) * 100.0;
        diversity.diversity_ratio = (count_2plus / (double)total_valid) * 100.0;

        map<int, int> distribution_map;
        for (int sol : solutions_per_point) {
            distribution_map[sol]++;
        }

        stringstream ss;
        for (auto& pair : distribution_map) {
            int sol_count = pair.first;
            int frequency = pair.second;
            double percentage = (frequency / (double)total_valid) * 100.0;
            ss << sol_count << "解:" << fixed << setprecision(0) << percentage << "% ";
        }
        diversity.distribution = ss.str();
    }

    if (print_result) {
        cout << "\n【" << method << "方法】\n";
        cout << "  成功率:        " << fixed << setprecision(1) << diversity.success_rate << "%\n";
        cout << "  平均解数:      " << fixed << setprecision(2) << diversity.mean_solutions << "\n";
        cout << "  变异系数:      " << fixed << setprecision(1) << diversity.cv << "%";

        if (diversity.cv < 15) {
            cout << "  ✓ 分布均匀\n";
        }
        else if (diversity.cv < 30) {
            cout << "  ○ 分布一般\n";
        }
        else {
            cout << "  ✗ 分布不均\n";
        }

        cout << "  集中度分析:\n";
        cout << "    - 只有1个解: " << fixed << setprecision(1) << diversity.concentration << "%\n";
        cout << "    - 有2个以上解: " << fixed << setprecision(1) << diversity.diversity_ratio << "%\n";

        // ========== 新增：时间统计输出 ==========
        cout << "  性能统计:\n";
        cout << "    - 总耗时: " << fixed << setprecision(2) << diversity.elapsed_time_ms << " ms\n";
        cout << "    - 每个点: " << fixed << setprecision(4) << diversity.time_per_point_ms << " ms\n";
        cout << "    - 吞吐量: " << fixed << setprecision(0) << diversity.throughput << " 点/秒\n";
        // =========================================

        cout << "  分布:          " << diversity.distribution << "\n";
    }

    return diversity;
}
void RobotArm::initializeMDH()
{
    // 定义关节类型：
    // 前三轴为移动关节（平移关节，is_revolute=false）
    // 后五轴为旋转关节（is_revolute=true）
                //alpha     a   d           theta    axisOffset
    mdh_[0] = { PI / 2,    0,   0,         PI / 2,      0,  false }; // d0，移动轴
    mdh_[1] = { PI / 2,    0,   0,        -PI / 2,      0,  false }; // d1，移动轴
    mdh_[2] = { PI / 2,    0,   0,         PI / 2,       0,  false }; // d2，移动轴
    mdh_[3] = { 0,         0,   295.5,      0,          0,  true }; // 轴3，旋转轴
    mdh_[4] = { PI / 2,  200,   0,         PI / 2,      0,  true }; // 轴4，旋转轴
    mdh_[5] = { PI / 2,    0,   381,        PI,         0,  true }; // 轴5，旋转轴
    mdh_[6] = { PI / 2,    0,   0,          PI,         0,  true }; // 轴6，旋转轴
    mdh_[7] = { PI / 2,    0,   129, PI, 0,  true }; // 轴7，旋转轴

    // 初始化关节限位
    limits_.min = { -400, 0, 100,
                    0,      0 * PI / 180, -PI, -90 * PI / 180, -180 / 180.0 * PI };
    limits_.max = { 2700, 2000, 1800,
                   2 * PI, 85 * PI / 180, PI, 90 * PI / 180, 180 / 180.0 * PI };
}

Eigen::Matrix4d RobotArm::computeTransform(const MDHParam& param, double joint_value) const
{
    const double alpha = param.alpha;
    const double a = param.a;

    // 计算d和θ，根据关节类型
    const double d = param.is_revolute ?
        param.d_base :          // 旋转关节，d固定
        param.d_base + joint_value + param.axis_offset; // 移动关节，d含offset

    const double theta = param.is_revolute ?
        (joint_value + param.theta_offset + param.axis_offset) : // 旋转关节，θ含jointValue
        param.theta_offset;                   // 移动关节，θ固定

    Eigen::Matrix4d Ti;
    const double ct = cos(theta);
    const double st = sin(theta);
    const double ca = cos(alpha);
    const double sa = sin(alpha);
    Ti << ct, -st, 0, a,
        st* ca, ct* ca, -sa, -d * sa,
        st* sa, ct* sa, ca, d* ca,
        0, 0, 0, 1;

    return Ti;
}

vector<RobotArm::Solution> RobotArm::inverseKinematics(
    const Matrix4d& T_target,
    const array<double, 2>& known_angles, const string& method) 
{
    const RobotArm::CoreParams cp = extractCoreParams(T_target);
    vector<Solution> solutions;
    string actual_method = method;

    // 自动模式选择逻辑
    if (method == "auto") {
        actual_method = getAutoMethod(cp);
    }

    if (actual_method == "45") {
        solutions = solve45Strategy(cp, known_angles[0], known_angles[1]);
    }
    else if (actual_method == "47") {
        solutions = solve47Strategy(cp, known_angles[0], known_angles[1]);
    }
    else {
        throw invalid_argument("不支持的求解方法");
    }
    if (method == "auto")
        if (solutions.empty())
        {
            if (actual_method == "45")
                solutions = solve47Strategy(cp, known_angles[0], known_angles[1]);
            else if(actual_method == "47")
                solutions = solve45Strategy(cp, known_angles[0], known_angles[1]);
        }

    return solutions;
}
/**
 * @brief 无解条件：nz=oz=0或(c4*s5)^2>nz^2+oz^2;c5=s4=0或(az)^2>c5^2+s4^2;c5=s4=0或(-oyc7-nys7)^2>c5^2+(s4*s5)^2
 */
std::vector<RobotArm::Solution> RobotArm::solve45Strategy(
    const CoreParams& cp, double theta4, double theta5) 
{
    std::vector<Solution> solutions;

    // 获取所需参数（通过新接口访问）
    const double az = cp.a().z();
    const Vector2d oz_nz(cp.o().z(), cp.n().z());

    // 避免重复解，使用哈希判断
    std::unordered_set<std::string> solution_hashes; // 哈希记录表

    // 求解 theta6
    vector<double> theta6{};
    if (!solveTheta45_6(theta4, theta5, az, theta6)) {
        return solutions;
    }

    // 求解 theta7
    vector<double> theta7{};
    if (!solveTheta45_7(theta4, theta5, oz_nz, theta7)) {
        return solutions;
    }

    // 组合所有解
    for (auto t6 : theta6) {
        for (auto t7 : theta7) {
            // 求解 theta3
            vector<double> theta3{};
            if (!solveTheta45_3(theta4, theta5, t7, cp, theta3)) {

                return solutions;
            }
            for (auto t3 : theta3) {
                array<double, 8> joints{};

                joints[3] = t3;
                joints[4] = theta4;
                joints[5] = theta5;
                joints[6] = t6;
                joints[7] = t7;

                // 计算平移关节（使用新的 p()接口）
                auto d = calcPrismatic(t3, theta4, theta5, t7, cp);
                joints[0] = d[0];
                joints[1] = d[1];
                joints[2] = d[2];

                // 组装并验证解并添加
                addUniqueSolution(
                    validateSolution(cp.T_, joints),
                    solutions,
                    solution_hashes
                );
            }
        }
    }

    return solutions;
}
/**
 * @brief 无解条件：c4=0或(-oz * c7 - nz * s7)^2>c4^2;c5=s4=0或(az)^2>c5^2+s4^2;c5=s4=0或(-oyc7-nys7)^2>c5^2+(s4*s5)^2
 */
std::vector<RobotArm::Solution> RobotArm::solve47Strategy(
    const CoreParams& cp, double theta4, double theta7) 
{
    std::vector<Solution> solutions;
    // 避免重复解，使用哈希判断
    std::unordered_set<std::string> solution_hashes; // 哈希记录表
    const double az = cp.a().z();
    const Vector2d oz_nz(cp.o().z(), cp.n().z());
    const Vector2d oy_ny(cp.o().y(), cp.n().y());

    // 求解 theta5
    vector<double> theta5;
    if (!solveTheta47_5(theta4, theta7, oz_nz, theta5)) {
        return solutions;
    }

    // 组合所有解
    for (auto t5 : theta5) {
        // 求解 theta6
        vector<double> theta6{};
        if (!solveTheta45_6(theta4, t5, az, theta6)) {
            return solutions;
        }
        for (auto t6 : theta6) {
            // 求解 theta3
            vector<double> theta3{};
            if (!solveTheta45_3(theta4, t5, theta7, cp, theta3)) {
                return solutions;
            }
            for (auto t3 : theta3) {
                array<double, 8> joints{};

                joints[3] = t3;
                joints[4] = theta4;
                joints[5] = t5;
                joints[6] = t6;
                joints[7] = theta7;

                // 计算平移关节（使用新的 p()接口）
                auto d = calcPrismatic(t3, theta4, t5, theta7, cp);
                joints[0] = d[0];
                joints[1] = d[1];
                joints[2] = d[2];

                // 组装并验证解并添加
                addUniqueSolution(
                    validateSolution(cp.T_, joints),
                    solutions,
                    solution_hashes
                );
            }
        }
    }

    return solutions;
}

RobotArm::CoreParams RobotArm::extractCoreParams(const Eigen::Matrix4d& T_goal) const
{
    CoreParams cp;
    cp.T_ = T_goal;

    // 验证矩阵有效性
    if (std::abs(T_goal(3, 3) - 1.0) > 1e-6) {
        throw std::invalid_argument("齐次矩阵无效：最后一元素应为 1");
    }

    // 自动检查旋转矩阵验证（正交性和行列式范围）
    constexpr double eps = 1e-5;
    auto R = cp.rotation();
    if ((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() > eps) {
        throw std::runtime_error("旋转矩阵不正交");
    }

    return cp;
}

Eigen::Matrix<double, 6, 8> RobotArm::computeJacobian(const std::array<double, 8>& q) const
{
    Eigen::Matrix<double, 6, 8> J = Eigen::Matrix<double, 6, 8>::Zero();

    // 1. 计算各连杆的变换矩阵
    auto T = computeAllJointTransforms(q);

    // 2. 计算雅可比矩阵各列
    Eigen::Vector3d p_e = T[7].block<3, 1>(0, 3); // 末端位置

    for (int i = 0; i < 8; ++i) {
        if (mdh_[i].is_revolute) {
            // 旋转关节贡献
            Eigen::Vector3d z_i = T[i].block<3, 1>(0, 2); // z轴方向
            Eigen::Vector3d p_i = T[i].block<3, 1>(0, 3); // 关节位置

            // 线速度部分: z_i × (p_e - p_i)
            J.block<3, 1>(0, i) = z_i.cross(p_e - p_i);

            // 角速度部分: z_i
            J.block<3, 1>(3, i) = z_i;
        }
        else {
            // 平移关节贡献
            Eigen::Vector3d z_i = T[i].block<3, 1>(0, 2);
            J.block<3, 1>(0, i) = z_i;
            J.block<3, 1>(3, i).setZero();
        }
    }

    return J;
}

/**
 * @brief c6s4+c4c5s6=-az。无解条件：c5=s4=0或(az)^2>c5^2+s4^2
 */
bool RobotArm::solveTheta45_6(double theta4, double theta5, double az,
    vector<double>& theta6)
{
    const double c4 = cos(theta4 + mdh_[4].axis_offset);
    const double s4 = sin(theta4 + mdh_[4].axis_offset);
    const double c5 = cos(theta5 + mdh_[5].axis_offset);

    //c6s4+c4c5s6=-az
    return solveTrigonometricEquationWrapped(c4*c5, s4, -az, theta6,6);

}
/**
 * @brief c4s5=-ozc7-nzs7。无解条件：nz=oz=0或(c4*s5)^2>nz^2+oz^2
 */
bool RobotArm::solveTheta45_7(double theta4, double theta5, const Eigen::Vector2d& oz_nz, std::vector<double>& theta7) 
{
    // 分解输入参数
    const double& oz = oz_nz[0];
    const double& nz = oz_nz[1];

    // Step 1: 预计算三角函数
    const double c4 = cos(theta4 + mdh_[4].axis_offset);
    const double s5 = sin(theta5 + mdh_[5].axis_offset);

    //c4s5=-ozc7-nzs7
    return solveTrigonometricEquationWrapped(-nz, -oz, c4*s5, theta7,7);

}
/**
 * @brief c4s5=-ozc7-nzs7。无解条件：c4=0或(-oz * c7 - nz * s7)^2>c4^2
 */
bool RobotArm::solveTheta47_5(double theta4, double theta7, const Eigen::Vector2d& oz_nz, std::vector<double>& theta5)
{
    // 分解输入参数
    const double& oz = oz_nz[0];
    const double& nz = oz_nz[1];

    // Step 1: 预计算三角函数
    const double c4 = cos(theta4 + mdh_[4].axis_offset);
    const double s7 = sin(theta7 + mdh_[7].axis_offset);
    const double c7 = cos(theta7 + mdh_[7].axis_offset);

    //c4s5=-ozc7-nzs7
    return solveTrigonometricEquationWrapped(c4, 0, -oz * c7 - nz * s7, theta5,5);

}
/**
 * @brief c3*c5-s3*s4*s5=-oyc7-nys7。无解条件：c5=s4=0和(-oyc7-nys7)^2>c5^2+(s4*s5)^2或(-oxc7-nxs7)^2>c5^2+(s4*s5)^2
 */
bool RobotArm::solveTheta45_3(double theta4, double theta5, double theta7, const CoreParams& cp, std::vector<double>& theta3)
{
    const double s4 = sin(theta4 + mdh_[4].axis_offset);
    const double s5 = sin(theta5 + mdh_[5].axis_offset);
    const double c5 = cos(theta5 + mdh_[5].axis_offset);
    const double s7 = sin(theta7 + mdh_[7].axis_offset);
    const double c7 = cos(theta7 + mdh_[7].axis_offset);
    //cout << "x:" << std::pow((-cp.o().x() * c7 - cp.n().x() * s7), 2);
    //cout << "y:" << std::pow((-cp.o().y() * c7 - cp.n().y() * s7), 2) <<endl;
    //bool ret;
    //if (std::pow((-cp.o().x() * c7 - cp.n().x() * s7),2) < std::pow((-cp.o().y() * c7 - cp.n().y() * s7),2))

    //    //s3*c5-c3*s4*s5=-oxc7-nxs7无解条件：c5=s4=0或(-oxc7-nxs7)^2>c5^2+(s4*s5)^2
        //ret= solveTrigonometricEquationWrapped( c5, -s4 * s5, -cp.o().x() * c7 - cp.n().x() * s7, theta3);

        //c3*c5-s3*s4*s5=-oyc7-nys7无解条件：c5=s4=0或(-oyc7-nys7)^2>c5^2+(s4*s5)^2
        //ret= solveTrigonometricEquationWrapped(-s4 * s5, c5, -cp.o().y() * c7 - cp.n().y() * s7, theta3);

    std::vector<double> theta3_all;
    std::vector<double> temp;

    bool ret1 = solveTrigonometricEquationWrapped(c5, -s4 * s5,
        -cp.o().x() * c7 - cp.n().x() * s7, temp,3);
    if (ret1) {
        theta3_all.insert(theta3_all.end(), temp.begin(), temp.end());
    }

    bool ret2 = solveTrigonometricEquationWrapped(-s4 * s5, c5,
        -cp.o().y() * c7 - cp.n().y() * s7, temp,3);
    if (ret2) {
        theta3_all.insert(theta3_all.end(), temp.begin(), temp.end());
    }
    for (int i = 0; i < theta3_all.size(); i++)
    {
        theta3_all[i] = wrapTo2Pi(theta3_all[i]);
    }
    // 简单去重
    std::sort(theta3_all.begin(), theta3_all.end());
    auto last = std::unique(theta3_all.begin(), theta3_all.end(),
        [](double a, double b) { return std::abs(a - b) < EPS; });
    theta3_all.erase(last, theta3_all.end());

    theta3 = theta3_all;
    return ret1 || ret2;
}

RobotArm::Solution RobotArm::validateSolution(const Eigen::Matrix4d& T_target, array<double, 8>& joints)
{
    Solution sol;
    sol.joints = joints;

    sol.valid = validateJoints(joints);  // 首先验证关节限位

    if (sol.valid) {       // 只在关节有效时进行计算
        const Matrix4d T_computed = forwardKinematics(joints);

        // 计算位置误差
        sol.pos_error = (T_computed.col(3).head<3>()
            - T_target.col(3).head<3>()).norm();

        // 计算旋转误差（轴角方法）
        Eigen::AngleAxisd aa(
            T_computed.block<3, 3>(0, 0).transpose()
            * T_target.block<3, 3>(0, 0)
        );
        sol.rot_error = aa.angle();

        // 检查有效性（位置和旋转同时满足）
        sol.valid = (sol.pos_error < EPS_POS)
            && (sol.rot_error < EPS_ANG);
    }

    return sol;
}

Eigen::Vector3d RobotArm::calcPrismatic(double theta3, double theta4, double theta5, double theta7, const CoreParams& cp) const
{
    // 分解目标参数 (根据 MDH 参数定义，需要调整这些变量对应关系)
    const double px = cp.p().x();  // 目标位置 x分量
    const double py = cp.p().y();  // 目标位置 y分量
    const double pz = cp.p().z();  // 目标位置 z分量

    const double ax = cp.a().x();  // 目标 z轴方向 x分量
    const double ay = cp.a().y();  // 目标 z轴方向 y分量
    const double az = cp.a().z();  // 目标 z轴方向 z分量

    const double nx = cp.n().x();  // 目标 x轴方向 x分量
    const double ny = cp.n().y();  // 目标 x轴方向 y分量
    const double nz = cp.n().z();  // 目标 x轴方向 z分量

    const double ox = cp.o().x();  // 目标 y轴方向 x分量
    const double oy = cp.o().y();  // 目标 y轴方向 y分量
    const double oz = cp.o().z();  // 目标 y轴方向 z分量

    // 预计算三角函数
    const double s3 = sin(theta3 + mdh_[3].axis_offset);
    const double c3 = cos(theta3 + mdh_[3].axis_offset);
    const double s4 = sin(theta4 + mdh_[4].axis_offset);
    const double c4 = cos(theta4 + mdh_[4].axis_offset);
    const double s5 = sin(theta5 + mdh_[5].axis_offset);
    const double c5 = cos(theta5 + mdh_[5].axis_offset);
    const double s7 = sin(theta7 + mdh_[7].axis_offset);
    const double c7 = cos(theta7 + mdh_[7].axis_offset);

    Eigen::Vector3d d;

    d[0] = -py + ay * (mdh_[7].d_base) - mdh_[7].a * ny * c7 + mdh_[7].a * s7 * oy
        + mdh_[2].a - mdh_[6].a * (c3 * s5 + c5 * s3 * s4) + mdh_[6].d_base * (c3 * c5 - s3 * s4 * s5)
        - s3 * (mdh_[4].a) + mdh_[4].d_base * c3 + mdh_[5].a * s3 * s4 - c4 * s3 * (mdh_[5].d_base)
        - mdh_[0].axis_offset;

    d[1] = px - ax * (mdh_[7].d_base) + mdh_[7].a * c7 * nx - mdh_[7].a * s7 * ox
        - mdh_[0].a - mdh_[3].a + mdh_[6].a * (s3 * s5 - c3 * c5 * s4)
        - mdh_[6].d_base * (c5 * s3 + c3 * s4 * s5) - mdh_[4].d_base * s3
        - c3 * (mdh_[4].a) + mdh_[5].a * c3 * s4
        - c3 * c4 * (mdh_[5].d_base)
        - mdh_[1].axis_offset;

    d[2] = -(pz - az * (mdh_[7].d_base) + mdh_[7].a * c7 * nz - mdh_[7].a * s7 * oz
        - mdh_[1].a + mdh_[3].d_base + s4 * (mdh_[5].d_base) + mdh_[5].a * c4
        - mdh_[6].a * c4 * c5 - mdh_[6].d_base * c4 * s5)
        - mdh_[2].axis_offset;

    return d;
}

// 验证关节范围
bool RobotArm::validateJoints(array<double, 8>& joints) {
    for (size_t i = 0; i < joints.size(); ++i) {
        if (joints[i] < limits_.min[i] ||
            joints[i] > limits_.max[i])
        {
            return false;
        }
    }
    return true;
}

// Matrix4d RobotArm::poseToTransformMatrix(double x, double y, double z, double qx, double qy, double qz, double qw)
// {
//     Matrix4d T = Matrix4d::Identity();

//     // 使用 Eigen 四元数类（注意构造函数参数顺序为 w,x,y,z）
//     Eigen::Quaterniond q(qw, qx, qy, qz);

//     if (q.norm() < 1e-6) {
//         throw std::invalid_argument("四元数模长过小");
//     }
//     q.normalize(); // 强制归一化

//     // 直接获取旋转矩阵
//     T.block<3, 3>(0, 0) = q.toRotationMatrix();

//     // 设置平移分量
//     T(0, 3) = x;
//     T(1, 3) = y;
//     T(2, 3) = z;

//     return T;
// }

// 角度函数
double RobotArm::wrapToPi(double angle) {
    angle = fmod(angle + PI, 2 * PI) - PI;
    return angle;
}

double RobotArm::wrapTo2Pi(double angle) {
    angle = fmod(angle, 2 * PI);
    if (angle < 0) {
        angle += 2 * PI;
    }
    return angle;
}

void RobotArm::addUniqueSolution(Solution&& new_sol, std::vector<Solution>& solution_list, std::unordered_set<std::string>& hash_set) const
{
    if (!new_sol.valid) return;

    // 生成唯一性哈希
    const std::string hash = generateSolutionHash(new_sol);

    // 仅在不存在相同解时添加
    if (hash_set.find(hash) == hash_set.end()) {
        solution_list.push_back(std::move(new_sol));
        hash_set.insert(hash);
    }
}

std::string RobotArm::generateSolutionHash(const Solution& sol) const
{
    std::string fingerprint;

    for (size_t i = 0; i < sol.joints.size(); ++i) {
        const double q = sol.joints[i];

        // 根据关节类型选择容差
        const double tol = (i < 3) ?
            RobotArm::EPS_POS :
            RobotArm::EPS_ANG;

        // 量化到一定精度
        const double normalized =
            std::round(q / tol) * tol;  // 四舍五入到容差级

        fingerprint += std::to_string(normalized) + "|";
    }

    return fingerprint;
}



bool RobotArm::solveTrigonometricEquation(const double A, const double B, const double C,
    std::vector<double>& solutions,
    double tolerance) {

    solutions.clear();

    // Step 1: 输入验证
    if (!std::isfinite(A) || !std::isfinite(B) || !std::isfinite(C)) {
        return false;
    }

    // Step 2: 计算 R = sqrt(A² + B²)
    const double R_sq = A * A + B * B;

    // Step 3: 检查系数奇异性 (A=B=0)
    if (R_sq < tolerance) {
        if (std::abs(C) < tolerance) {
            return false;  // 奇异性：无穷多解
        }
        else {
            return false;   // 无解
        }
    }

    const double R = std::sqrt(R_sq);

    // Step 4: 检查数学可行性 |C| ≤ R
    if (std::abs(C) > R + tolerance) {
        return false;  // 无解
    }

    // Step 5: 计算归一化值和cos分量
    const double C_normalized = C / R;
    const double cos_component_sq = 1.0 - C_normalized * C_normalized;

    // Step 6: 计算基准角度
    const double phi = std::atan2(B, A);  // 注意：A对应cos，B对应sin

    // Step 7: 处理不同情况
    if (cos_component_sq < tolerance) {
        // 边界奇异情况：只有一个解
        const double alpha = (C >= 0) ? PI / 2 : -PI / 2;
        solutions.push_back(alpha - phi);
        return true;  // 单解
    }
    else {
        // 正常情况：两个解
        const double cos_component = std::sqrt(cos_component_sq);

        const double alpha1 = std::atan2(C_normalized, cos_component);
        const double alpha2 = std::atan2(C_normalized, -cos_component);

        solutions.push_back(alpha1 - phi);
        solutions.push_back(alpha2 - phi);

        return true;  // 双解
    }
}

/**
 * @brief 求解三角方程 A*sin(x) + B*cos(x) = C。奇异性条件：A=B=0或C^2>A^2+B^2
 * @param A 系数A
 * @param B 系数B
 * @param C 常数项C
 * @param solutions 输出解数组，最多2个解
 * @param tolerance 数值容差，默认1e-12
 * @return int 返回值：0=无解，1=有解，-1=奇异性
 */
bool RobotArm::solveTrigonometricEquationWrapped(const double A,const double B,const double C,
    std::vector<double>& solutions, int index, double tolerance ) {

    bool result = solveTrigonometricEquation(A, B, C, solutions, tolerance);

    // 包装角度到[-π, π]
    for (double& sol : solutions) {
        sol -= mdh_[index].axis_offset;
        sol = std::atan2(std::sin(sol), std::cos(sol));
    }

    return result;
}
