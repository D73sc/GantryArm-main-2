// RobotArm.h
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <iostream>
#include <unordered_set>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <map> 
#include <chrono>

#include "robot_types.h"
constexpr double PI = 3.14159265358979323846;
using namespace Eigen;
using namespace std;

class  RobotArm {

public:
    struct MDHParam {
        double alpha;
        double a;
        double d_base;
        double theta_offset;//MDH theta offset
        double axis_offset;//axis value offset

        bool is_revolute; 
    };

    struct CoreParams {
        Eigen::Matrix4d T_;  
        Eigen::Vector3d a() const { return T_.col(2).head<3>(); }  // [ax, ay, az]
        Eigen::Vector3d o() const { return T_.col(1).head<3>(); }  // [ox, oy, oz]
        Eigen::Vector3d n() const { return T_.col(0).head<3>(); }  // [nx, ny, nz]
        Eigen::Vector3d p() const { return T_.col(3).head<3>(); }  // [px, py, pz]
        Eigen::Matrix3d rotation() const { return T_.block<3, 3>(0, 0); }

    };

    struct JointLimits {
        std::array<double, 8> min;
        std::array<double, 8> max;
    };

    struct Solution {
        std::array<double, 8> joints; // [d0,d1,d2,theta3-theta7]
        std::array<double, 8> velocities;
        double pos_error;
        double rot_error;
        bool valid=false;

        operator bool() const { return valid; }
    };

    struct SolutionDiversity {
        // 基本指标
        double success_rate;
        double mean_solutions;
        double cv;
        double concentration;
        double diversity_ratio;

        // 新增：时间统计
        double elapsed_time_ms;       // 耗时（毫秒）
        double time_per_point_ms;     // 每个搜索点的平均耗时（毫秒）
        double throughput;            // 吞吐量（点数/秒）

        string distribution;
    };

    explicit RobotArm(double load = 0.0)
    {
        initializeMDH();
        tcp_.x=0;
        tcp_.y=0;
        tcp_.z=load;
        tcp_.rx=0;
        tcp_.ry=0;
        tcp_.rz=0;
    }

    Eigen::Matrix4d forwardKinematics(const std::array<double, 8>& q) const;
    /**
     * @brief 计算工具坐标系的变换矩阵
     * @param end_effector_transform 末端执行器的齐次变换矩阵(4x4)
     * @param tool_offset 工具坐标系相对于末端执行器的偏移量(x,y,z)
     * @return 包含工具坐标系的完整变换矩阵
     */
    Eigen::Matrix4d computeToolTransform(
        const Eigen::Matrix4d& end_effector_transform,
        const Eigen::Vector3d& tool_offset);

    Eigen::Matrix4d getTCPTransform(const std::array<double, 8> &q);

    //获取自动方法使用45或者47
    string getAutoMethod(const CoreParams cp)const;
    //调用运动学逆解
    std::vector<Solution> inverseKinematics(
        const Matrix4d& T_target,
        const array<double, 2>& known_angles, const string& method = "auto") ;
    //解析目标矩阵中的参数
    CoreParams extractCoreParams(const Eigen::Matrix4d& T_goal) const;

    double aoz=1.5, anz=1.5;
    //计算雅各比矩阵
    Eigen::Matrix<double, 6, 8> computeJacobian(const std::array<double, 8>& q) const;
    const JointLimits& getJointLimits() const { return limits_; }
    //判断关节是否符合限位
    bool validateJoints(std::array<double, 8>& joints) ;

    // //读取单位为m的数据
    // static vector<Matrix4d> loadTrajectoryFromCSV(const string& filename);
    // //读取单位为mm的数据
    // static vector<Matrix4d> loadmmTrajectoryFromCSV(const string& filename);
    //读取关节角
    std::vector<std::array<double, 8>> readJointTrajectoryFromCSV(const std::string& filename);
    //网格化测试逆解
    SolutionDiversity  gridSearchInverseSolutions(
        const Matrix4d& T_goal,
        const string& method,
        int grid_steps = 100,
        const string& output_csv = "grid_solutions.csv",
        bool print_result = false
    );

    //计算所有的旋转矩阵的连乘
    std::vector<Eigen::Matrix4d> computeAllJointTransforms(
        const std::array<double, 8>& q
    ) const;
    //计算单个旋转矩阵
    std::vector<Eigen::Matrix4d> computeJointTransforms(
        const std::array<double, 8>& q
    ) const;

    /**
     * @brief 修改指定关节的MDH参数
     * @param joint_index 关节索引(0-7)
     * @param a 新的a参数(连杆长度)
     * @param alpha 新的alpha参数(连杆扭转角)，单位弧度
     * @param d 新的d参数(连杆偏移)
     * @param theta 新的theta参数(关节角度)，单位弧度
     * @throws std::out_of_range 如果关节索引无效
     */
    bool modifyMDHParam(int joint_index, double a, double alpha, double d, double theta);
    //重置DH
    void resetMDHParam() { initializeMDH(); }

    void setTcp(Point6D tcp){tcp_=tcp;}

    Point6D getTcp(){return tcp_;}

    // 单个变换矩阵：使用指定的TCP变换矩阵
    Eigen::Matrix4d tcpToFlange(const Eigen::Matrix4d& tcp_world_pose) {
        // 计算TCP到法兰盘的逆变换
        Eigen::Matrix4d flange_to_tcp_transform = toEigenMatrix(tcp_).inverse();

        // 将TCP世界位姿转换为变换矩阵
        Eigen::Matrix4d tcp_matrix = tcp_world_pose;

        // 计算法兰盘位姿：法兰盘 = TCP × (TCP到法兰盘的逆)
        Eigen::Matrix4d flange_matrix = tcp_matrix * flange_to_tcp_transform;

        return flange_matrix;
    }

    // 单个变换矩阵：使用指定的TCP变换矩阵
    Eigen::Matrix4d tcpToFlange(const Point6D& tcp_world_pose) {
        // 计算TCP到法兰盘的逆变换
        Eigen::Matrix4d flange_to_tcp_transform = toEigenMatrix(tcp_).inverse();

        // 将TCP世界位姿转换为变换矩阵
        Eigen::Matrix4d tcp_matrix = toEigenMatrix(tcp_world_pose);

        // 计算法兰盘位姿：法兰盘 = TCP × (TCP到法兰盘的逆)
        Eigen::Matrix4d flange_matrix = tcp_matrix * flange_to_tcp_transform;

        return flange_matrix;
    }

    // 批量转换：vector<Eigen::Matrix4d> 输入
    std::vector<Eigen::Matrix4d> tcpToFlange(const std::vector<Eigen::Matrix4d>& tcp_matrices) const {
        std::vector<Eigen::Matrix4d> flange_poses;
        flange_poses.reserve(tcp_matrices.size());

        Eigen::Matrix4d flange_to_tcp_transform = toEigenMatrix(tcp_).inverse();

        for (const auto& tcp_matrix : tcp_matrices) {
            Eigen::Matrix4d flange_matrix = tcp_matrix * flange_to_tcp_transform;
            flange_poses.push_back(flange_matrix);
        }

        return flange_poses;
    }

    // 批量转换：vector<Point6D> 输入
    std::vector<Eigen::Matrix4d> tcpToFlange(const std::vector<Point6D>& tcp_poses) const {
        std::vector<Eigen::Matrix4d> flange_poses;
        flange_poses.reserve(tcp_poses.size());

        Eigen::Matrix4d flange_to_tcp_transform = toEigenMatrix(tcp_).inverse();

        for (const auto& tcp_pose : tcp_poses) {
            Eigen::Matrix4d tcp_matrix = toEigenMatrix(tcp_pose);
            Eigen::Matrix4d flange_matrix = tcp_matrix * flange_to_tcp_transform;
            flange_poses.push_back(flange_matrix);
        }

        return flange_poses;
    }


private:
    std::array<MDHParam, 8> mdh_;
    Point6D tcp_;
    JointLimits limits_;
    double EPS_POS = 1e-2;  
    double EPS_ANG = 1e-2;  
    //初始化DH
    void initializeMDH();
    //MDH矩阵
    Eigen::Matrix4d computeTransform(const MDHParam& param,
        double joint_value) const;
    //关节角45的计算策略
    std::vector<Solution> solve45Strategy(
        const CoreParams& cp, double theta4, double theta5) ;
    //关节角47的计算策略
    std::vector<Solution> solve47Strategy(const CoreParams& cp,
        double theta4, double theta7) ;
    //已知45求关节6
    bool solveTheta45_6(double theta4, double theta5, double az,
        std::vector<double>& theta6) ;
    //已知45求关节7
    bool solveTheta45_7(double theta4, double theta5, const Eigen::Vector2d& oz_nz,
        std::vector<double>& theta7) ;
    //已知45求关节3
    bool solveTheta45_3(double theta4, double theta5, double theta7,
        const CoreParams& cp, std::vector<double>& theta3) ;
    //已知47求关节5
    bool solveTheta47_5(double theta4, double theta7, const Eigen::Vector2d& oz_nz, std::vector<double>& theta5) ;
    //验证解是否正确
    Solution validateSolution(
        const Eigen::Matrix4d& T_target,
         array<double, 8>& joints) ;
    //计算d0 d1 d2
    Eigen::Vector3d calcPrismatic(double theta3, double theta4, double theta5, double theta7,
        const CoreParams& cp) const;
    //转换到-PI到PI
    static double wrapToPi(double angle);

    //转换到0到2PI
    static double wrapTo2Pi(double angle);

    void addUniqueSolution(
        Solution&& new_sol,
        std::vector<Solution>& solution_list,
        std::unordered_set<std::string>& hash_set) const;

    std::string generateSolutionHash(const Solution& sol) const;

    bool solveTrigonometricEquation(const double A, const double B, const double C, std::vector<double>& solutions, double tolerance = 1e-12);

    bool solveTrigonometricEquationWrapped(const double A, const double B, const double C, std::vector<double>& solutions,int index, double tolerance = 1e-12);

};


