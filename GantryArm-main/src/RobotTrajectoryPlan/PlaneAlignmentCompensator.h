#ifndef PLANE_ALIGNMENT_COMPENSATOR_H
#define PLANE_ALIGNMENT_COMPENSATOR_H

#include <Eigen/Dense>
#include <array>
#include <string>

class PlaneAlignmentCompensator {
public:
    PlaneAlignmentCompensator(double L = 54.0, bool debug_mode = false);
        
    // 设置调试模式
    void setDebugMode(bool enable);
    // 设置激光传感器位置（末端坐标系中）
    void setLaserPositions(const std::array<Eigen::Vector3d, 3>& positions);

    // 计算调整后的机械臂位姿
    Eigen::Matrix4d computeAdjustedPose(
        const Eigen::Matrix4d& current_pose,
        const Eigen::Vector3d& measurements,
        double target_value = 200.0);

    // 调试打印函数
    void debugPrint(const std::string& msg, const Eigen::Matrix4d& mat) const;
    void debugPrint(const std::string& msg, const Eigen::Vector3d& vec) const;
    void debugPrint(const std::string& msg, double value) const;

private:
    // 计算表面法向量
    Eigen::Vector3d computeSurfaceNormal(
        const Eigen::Vector3d& measurements,
        const Eigen::Matrix4d& end_effector_pose) const;

    // 重建旋转矩阵
    Eigen::Matrix3d reconstructRotationMatrix(
        const Eigen::Matrix4d& current_pose,
        const Eigen::Vector3d& surface_normal) const;

    std::array<Eigen::Vector3d, 3> laser_positions_;
    const double L_ = 54.0; // 传感器间距参数
    bool debug_mode_ = false;
};

#endif // PLANE_ALIGNMENT_COMPENSATOR_H
