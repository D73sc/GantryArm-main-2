#include "PlaneAlignmentCompensator.h"
#include <iostream>
#include <cmath>

PlaneAlignmentCompensator::PlaneAlignmentCompensator(double L,bool debug_mode)
    :L_(L), debug_mode_(debug_mode)
{
    // 初始化默认激光位置（等边三角形排列）
    // const double sqrt3 = std::sqrt(3.0);
    // laser_positions_ = {
    //     Eigen::Vector3d(40.37, -53.58, -100),          // 传感器1
    //     Eigen::Vector3d(-66.3, -5.18, -100),  // 传感器2
    //     Eigen::Vector3d(23.78,63.49, -100)  // 传感器3
    // };
    // laser_positions_ = {
    //     Eigen::Vector3d(-4.55, -60.59, 18.16),          // 传感器1
    //     Eigen::Vector3d(-50.19, 34.23, 18.16),  // 传感器2
    //     Eigen::Vector3d(54.74,26.35, 18.16)  // 传感器3
    // };
    laser_positions_ = {
        Eigen::Vector3d(-4.55, -60.59, -200),          // 传感器1
        Eigen::Vector3d(-50.19, 34.23, -200),  // 传感器2
        Eigen::Vector3d(54.74,26.35, -200)  // 传感器3
    };

}

void PlaneAlignmentCompensator::setLaserPositions(const std::array<Eigen::Vector3d, 3>& positions) {
    laser_positions_ = positions;
}

void PlaneAlignmentCompensator::setDebugMode(bool enable) {
    debug_mode_ = enable;
}


Eigen::Vector3d PlaneAlignmentCompensator::computeSurfaceNormal(
    const Eigen::Vector3d& measurements,
    const Eigen::Matrix4d& end_effector_pose) const
{
    if (debug_mode_) debugPrint("Current end effector pose:", end_effector_pose);

    // 获取末端坐标系Z轴在世界坐标系中的方向
    Eigen::Vector3d end_z_world = end_effector_pose.block<3, 1>(0, 2);
    if (debug_mode_) debugPrint("End effector Z axis:", end_z_world);

    // 将3D位置转换为齐次坐标（4D）
    Eigen::Vector4d p1_homog = laser_positions_[0].homogeneous();
    Eigen::Vector4d p2_homog = laser_positions_[1].homogeneous();
    Eigen::Vector4d p3_homog = laser_positions_[2].homogeneous();

    // 执行矩阵乘法（4x4 * 4x1 = 4x1）
    Eigen::Vector4d p1_world_homog = end_effector_pose * p1_homog;
    Eigen::Vector4d p2_world_homog = end_effector_pose * p2_homog;
    Eigen::Vector4d p3_world_homog = end_effector_pose * p3_homog;

   
    // 转换回3D坐标
    Eigen::Vector3d p1_world = p1_world_homog.head<3>();
    Eigen::Vector3d p2_world = p2_world_homog.head<3>();
    Eigen::Vector3d p3_world = p3_world_homog.head<3>();

    if (debug_mode_) {
        debugPrint("Laser1 world position:", p1_world);
        debugPrint("Laser2 world position:", p2_world);
        debugPrint("Laser3 world position:", p3_world);
    }
    //变为p1的激光末端点
    p1_homog(2) += measurements[0];
    p2_homog(2) += measurements[1];
    p3_homog(2) += measurements[2];
    Eigen::Vector4d p1_z = p1_homog;
    Eigen::Vector4d p2_z = p2_homog;
    Eigen::Vector4d p3_z = p3_homog;

    // 执行矩阵乘法（4x4 * 4x1 = 4x1）
    Eigen::Vector4d p1z_world_homog = end_effector_pose * p1_z;
    Eigen::Vector4d p2z_world_homog = end_effector_pose * p2_z;
    Eigen::Vector4d p3z_world_homog = end_effector_pose * p3_z;
    // 计算实际接触点 (沿末端Z轴方向)
    Eigen::Vector3d contact1 = p1z_world_homog.head<3>();
    Eigen::Vector3d contact2 = p2z_world_homog.head<3>();
    Eigen::Vector3d contact3 = p3z_world_homog.head<3>();

    if (debug_mode_) {
        debugPrint("Contact point 1:", contact1);
        debugPrint("Contact point 2:", contact2);
        debugPrint("Contact point 3:", contact3);
    }

    // 计算两个平面向量
    Eigen::Vector3d v1 = contact2 - contact1;
    Eigen::Vector3d v2 = contact3 - contact1;

    if (debug_mode_) {
        debugPrint("Plane vector 1 (v1):", v1);
        debugPrint("Plane vector 2 (v2):", v2);
    }

    // 计算法向量 (归一化)
    Eigen::Vector3d normal = v1.cross(v2).normalized();

    // 确保法向量指向机械臂末端坐标系Z轴的正方向
    if (normal.dot(end_z_world) < 0) {
        normal = -normal;
    }

    if (debug_mode_) debugPrint("Computed surface normal:", normal);

    return normal;
}

Eigen::Matrix3d PlaneAlignmentCompensator::reconstructRotationMatrix(
    const Eigen::Matrix4d& current_pose,
    const Eigen::Vector3d& surface_normal) const
{
    // 提取当前末端坐标系X轴和Y轴在世界坐标系中的方向
    Eigen::Vector3d current_x = current_pose.block<3, 1>(0, 0);
    Eigen::Vector3d current_y = current_pose.block<3, 1>(0, 1);
    if (debug_mode_) {
        debugPrint("Current end effector X axis:", current_x);
        debugPrint("Current end effector Y axis:", current_y);
    }

    // 新的Z轴就是法向量方向
    Eigen::Vector3d new_z = surface_normal.normalized();

    // 方法1: 先尝试保留X轴方向
    // 计算X轴在法平面上的投影(去除Z方向分量)
    Eigen::Vector3d new_x = current_x - current_x.dot(new_z) * new_z;

    // 如果投影太小(几乎与法线平行)，则使用原始Y轴作为基准
    if (new_x.norm() < 1e-6) {
        new_x = current_y - current_y.dot(new_z) * new_z;
        if (new_x.norm() < 1e-6) {
            // 如果还是太小，则任意选择一个与Z轴正交的向量
            new_x = Eigen::Vector3d::UnitX();
            new_x = new_x - new_x.dot(new_z) * new_z;
            if (new_x.norm() < 1e-6) {
                new_x = Eigen::Vector3d::UnitY();
                new_x = new_x - new_x.dot(new_z) * new_z;
            }
        }
    }
    new_x.normalize();

    // 计算新的Y轴 (保证正交)
    Eigen::Vector3d new_y = new_z.cross(new_x).normalized();

    // 构造旋转矩阵
    Eigen::Matrix3d new_rotation;
    new_rotation.col(0) = new_x;
    new_rotation.col(1) = new_y;
    new_rotation.col(2) = new_z;

    if (debug_mode_) {
        Eigen::Matrix4d debug_matrix = Eigen::Matrix4d::Identity();
        debug_matrix.block<3, 3>(0, 0) = new_rotation;
        debugPrint("New rotation matrix:", debug_matrix);
    }

    return new_rotation;
}

Eigen::Matrix4d PlaneAlignmentCompensator::computeAdjustedPose(
    const Eigen::Matrix4d& current_pose,
    const Eigen::Vector3d& measurements,
    double target_value)
{
    if (debug_mode_) debugPrint("Input measurements:", measurements);

    // 1. 计算表面法向量
    Eigen::Vector3d normal = computeSurfaceNormal(measurements, current_pose);

    // 2. 重建旋转矩阵
    Eigen::Matrix3d new_rotation = reconstructRotationMatrix(current_pose, normal);

    // 3. 计算需要调整的位置
    // 获取当前末端坐标系Z轴方向
    Eigen::Vector3d end_z_world = current_pose.block<3, 1>(0, 2);

    // 计算三个激光当前接触点的平均高度差
    double avg_measurement = measurements.mean();
    double delta = target_value - avg_measurement;

    if (debug_mode_) {
        debugPrint("Average measurement:", avg_measurement);
        debugPrint("Delta adjustment:", delta);
    }

    // 4. 构造新位姿
    Eigen::Matrix4d new_pose = current_pose;
    new_pose.block<3, 3>(0, 0) = new_rotation;
    new_pose.block<3, 1>(0, 3) -= end_z_world * delta;

    if (debug_mode_) debugPrint("Adjusted pose:", new_pose);

    return new_pose;
}

void PlaneAlignmentCompensator::debugPrint(const std::string& msg, const Eigen::Matrix4d& mat) const {
    if (!debug_mode_) return;

    std::cout << "[DEBUG] " << msg << "\n";
    for (int i = 0; i < 4; ++i) {
        std::cout << "  [ ";
        for (int j = 0; j < 4; ++j) {
            std::cout << mat(i, j);
            if (j < 3) std::cout << ", ";
        }
        std::cout << " ]\n";
    }
}

void PlaneAlignmentCompensator::debugPrint(const std::string& msg, const Eigen::Vector3d& vec) const {
    if (!debug_mode_) return;

    std::cout << "[DEBUG] " << msg << " [ "
        << vec.x() << ", " << vec.y() << ", " << vec.z() << " ]\n";
}

void PlaneAlignmentCompensator::debugPrint(const std::string& msg, double value) const {
    if (!debug_mode_) return;

    std::cout << "[DEBUG] " << msg << " " << value << "\n";
}
