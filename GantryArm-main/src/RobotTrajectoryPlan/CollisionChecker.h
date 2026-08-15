// File: CollisionChecker.h
#pragma once
#include <vector>
#include <array>
#include <algorithm> // 用于min/max计算
#include <stdexcept>  // for std::invalid_argument
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <iostream>
#include "RobotArm.h"

struct  BoundingBox {
    
    enum ObjectType { FIXED_BASE, ROBOT_LINK };
    enum Shape { CUBE, CYLINDER };
    
    ObjectType object_type;
    Shape shape;
    std::array<float, 3> position;  // 包围盒中心坐标
    std::array<float, 4> rotation = {1.0f, 0.0f, 0.0f, 0.0f}; // 四元数
    std::array<float, 3> color = {0.7f, 0.7f, 0.7f}; // 默认灰色
    union {
        std::array<float, 3> cube_half_extents; // 立方体半尺寸
    };
    
    // 动态属性 (ROBOT_LINK时有效)
    Eigen::Vector3d center_local;
    Eigen::Matrix3d rotation_local;
    
    // 运行时计算的全局参数
    Eigen::Vector3d center_global;
    Eigen::Matrix3d rotation_global;
    
    // 预计算的AABB边界
    Eigen::Vector3f aabb_min;
    Eigen::Vector3f aabb_max;
};


class CollisionSystem {
   
public:
    // 构造函数
    explicit CollisionSystem(const RobotArm& arm);

    // 获取包围盒数据
    const std::vector<BoundingBox>& GetAABBs() const { return m_aabbs; }
    const std::vector<BoundingBox>& GetRobotOBBs() const { return m_robot_boxes; }


    const  std::vector<Eigen::Matrix4d>& GetFK() const { 
        return global_transforms; 
    }

    // 从CSV文件加载AABB包围盒
    void LoadAABBFromCSV(const std::string& file_path, 
                        const std::array<float,3>& color = {0.7f, 0.7f, 0.7f});

    // 从CSV文件加载机械臂OBB包围盒
    void LoadRobotOBBFromCSV(const std::string& csv_path);

    // 更新机械臂OBB状态
    void UpdateRobotOBBs(const std::array<double, 8>& q);

    // 主碰撞检测接口
    bool CheckCollision(const std::array<double, 8>& q);


private:
    const RobotArm& arm_;
    std::vector<Eigen::Matrix4d> zero_alltransforms;

    static std::vector<std::array<float,3>> LoadPoints(
        const std::string& file_path
    );

    // 从8个顶点创建立方体包围盒（Solidworks数据导入）
    BoundingBox CreateCubeFromVertices(
        const std::array<std::array<float,3>,8>& vertices
    );

    // 碰撞检测核心函数
    bool CheckAABBOBBCollision(const BoundingBox& aabb, const BoundingBox& obb) const;
    bool CheckAABBOverlap(const Eigen::Vector3f& min1, const Eigen::Vector3f& max1,
                         const Eigen::Vector3f& min2, const Eigen::Vector3f& max2) const;

    std::vector<BoundingBox> m_aabbs; // 所有碰撞体
    std::vector<BoundingBox> m_robot_boxes; // 所有机械臂碰撞体
    std::vector<Eigen::Matrix4d> global_transforms;

};
