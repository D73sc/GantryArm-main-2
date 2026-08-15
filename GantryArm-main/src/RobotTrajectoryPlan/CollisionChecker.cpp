// File: CollisionUtils.cpp
#include "CollisionChecker.h"

CollisionSystem::CollisionSystem(const RobotArm& arm)
    : arm_(arm)
{
    	const std::array<double, 8> q_zero = {0,0,0,0,0,0,0,0};
    	zero_alltransforms = arm_.computeAllJointTransforms(q_zero); 

}

BoundingBox CollisionSystem::CreateCubeFromVertices(const std::array<std::array<float, 3>, 8> &vertices)
{
    // 计算各轴边界
    float min_x = vertices[0][0], max_x = vertices[0][0];
    float min_y = vertices[0][1], max_y = vertices[0][1];
    float min_z = vertices[0][2], max_z = vertices[0][2];

    for (const auto &v : vertices)
    {
        min_x = std::min(min_x, v[0]);
        max_x = std::max(max_x, v[0]);
        min_y = std::min(min_y, v[1]);
        max_y = std::max(max_y, v[1]);
        min_z = std::min(min_z, v[2]);
        max_z = std::max(max_z, v[2]);
    }

    // 构造包围盒
    BoundingBox box;
    box.shape = BoundingBox::CUBE;
    box.position = {
        (min_x + max_x) / 2.0f,
        (min_y + max_y) / 2.0f,
        (min_z + max_z) / 2.0f};
    box.cube_half_extents = {
        (max_x - min_x) / 2.0f,
        (max_y - min_y) / 2.0f,
        (max_z - min_z) / 2.0f};
    return box;
}

bool CollisionSystem::CheckAABBOBBCollision(const BoundingBox &aabb, const BoundingBox &obb) const
{
    // 获取AABB数据
    Eigen::Vector3f aabbPos(aabb.position.data());
    Eigen::Vector3f aabbExt(aabb.cube_half_extents.data());
    
    // 获取OBB数据
    Eigen::Vector3f obbPos = obb.center_global.cast<float>();
    Eigen::Vector3f obbExt(obb.cube_half_extents.data());
    Eigen::Matrix3f obbRot = obb.rotation_global.cast<float>();
    
    // 中心向量 (从AABB指向OBB)
    Eigen::Vector3f centerVec = obbPos - aabbPos;
    
    // 1. 测试AABB的3个轴
    for(int i = 0; i < 3; ++i) {
        Eigen::Vector3f axis = Eigen::Vector3f::Unit(i);
        float r = aabbExt[i] + 
                 obbExt[0] * std::abs(obbRot.col(0).dot(axis)) +
                 obbExt[1] * std::abs(obbRot.col(1).dot(axis)) +
                 obbExt[2] * std::abs(obbRot.col(2).dot(axis));
        
        if(std::abs(centerVec.dot(axis)) > r + 1e-6f) {
            return false;
        }
    }
    
    // 2. 测试OBB的3个轴
    for(int i = 0; i < 3; ++i) {
        Eigen::Vector3f axis = obbRot.col(i);
        float r = aabbExt[0] * std::abs(axis.x()) +
                 aabbExt[1] * std::abs(axis.y()) +
                 aabbExt[2] * std::abs(axis.z()) +
                 obbExt[i];
        
        if(std::abs(centerVec.dot(axis)) > r + 1e-6f) {
            return false;
        }
    }
    
    // 3. 测试9个叉积轴 (简化计算)
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            Eigen::Vector3f axis = Eigen::Vector3f::Unit(i).cross(obbRot.col(j));
            float len2 = axis.squaredNorm();
            if(len2 < 1e-12f) continue; // 平行轴跳过
            
            axis /= std::sqrt(len2); // 归一化
            
            // 计算投影半径 (简化公式)
            float r = 0.0f;
            for(int k = 0; k < 3; ++k) {
                r += aabbExt[k] * std::abs(Eigen::Vector3f::Unit(k).dot(axis));
                r += obbExt[k] * std::abs(obbRot.col(k).dot(axis));
            }
            
            if(std::abs(centerVec.dot(axis)) > r + 1e-6f) {
                return false;
            }
        }
    }
    
    return true;
}

bool CollisionSystem::CheckAABBOverlap(const Eigen::Vector3f &min1, const Eigen::Vector3f &max1, const Eigen::Vector3f &min2, const Eigen::Vector3f &max2) const
{
    // 计算两个AABB的尺寸
    const Eigen::Vector3f size1 = max1 - min1;
    const Eigen::Vector3f size2 = max2 - min2;
    
    // 检查各轴重叠，跳过长度>2的轴
    return ((size1.x() > 2.0f || size2.x() > 2.0f) || (max1.x() >= min2.x() && min1.x() <= max2.x())) &&
           ((size1.y() > 2.0f || size2.y() > 2.0f) || (max1.y() >= min2.y() && min1.y() <= max2.y())) &&
           ((size1.z() > 2.0f || size2.z() > 2.0f) || (max1.z() >= min2.z() && min1.z() <= max2.z()));
}

void CollisionSystem::LoadAABBFromCSV(const std::string &file_path, const std::array<float, 3> &color)
{
    auto points = LoadPoints(file_path);
    
    for(size_t i = 0; i < points.size(); i += 8) {
        std::array<std::array<float,3>,8> vertices;
        for(int j = 0; j < 8; j++) {
            vertices[j][0] = points[i + j][0];
            vertices[j][1] = points[i + j][1];
            vertices[j][2] = points[i + j][2];
        }
        
        BoundingBox box = CreateCubeFromVertices(vertices);
        box.object_type = BoundingBox::FIXED_BASE;
        box.color = color;
        
        // 预计算AABB边界
        Eigen::Map<const Eigen::Vector3f> pos(box.position.data());
        Eigen::Map<const Eigen::Vector3f> ext(box.cube_half_extents.data());
        box.aabb_min = pos - ext;
        box.aabb_max = pos + ext;
        // std::cout<<i<<": aabb_min:"<<box.aabb_min<<"aabb_max"<<box.aabb_max<<std::endl;
        m_aabbs.push_back(box);
    }
}

void CollisionSystem::LoadRobotOBBFromCSV(const std::string& csv_path) {
    auto all_points = LoadPoints(csv_path);
    const size_t group_count = all_points.size() / 8;

    for(int i = 0; i < group_count-1; i++) {
        std::array<Eigen::Vector3d, 8> vertices;
        const auto& base_T_alljoint = zero_alltransforms[i];
        
        Eigen::Matrix3d R_base_joint = base_T_alljoint.block<3,3>(0,0);
        Eigen::Vector3d t_base_joint = base_T_alljoint.block<3,1>(0,3);

        for(int j=0; j<8; j++) {
            Eigen::Vector3d v_world(
                all_points[8*i+j][0] * 1000.0 - t_base_joint.x(), 
                all_points[8*i+j][1] * 1000.0 - t_base_joint.y(),
                all_points[8*i+j][2] * 1000.0 - 2358 - t_base_joint.z()
            );
            vertices[j] = v_world;
        }

        // 计算OBB
        Eigen::Vector3d min = vertices[0];
        Eigen::Vector3d max = vertices[0];
        for(const auto& v : vertices){
            min = min.cwiseMin(v);
            max = max.cwiseMax(v);
        }

        BoundingBox box;
        box.shape = BoundingBox::CUBE;
        box.object_type = BoundingBox::ROBOT_LINK;
        box.center_local = (max + min) * 0.5;
        box.cube_half_extents = {
            static_cast<float>((max.x() - min.x()) * 0.0005),
            static_cast<float>((max.y() - min.y()) * 0.0005),
            static_cast<float>((max.z() - min.z()) * 0.0005)
        };
        box.rotation_local = R_base_joint.inverse();
        
        m_robot_boxes.push_back(box);
    }
}

void CollisionSystem::UpdateRobotOBBs(const std::array<double, 8>& q) {
    global_transforms = arm_.computeAllJointTransforms(q);
    auto revelative_transforms = arm_.computeAllJointTransforms(q);

    for(size_t joint_idx = 0; joint_idx < m_robot_boxes.size(); joint_idx++) {
        BoundingBox& box = m_robot_boxes[joint_idx];
        const Eigen::Matrix4d& base_T_joint = global_transforms[joint_idx];
        const Eigen::Matrix4d& base_T_revelativejoint = revelative_transforms[joint_idx];

        Eigen::Matrix3d R_base_joint = base_T_joint.block<3,3>(0,0);
        Eigen::Matrix3d R_base_revelativejoint = base_T_revelativejoint.block<3,3>(0,0);
        Eigen::Vector3d t_base_joint = base_T_joint.block<3,1>(0,3);
        
        Eigen::Vector3d center_base;
        if(joint_idx < 3) {
            box.rotation_global = Eigen::Matrix3d::Identity();
            center_base = box.center_local + t_base_joint;
        } else {
            box.rotation_global = R_base_joint * box.rotation_local;
            center_base = R_base_revelativejoint * box.center_local + t_base_joint;
        }

        box.center_global.x() = center_base.x() / 1000.0;
        box.center_global.y() = center_base.y() / 1000.0;
        box.center_global.z() = center_base.z() / 1000.0 + 2.358;
        
        // 更新OBB的AABB近似
        Eigen::Vector3f extents(
            box.cube_half_extents[0], 
            box.cube_half_extents[1], 
            box.cube_half_extents[2]
        );
        Eigen::Vector3f center = box.center_global.cast<float>();
        Eigen::Matrix3f rot = box.rotation_global.cast<float>();
        Eigen::Vector3f aabb_min = center;
        Eigen::Vector3f aabb_max = center;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float val = rot(i,j) * extents[j];
                if (val > 0) {
                    aabb_max(i) += val;
                    aabb_min(i) -= val;
                } else {
                    aabb_max(i) -= val;
                    aabb_min(i) += val;
                }
            }
        }

        box.aabb_min = aabb_min;
        box.aabb_max = aabb_max;
                // std::cout<<joint_idx<<": obb_min:"<<box.aabb_min<<"obb_max"<<box.aabb_max<<std::endl;

    }
}

bool CollisionSystem::CheckCollision(const std::array<double, 8>& q) {
    // 重置所有机械臂OBB颜色为默认灰色
    for(auto& box : m_robot_boxes) {
        box.color = {0.7f, 0.7f, 0.7f}; // 默认灰色
    }
    
    // 更新机械臂OBB状态
    UpdateRobotOBBs(q);
    
    bool collisionDetected = false;
    
    // 检查每个机械臂OBB与所有AABB的碰撞
    // 跳过前3个OBB
    for(size_t i = 3; i < m_robot_boxes.size(); i++) {
        auto& obb = m_robot_boxes[i]; // 注意这里改为非const引用
        
        for(const auto& aabb : m_aabbs) {
            // // // 快速AABB排除
            if(CheckAABBOverlap(aabb.aabb_min, aabb.aabb_max, 
                                obb.aabb_min, obb.aabb_max)) {
                // std::cout << "Collision detected with robot AABB " << i << std::endl;
            }
            else
                continue;

            
            // 精确检测
            if(CheckAABBOBBCollision(aabb, obb)) {
                // std::cout << "Collision detected with robot OBB " << i << std::endl;
                obb.color = {1.0f, 0.0f, 0.0f}; // 设置为红色
                collisionDetected = true;
                // 不立即返回，继续检查其他可能碰撞
                return collisionDetected;
            }
        }
    }
    
    return collisionDetected;
}


std::vector<std::array<float, 3>> CollisionSystem::LoadPoints(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("无法打开文件: " + file_path);
    }

    std::vector<std::array<float, 3>> points;
    std::string line;
    //跳过首行
    std::getline(file, line);
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string field;

        // 检查字段数量是否合法
        std::vector<std::string> fields;
        while (std::getline(ss, field, ',')) {
            // 去除前后空格
            field.erase(field.find_last_not_of(" \t\r\n") + 1);
            field.erase(0, field.find_first_not_of(" \t\r\n"));
            fields.push_back(field);
        }

        // 安全访问字段
        try
        {
            if (fields.size() < 3)
            {
                throw std::runtime_error("该行只有" + std::to_string(fields.size()) + "个字段，不足3个");
            }
            std::array<float, 3> point;
            point[0] = std::stof(fields[0]) / -1000.0f;
            point[1] = std::stof(fields[1]) / 1000.0f;
            point[2] = std::stof(fields[2]) / 1000.0f;
            points.push_back(point);
        }
        catch (const std::exception &e)
        {
            // 构造更详细的错误信息
            std::string error_msg = "行 '" + line + "' 转换失败: " + e.what();
            throw std::invalid_argument(error_msg);
        }
    }

    if (points.size() % 8 != 0)
    {
        throw std::invalid_argument("顶点总数必须是 8的倍数");
    }

    return points;
}






