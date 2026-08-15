#include "curve_processor_eigen.h"
#include <cmath>
#include <sstream>
#include <iostream>
using namespace std;

// 示例数据解析（根据实际CSV格式调整）
CurveProcessor::CurveProcessor(const string &csv_path)
{

    ifstream file(csv_path);
    if (!file.is_open())
    {
        cout << ("无法打开文件: " + csv_path);
    }

    string line;

    // Step 1: 跳过标题行
    getline(file, line);
    // Step 2: 加载实际数据
    while (getline(file, line))
    {
        stringstream ss(line);
        string field;

        // 检查字段数量是否合法
        vector<string> fields;
        while (getline(ss, field, ','))
        {
            fields.push_back(field);
        }
        if (fields.size() != 5)
        { // 包含Type和4个数值字段
            cerr << "警告：跳过无效数据行：" << line << endl;
            continue;
        }

        // Step 3: 解析各字段
        try
        {
            // Step 3: 解析坐标值（注意字段索引对应）
            Vector3d point;
            point.x() = stod(fields[1]) / 1000.0; // X在第二列
            point.y() = stod(fields[2]) / 1000.0; // Y在第三列，正序
            point.z() = stod(fields[3]) / 1000.0; // Z在第四列，正序
            // 解析SegmentID (第五列)
            int seg_id = stoi(fields[4]);

            // 存储到曲线数据
            _curve_data[seg_id].push_back(point);
        }
        catch (const exception &e)
        {
            cerr << "解析错误 (" << e.what() << ") : " << line << endl;
        }
    }
    computeCentroid();
    validateCurvePairs();
    computeMidpoints();
    computeFaceNormals();

    //// Step 4: 打印加载统计
    // cout << "\n===== 数据加载报告 =====" << endl;
    // cout << "找到的SegmentID数量: " << _curve_data.size() << endl;
    // for (const auto &[seg_id, points] : _curve_data)
    //{
    //     cout << " - Segment " << seg_id << ": "
    //          << points.size() << " 个点" << endl;

    //    // 可选：打印前三个点坐标验证
    //    for (size_t i = 0; i < (int)points.size(); ++i)
    //    {
    //        cout << "   Point " << i << ": ["
    //             << points[i].x() << ", "
    //             << points[i].y() << ", "
    //             << points[i].z() << "]" << endl;
    //    }
    //}
}

std::vector<Pose> CurveProcessor::generateMoveItTrajectory()
{
    std::vector<Pose> trajectory;

    // 直接按 pairs 的顺序遍历，确保输出顺序
    for (size_t face_id = 0; face_id < pairs.size(); ++face_id)
    {

        // 检查该面是否已计算（防止访问不存在的key）
        if (_face_data.find(face_id) == _face_data.end())
        {
            std::cerr << "警告：面 " << face_id << " 数据未初始化，跳过" << std::endl;
            continue;
        }

        const FaceData &face_data = _face_data[face_id];
        const vector<Vector3d> &midpoints = face_data.midpoints;
        const Vector3d &face_normal = face_data.normal;

        for (size_t i = 0; i < midpoints.size(); ++i)
        {
            Pose pose;
            const Vector3d &mp_pos = midpoints[i];
            pose.position = mp_pos;

            Vector3d tangent = computeTangent(midpoints, i, face_normal);
            pose.orientation = calculateOrientation(tangent, face_normal);

            trajectory.push_back(pose);
        }
    }
    return trajectory;
}

std::vector<Eigen::Matrix4d> CurveProcessor::generateIKTrajectory()
{
    std::vector<Eigen::Matrix4d> trajectory;

    // 获取位姿轨迹
    const std::vector<Pose> &trajectoryPose = generateMoveItTrajectory();

    // 预分配空间提高效率
    trajectory.reserve(trajectoryPose.size());

    // 转换参数（毫米单位转换和Z轴偏移）
    const double scale = 1000.0;     // 米到毫米转换
    const double z_offset = -2358.0; // 机械臂基座偏移

    for (const Pose &p : trajectoryPose)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // 构造并归一化四元数
        Eigen::Quaterniond q(p.orientation.w(),
                             p.orientation.x(),
                             p.orientation.y(),
                             p.orientation.z());
        q.normalize();

        // 设置旋转部分
        T.block<3, 3>(0, 0) = q.toRotationMatrix();

        // 设置平移部分（转换为毫米并应用Z轴偏移）
        T(0, 3) = p.position.x() * scale;
        T(1, 3) = p.position.y() * scale;
        T(2, 3) = p.position.z() * scale + z_offset;

        trajectory.push_back(T);
    }

    return trajectory;
}

std::vector<Eigen::Matrix4d> CurveProcessor::generateOrientInpIKTrajectory(std::string file)
{
    std::vector<Eigen::Matrix4d> trajectory;
    std::vector<Pose> Pose_trajectory;

    // 获取原始轨迹并检查有效性
    const std::vector<Pose> &trajectoryPose = generateMoveItTrajectory();
    if (trajectoryPose.empty())
    {
        return trajectory;
    }

    // 配置参数（可提取为类成员变量）
    const double mm_scale = 1000.0;    // 米到毫米转换
    const double z_offset = -2358.0;   // 机械臂基座偏移(mm)
    const double max_step_dist = 10.0; // 最大步长(mm)
    const double max_step_angle = 0.1; // 最大旋转步长(rad)
    const double min_interp_steps = 1; // 最小插值点数

    // 初始化第一个点
    auto createTransform = [&](const Pose &pose)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(pose.orientation.w(), pose.orientation.x(),
                             pose.orientation.y(), pose.orientation.z());
        q.normalize();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(
            pose.position.x() * mm_scale,
            pose.position.y() * mm_scale,
            pose.position.z() * mm_scale + z_offset);
        return T;
    };

    trajectory.push_back(createTransform(trajectoryPose[0]));
    Pose Pose0 = trajectoryPose[0];
    Pose0.position.x() *= mm_scale;
    Pose0.position.y() *= mm_scale;
    Pose0.position.z() *= mm_scale;
    Pose0.position.z() += z_offset;

    Pose_trajectory.push_back(Pose0);
    // 插值处理
    for (size_t i = 1; i < trajectoryPose.size(); ++i)
    {
        const Pose &prev = trajectoryPose[i - 1];
        const Pose &curr = trajectoryPose[i];

        // 计算位置和姿态差
        Eigen::Vector3d prev_pos(prev.position * mm_scale + Eigen::Vector3d(0, 0, z_offset));
        Eigen::Vector3d curr_pos(curr.position * mm_scale + Eigen::Vector3d(0, 0, z_offset));

        Eigen::Quaterniond prev_q(prev.orientation.w(), prev.orientation.x(),
                                  prev.orientation.y(), prev.orientation.z());
        Eigen::Quaterniond curr_q(curr.orientation.w(), curr.orientation.x(),
                                  curr.orientation.y(), curr.orientation.z());
        prev_q.normalize();
        curr_q.normalize();

        // 自适应插值步数
        double dist = (curr_pos - prev_pos).norm();
        double angle = prev_q.angularDistance(curr_q);

        int steps = std::max(
            static_cast<int>(dist / max_step_dist),
            static_cast<int>(angle / max_step_angle));
        steps = std::max(steps, static_cast<int>(min_interp_steps));
        // 执行插值
        for (int j = 1; j <= steps; ++j)
        {
            double t = static_cast<double>(j) / steps;

            // 位置线性插值
            Eigen::Vector3d interp_pos = prev_pos + t * (curr_pos - prev_pos);

            // 姿态球面线性插值
            Eigen::Quaterniond interp_q = prev_q.slerp(t, curr_q);
            Pose interp;
            interp.orientation = interp_q;
            interp.position = interp_pos;

            // 构建变换矩阵
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = interp_q.toRotationMatrix();
            T.block<3, 1>(0, 3) = interp_pos;
            Pose_trajectory.push_back(interp);
            trajectory.push_back(T);
        }
    }
    if (file != " ")
        savePosesToCSV(file, Pose_trajectory);

    return trajectory;
}

void CurveProcessor::computeCentroid()
{
    Vector3d sum = Vector3d::Zero();
    int total_points = 0;

    // 使用C++11兼容的方式遍历map
    for (const auto &curve_pair : _curve_data)
    {
        int seg_id = curve_pair.first;
        const vector<Vector3d> &points = curve_pair.second;

        for (const auto &pt : points)
        {
            sum += pt;
            total_points++;
        }
    }

    if (total_points == 0)
        throw runtime_error("无有效数据点");
    _centroid = sum / total_points;
}

void CurveProcessor::validateCurvePairs()
{
    for (const auto &pair : pairs)
    {
        int a = pair.first;
        int b = pair.second;
        if (_curve_data[a].size() != _curve_data[b].size())
        {
            cout << ("曲线对 " + std::to_string(static_cast<long long>(a)) +
                     "-" + std::to_string(static_cast<long long>(b)) +
                     " 点数不匹配");
        }
    }
}

void CurveProcessor::computeMidpoints()
{
    for (size_t face_id = 0; face_id < pairs.size(); ++face_id)
    {
        // 替换结构化绑定为C++11兼容方式
        int id1 = pairs[face_id].first;
        int id2 = pairs[face_id].second;

        const auto &curve1 = _curve_data[id1];
        const auto &curve2 = _curve_data[id2];

        vector<Vector3d> midpoints;
        for (size_t i = 0; i < curve1.size(); ++i)
        {
            midpoints.push_back((curve1[i] + curve2[i]) / 2.0);
        }
        _face_data[face_id].midpoints = midpoints;

        // 可选：打印前三个点坐标验证
        /*
        if (face_id == 0 && !midpoints.empty()) {  // 只打印第一个面的前三个点作为示例
            cout << "Face " << face_id << " 前三个中点坐标:" << endl;
            for (size_t i = 0; i < min((size_t)3, midpoints.size()); ++i) {
                cout << "  Point " << i << ": ["
                     << midpoints[i].x() << ", "
                     << midpoints[i].y() << ", "
                     << midpoints[i].z() << "]" << endl;
            }
        }
        */
    }
}

Vector3d CurveProcessor::computeTangent(
    const vector<Vector3d> &midpoints,
    size_t index,
    const Vector3d &face_normal // 接收当前面的法线！
)
{
    if (midpoints.size() < 2)
        return Vector3d::UnitX();

    // 原始切线估计（中心差分）
    Vector3d raw_tangent;
    if (index == 0)
    {
        raw_tangent = midpoints[1] - midpoints[0];
    }
    else if (index >= midpoints.size() - 1)
    {
        raw_tangent = midpoints.back() - midpoints[midpoints.size() - 2];
    }
    else
    {
        raw_tangent = (midpoints[index + 1] - midpoints[index]);
    }
    raw_tangent.normalized();

    // 投影到切平面
    Vector3d tangent = raw_tangent - raw_tangent.dot(face_normal) * face_normal;
    if (tangent.norm() < 1e-5)
    {
        return Vector3d::UnitX(); // 降级处理
    }
    return tangent.normalized();
}

void CurveProcessor::transformAllCurves()
{
    // 遍历所有曲线对
    for (auto &curve_pair : _curve_data)
    {
        // if(curve_pair.first < 1)
        //     continue; // 跳过第一个

        auto &points = curve_pair.second;

        // 变换曲线上的每个点
        for (auto &point : points)
        {
            Vector4d homo_point;
            homo_point << point, 1.0;
            Vector4d transformed = _transform_matrix * homo_point;
            point = transformed.head<3>();
        }
    }

    // 重新计算质心和法线等
    computeCentroid();
    computeFaceNormals();
    computeMidpoints();
}

Quaterniond CurveProcessor::calculateOrientation(
    const Vector3d &tangent, const Vector3d &normal)
{
    Vector3d T = tangent.normalized();
    Vector3d N = normal.normalized();

    // 强制正交化（确保T和N正交）
    if (fabs(N.dot(T)) > 1e-3)
    {
        T = T - N * N.dot(T);
        T.normalize();
    }

    // // 计算副法线B（Y轴）：必须满足右手定则 T × B = N
    // // 因为 X×Y=Z，所以 Y = Z×X = N×T
    Vector3d B = N.cross(T).normalized();

    Matrix3d rot_mat;
    rot_mat.col(0) = T; // X轴：轨迹切线方向（前进）
    rot_mat.col(1) = B; // Y轴：副法线（与TN组成右手系）
    rot_mat.col(2) = N; // Z轴：法线方向

    return Quaterniond(rot_mat).normalized();
}

void CurveProcessor::computeFaceNormals()
{
    for (size_t face_id = 0; face_id < pairs.size(); ++face_id)
    {
        // 替换结构化绑定为C++11兼容方式
        const int seg1 = pairs[face_id].first;
        const int seg2 = pairs[face_id].second;

        const vector<Vector3d> &curve1 = _curve_data.at(seg1);
        const vector<Vector3d> &curve2 = _curve_data.at(seg2);

        // 合并两侧曲线的点
        vector<Vector3d> all_points = curve1;
        all_points.insert(all_points.end(), curve2.begin(), curve2.end());

        if (all_points.size() < 3)
        {
            cerr << "警告：面" << face_id << "点数不足3，无法计算法线！使用默认Z轴。" << endl;
            _face_data[face_id].normal = Vector3d::UnitZ();
            continue;
        }

        // 根据点数选择方法
        Vector3d normal;
        if (all_points.size() == 3)
        {
            normal = computeNormalByThreePoints(all_points);
        }
        else
        {
            normal = computeNormalBySVD(all_points); // 推荐方法
        }

        // 法线方向验证与矫正
        Vector3d face_centroid = Vector3d::Zero();
        for (const auto &pt : all_points)
        {
            face_centroid += pt;
        }
        face_centroid /= all_points.size();

        Vector3d dir_outward = (face_centroid - _centroid).normalized();
        if (normal.dot(dir_outward) > 0)
        {
            normal = -normal;
        }

        // 存储到面数据
        _face_data[face_id].normal = normal.normalized();
    }
}

Vector3d CurveProcessor::computeNormalByThreePoints(const vector<Vector3d> &points)
{
    if (points.size() < 3)
    {
        cerr << "错误：至少需要3个点！返回默认法线Z轴." << endl;
        return Vector3d::UnitZ();
    }

    // 自动选取三个非共线点（改进：取最大间距三点）
    Vector3d P0 = points.front();
    Vector3d P1 = points[points.size() / 4]; // 中间点
    Vector3d P2 = points.back();

    // 如果末尾点共线，寻找其他候选点
    Vector3d v1 = (P1 - P0).normalized();
    Vector3d v2 = (P2 - P0).normalized();
    if (v1.cross(v2).norm() < 1e-5)
    { // 后三点共线，尝试其他点
        for (size_t i = 1; i < points.size() - 1; ++i)
        {
            P1 = points[i];
            v1 = (P1 - P0).normalized();
            v2 = (P2 - P0).normalized();
            if (v1.cross(v2).norm() > 1e-5)
                break;
        }
    }

    Vector3d normal = v1.cross(v2).normalized();
    return normal;
}

// ================ 强鲁棒性的平面法线计算方法 ================
Vector3d CurveProcessor::computeNormalBySVD(const vector<Vector3d> &points)
{
    const size_t n = points.size();
    if (n < 3)
        return Vector3d::UnitZ(); // 或抛出异常

    // (1) 计算点云质心并中心化数据
    Vector3d centroid(0, 0, 0);
    for (const auto &p : points)
        centroid += p;
    centroid /= n;

    MatrixXd centered(n, 3); // 行存储每个点与质心的偏差
    for (size_t i = 0; i < n; ++i)
    {
        centered.row(i) = points[i] - centroid;
    }

    // (2) 对中心化后的数据矩阵进行SVD分解
    JacobiSVD<MatrixXd> svd(centered, ComputeThinV);
    const Matrix3d &V = svd.matrixV();

    // (3) 法线为最小奇异值对应的右奇异向量（第3列，0-based索引2）
    Vector3d normal = V.col(2);
    normal.normalize(); // 单位化

    // (4) 法线方向矫正：确保all_points的平均投影在法线方向与质心-原点向量一致
    Vector3d outward = (centroid - _centroid).normalized();
    if (normal.dot(outward) < 0)
    {
        normal *= -1;
    }

    // (可选) 验证平面拟合度：计算点对平面的平均正交距离
    double sum_dist = 0.0;
    const double D = -normal.dot(centroid); // 平面方程为 normal·X + D = 0
    for (const auto &p : points)
    {
        sum_dist += abs(normal.dot(p) + D);
    }
    cout << "平均拟合误差: " << sum_dist / n << endl;

    return normal;
}

bool CurveProcessor::savePosesToCSV(const std::string &filename,
                                    const std::vector<Pose> &poses)
{
    ofstream file(filename);
    if (!file.is_open())
    {
        return false; // 文件打开失败（常见原因：路径权限/目录不存在）
    }

    // 写入CSV表头（描述各列含义）
    file << "x,y,z,qx,qy,qz,qw\n";

    // 设置数值格式：固定小数位 + 6位小数（ROS中常用精度）
    file << std::fixed << std::setprecision(6);

    // 遍历所有位姿点
    for (const auto &pose : poses)
    {
        const auto &p = pose.position;    // 位置信息
        const auto &q = pose.orientation; // 四元数方向

        // 写入一行数据，格式：x,y,z,qx,qy,qz,qw
        file << p.x() << ","
             << p.y() << ","
             << p.z() << ","
             << q.x() << ","
             << q.y() << ","
             << q.z() << ","
             << q.w() << "\n";
    }

    file.close();
    return true; // 导出成功
}

void CurveProcessor::loadCornerPoints()
{
    // 检查模型角点是否存在
    if (_curve_data.find(0) == _curve_data.end() || _curve_data[0].size() != 8)
    {
        throw runtime_error("模型角点未正确加载或不足8个点");
    }
    // 加载模型角点(从文件或直接赋值)
    _model_corners = _curve_data[0]; // 替换为你的模型角点数据
    // 检查现实角点
    if (_real_corners.size() != 8)
    {
        throw runtime_error("需要提供8个现实角点");
    }
    // 计算变换矩阵
    _transform_matrix = computeRigidTransform(_model_corners, _real_corners);
}

void CurveProcessor::loadCirclePoints()
{
    // 检查模型角点是否存在
    if (_curve_data.find(-1) == _curve_data.end() || _curve_data[-1].size() != 6)
    {
        throw runtime_error("模型圆点未正确加载或不足6个点");
    }
    // 加载模型角点(从文件或直接赋值)
    _model_circles = _curve_data[-1]; // 替换为你的模型角点数据

    // 计算变换矩阵
    _transform_matrix = computeRigidTransform(_model_circles, _real_circles);
}

// 设置现实角点
void CurveProcessor::setRealCorners(const vector<Vector3d> &corners)
{
    if (corners.size() != 8)
    {
        return;
        throw invalid_argument("必须提供8个角点");
    }
    _real_corners = corners;
    loadCornerPoints();
    // debugRigidTransformBidirectional();

    transformAllCurves();
}

void CurveProcessor::setRealCircles(const vector<Vector3d> &circles)
{

    _real_circles = circles;
    loadCirclePoints();
    transformAllCurves();
}
Matrix4d CurveProcessor::computeRigidTransform(const vector<Vector3d> &modelPoints, const vector<Vector3d> &realPoints)
{
    // 基本检查
    if (modelPoints.empty() || modelPoints.size() != realPoints.size())
    {
        cerr << "输入点云为空或大小不匹配" << endl;
        return Matrix4d::Identity();
    }

    // 收集有效点对
    vector<Vector3d> validModelPoints;
    vector<Vector3d> validRealPoints;

    for (size_t i = 0; i < modelPoints.size(); ++i)
    {
        if (!modelPoints[i].hasNaN() && !realPoints[i].hasNaN())
        {
            validModelPoints.push_back(modelPoints[i]);
            validRealPoints.push_back(realPoints[i]);
            cout << "点对 " << i << ": Model " << modelPoints[i].transpose()
                 << " -> Real " << realPoints[i].transpose() << endl;
        }
    }

    if (validModelPoints.size() < 3)
    {
        cerr << "有效点对数量不足: " << validModelPoints.size() << endl;
        return Matrix4d::Identity();
    }

    size_t n = validModelPoints.size();

    // 1. 计算质心
    Vector3d centroid_model = Vector3d::Zero();
    Vector3d centroid_real = Vector3d::Zero();
    for (size_t i = 0; i < n; ++i)
    {
        centroid_model += validModelPoints[i];
        centroid_real += validRealPoints[i];
    }
    centroid_model /= n;
    centroid_real /= n;

    // 2. 去质心化并构建协方差矩阵 H
    Matrix3d H = Matrix3d::Zero();
    for (size_t i = 0; i < n; ++i)
    {
        Vector3d p = validModelPoints[i] - centroid_model; // 源点
        Vector3d q = validRealPoints[i] - centroid_real;   // 目标点
        H += p * q.transpose();                            // 注意：p * q^T
    }

    // 3. SVD分解
    JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    // 4. 计算旋转矩阵 R = V * U^T
    Matrix3d R = V * U.transpose();

    // 5. 处理反射情况（保证det(R) = +1）
    if (R.determinant() < 0)
    {
        V.col(2) *= -1; // 翻转V的最后一列
        R = V * U.transpose();
    }

    // 6. 计算平移向量
    Vector3d t = centroid_real - R * centroid_model;

    // 7. 构造4x4变换矩阵
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    cout << "\n使用了 " << n << " 个点计算刚体变换" << endl;
    cout << "旋转矩阵行列式: " << R.determinant() << endl;

    // 8. 验证误差
    double total_error = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        Vector3d transformed = R * validModelPoints[i] + t;
        double error = (transformed - validRealPoints[i]).norm();
        total_error += error;
        cout << "点 " << i << " 误差: " << error << " mm" << endl;
    }
    cout << "平均误差: " << total_error / n << " mm\n"
         << endl;

    return T;
}

// Matrix4d CurveProcessor::computeRigidTransform(const vector<Vector3d> &modelPoints, const vector<Vector3d> &realPoints)
// {
//     // 检查输入基本条件
//     if (modelPoints.empty()) {
//         cerr << "输入点云为空" << endl;
//         return Matrix4d::Identity();
//     }

//     // 收集有效的点对
//     vector<Vector3d> validModelPoints;
//     vector<Vector3d> validRealPoints;
//     // 检查有效点的数量

//     for (size_t i = 0; i < modelPoints.size(); ++i) {
//         // 检查点是否有效（可以根据需要添加其他有效性检查）
//         if (!modelPoints[i].hasNaN() && !realPoints[i].hasNaN()) {
//             validModelPoints.push_back(modelPoints[i]);
//             validRealPoints.push_back(realPoints[i]);
//             // 【新增】调试信息：打印对应点对
//             cout << "点对 " << i << ": Model" << modelPoints[i].transpose()
//                  << " -> Real" << realPoints[i].transpose() << endl;

//         }
//     }
//     if (validModelPoints.size() < 3) {
//         cerr << "有效点对数量不足（至少需要3个点）: " << validModelPoints.size() << endl;
//         return Matrix4d::Identity();
//     }

//     // 计算质心
//     Vector3d centroid_model = Vector3d::Zero();
//     Vector3d centroid_real = Vector3d::Zero();

//     for (size_t i = 0; i < validModelPoints.size(); ++i) {
//         centroid_model += validModelPoints[i];
//         centroid_real += validRealPoints[i];
//     }
//     centroid_model /= validModelPoints.size();
//     centroid_real /= validRealPoints.size();

//     // 计算去质心坐标
//     MatrixXd H = MatrixXd::Zero(3, 3);
//     for (size_t i = 0; i < validModelPoints.size(); ++i) {
//         Vector3d p = validModelPoints[i] - centroid_model;
//         Vector3d q = validRealPoints[i] - centroid_real;
//         H += p * q.transpose();
//     }

//     // SVD分解
//     JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
//     Matrix3d U = svd.matrixU();
//     Matrix3d V = svd.matrixV();

//     // 计算旋转矩阵
//     Matrix3d R = V * U.transpose();

//     // 处理反射情况
//     if (R.determinant() < 0) {
//         V.col(2) *= -1;
//         R = V * U.transpose();
//     }

//     // 计算平移向量
//     Vector3d t = centroid_real - R * centroid_model;

//     // 构造4x4变换矩阵
//     Matrix4d T = Matrix4d::Identity();
//     T.block<3, 3>(0, 0) = R;
//     T.block<3, 1>(0, 3) = t;

//     cout << "使用了 " << validModelPoints.size() << " 个有效点对计算刚体变换" << endl;

//         // 【新增】计算并打印变换误差
//     double total_error = 0.0;
//     for (size_t i = 0; i < validModelPoints.size(); ++i) {
//         Vector4d model_homo;
//         model_homo << validModelPoints[i], 1.0;
//         Vector3d transformed = (T * model_homo).head<3>();
//         double error = (transformed - validRealPoints[i]).norm();
//         total_error += error;
//         cout << "点 " << i << " 误差: " << error << " mm" << endl;
//     }
//     cout << "平均误差: " << total_error / validModelPoints.size() << " mm\n" << endl;

//     return T;
// }

// ========== 侧面轨迹生成函数（添加间隔采样） ==========
// std::vector<Pose> CurveProcessor::generateSideFaceTrajectory(int point_interval) {
//     std::vector<Pose> side_trajectory;
//     // 确保间隔参数合法（最小为1，避免无效值）
//     point_interval = std::max(1, point_interval);
//     // 定义四个侧面的曲线对（上边:奇数，下边:偶数）
//     // const std::vector<std::pair<int, int>> side_pairs = { {9,10}, {11,12}, {13,14}, {15,16} };
//     const std::vector<std::pair<int, int>> side_pairs = {  {15,16} };

//     // 遍历每个侧面
//     for (const auto& pair : side_pairs) {
//         int upper_seg = pair.first;  // 上边编号（9/11/13/15）
//         int lower_seg = pair.second; // 下边编号（10/12/14/16）

//         // 检查曲线数据是否存在
//         if (_curve_data.find(upper_seg) == _curve_data.end() ||
//             _curve_data.find(lower_seg) == _curve_data.end()) {
//             std::cerr << "警告：侧面曲线对 " << upper_seg << "/" << lower_seg << " 数据不存在，跳过该面" << std::endl;
//             continue;
//         }

//         const auto& upper_points = _curve_data[upper_seg];  // 上边所有点
//         const auto& lower_points = _curve_data[lower_seg];  // 下边所有点

//         // 检查上下边是否为空
//         if (upper_points.empty() || lower_points.empty()) {
//             std::cerr << "警告：侧面曲线对 " << upper_seg << "/" << lower_seg << " 为空，跳过该面" << std::endl;
//             continue;
//         }

//         // 计算当前侧面的法线（复用原逻辑）
//         std::vector<Eigen::Vector3d> all_face_points = upper_points;
//         all_face_points.insert(all_face_points.end(), lower_points.begin(), lower_points.end());
//         Eigen::Vector3d face_normal;
//         if (all_face_points.size() == 3) {
//             face_normal = computeNormalByThreePoints(all_face_points);
//         }
//         else {
//             face_normal = computeNormalBySVD(all_face_points);
//         }
//         // 法线方向矫正（朝外）
//         Eigen::Vector3d face_centroid = Eigen::Vector3d::Zero();
//         for (const auto& pt : all_face_points) face_centroid += pt;
//         face_centroid /= all_face_points.size();
//         Eigen::Vector3d dir_outward = (face_centroid - _centroid).normalized();
//         if (face_normal.dot(dir_outward) > 0) {
//             face_normal = -face_normal;
//         }
//         face_normal.normalize();

//         // ========== 适配长短边 + 间隔采样核心逻辑 ==========
//         // 选择"向上"作为统一方向（也可以选择侧面主方向）
//         Eigen::Vector3d reference_tangent = (upper_points[0] - lower_points[0]).normalized();
//         reference_tangent = reference_tangent - reference_tangent.dot(face_normal) * face_normal;
//         reference_tangent.normalize();

//         const size_t upper_count = upper_points.size();
//         const size_t lower_count = lower_points.size();
//         const size_t short_len = std::min(upper_count, lower_count);   // 短边总点数
//         const bool is_upper_long = (upper_count > lower_count);        // 上边是否为长边

//         size_t short_last_idx = short_len - 1;  // 实际最后一个点
//         size_t short_last_sampled_idx = (short_len - 1) / point_interval * point_interval;  // 最后一个采样点

//         Eigen::Vector3d short_last_pt = is_upper_long ?
//                                             lower_points[short_last_idx] : upper_points[short_last_idx];

//         // ========== 阶段1：短边范围内（按间隔采样） ==========
//         for (size_t i = 0; i < short_len - point_interval; i += 2 * point_interval) {
//             size_t next_i = i + point_interval;

//             // 计算当前列的统一切线（向上）
//             Eigen::Vector3d tangent_i = (upper_points[i] - lower_points[i]).normalized();
//             tangent_i = tangent_i - tangent_i.dot(face_normal) * face_normal;
//             tangent_i.normalize();

//             Eigen::Vector3d tangent_next = (upper_points[next_i] - lower_points[next_i]).normalized();
//             tangent_next = tangent_next - tangent_next.dot(face_normal) * face_normal;
//             tangent_next.normalize();

//             // 生成四个点的轨迹
//             Pose lower_i, upper_i, upper_next, lower_next;
//             lower_i.position = lower_points[i];
//             lower_i.orientation = calculateOrientation(tangent_i, face_normal);

//             upper_i.position = upper_points[i];
//             upper_i.orientation = calculateOrientation(tangent_i, face_normal);

//             upper_next.position = upper_points[next_i];
//             upper_next.orientation = calculateOrientation(tangent_next, face_normal);

//             lower_next.position = lower_points[next_i];
//             lower_next.orientation = calculateOrientation(tangent_next, face_normal);

//             side_trajectory.push_back(lower_i);
//             side_trajectory.push_back(upper_i);
//             side_trajectory.push_back(upper_next);
//             side_trajectory.push_back(lower_next);
//         }

//         // ========== 关键修改2：补充短边最后一个点（如果未被采样） ==========
//         if (short_last_sampled_idx != short_last_idx) {
//             // 最后一个点的切线
//             Eigen::Vector3d tangent_last = (upper_points[short_last_idx] - lower_points[short_last_idx]).normalized();
//             tangent_last = tangent_last - tangent_last.dot(face_normal) * face_normal;
//             tangent_last.normalize();

//             // 从最后一个采样点到最后一个实际点
//             Eigen::Vector3d tangent_sampled = (upper_points[short_last_sampled_idx] - lower_points[short_last_sampled_idx]).normalized();
//             tangent_sampled = tangent_sampled - tangent_sampled.dot(face_normal) * face_normal;
//             tangent_sampled.normalize();

//             Pose lower_sampled, upper_sampled, upper_last, lower_last;

//             lower_sampled.position = lower_points[short_last_sampled_idx];
//             lower_sampled.orientation = calculateOrientation(tangent_sampled, face_normal);

//             upper_sampled.position = upper_points[short_last_sampled_idx];
//             upper_sampled.orientation = calculateOrientation(tangent_sampled, face_normal);

//             upper_last.position = upper_points[short_last_idx];
//             upper_last.orientation = calculateOrientation(tangent_last, face_normal);

//             lower_last.position = lower_points[short_last_idx];
//             lower_last.orientation = calculateOrientation(tangent_last, face_normal);

//             side_trajectory.push_back(lower_sampled);
//             side_trajectory.push_back(upper_sampled);
//             side_trajectory.push_back(upper_last);
//             side_trajectory.push_back(lower_last);
//         }

//         // ========== 阶段2：长边剩余部分 ==========
//         std::vector<Eigen::Vector3d> long_side_points = is_upper_long ? upper_points : lower_points;
//         std::vector<Eigen::Vector3d> long_continuous_pts;

//         // 从短边最后点对应的长边点开始
//         long_continuous_pts.push_back(is_upper_long ?
//                                           upper_points[short_last_idx] : lower_points[short_last_idx]);

//         // ========== 关键修改3：按间隔采样长边剩余点 ==========
//         for (size_t i = short_last_idx + point_interval; i < long_side_points.size(); i += point_interval) {
//             long_continuous_pts.push_back(long_side_points[i]);
//         }

//         // ========== 关键修改4：补充长边实际最后一个点 ==========
//         size_t long_last_idx = long_side_points.size() - 1;
//         size_t long_last_sampled_idx = short_last_idx +
//                                        ((long_last_idx - short_last_idx) / point_interval) * point_interval;

//         if (long_last_sampled_idx != long_last_idx && long_last_idx > short_last_idx) {
//             long_continuous_pts.push_back(long_side_points[long_last_idx]);
//         }

//         if (long_continuous_pts.size() < 2) continue;

//         // 计算短边最后点的统一切线
//         Eigen::Vector3d short_tangent = is_upper_long ?
//                                             (upper_points[short_last_idx] - lower_points[short_last_idx]).normalized() :
//                                             (upper_points[short_last_idx] - lower_points[short_last_idx]).normalized();
//         short_tangent = short_tangent - short_tangent.dot(face_normal) * face_normal;
//         short_tangent.normalize();

//         // 生成长边剩余部分轨迹
//         for (size_t j = 0; j < long_continuous_pts.size() - 1; ++j) {
//             Eigen::Vector3d prev_long_pt = long_continuous_pts[j];
//             Eigen::Vector3d curr_long_pt = long_continuous_pts[j + 1];

//             Pose p1, p2, p3, p4;
//             p1.position = short_last_pt;
//             p1.orientation = calculateOrientation(short_tangent, face_normal);

//             p2.position = prev_long_pt;
//             p2.orientation = calculateOrientation(short_tangent, face_normal);

//             p3.position = curr_long_pt;
//             p3.orientation = calculateOrientation(short_tangent, face_normal);

//             p4.position = short_last_pt;
//             p4.orientation = calculateOrientation(short_tangent, face_normal);

//             side_trajectory.push_back(p1);
//             side_trajectory.push_back(p2);
//             side_trajectory.push_back(p3);
//             side_trajectory.push_back(p4);
//         }
//     }

//     return side_trajectory;
// }
std::vector<Pose> CurveProcessor::generateSideFaceTrajectory(double interval_mm)
{
    std::vector<Pose> side_trajectory;
    interval_mm = std::max(0.1, interval_mm);
    double interval_m = interval_mm / 1000.0;
    double half_interval = interval_m / 2.0;

    for (const auto &pair : side_pairs)
    {
        int upper_seg = pair.first;
        int lower_seg = pair.second;

        if (_curve_data.find(upper_seg) == _curve_data.end() ||
            _curve_data.find(lower_seg) == _curve_data.end())
        {
            continue;
        }

        const auto &upper_points = _curve_data[upper_seg];
        const auto &lower_points = _curve_data[lower_seg];

        if (upper_points.empty() || lower_points.empty())
        {
            continue;
        }

        // ========== 计算面的法线 ==========
        std::vector<Eigen::Vector3d> all_face_points = upper_points;
        all_face_points.insert(all_face_points.end(), lower_points.begin(), lower_points.end());

        Eigen::Vector3d face_normal;
        if (all_face_points.size() == 3)
        {
            face_normal = computeNormalByThreePoints(all_face_points);
        }
        else
        {
            face_normal = computeNormalBySVD(all_face_points);
        }

        // ========== 确保法线指向外侧 ==========
        Eigen::Vector3d face_centroid = Eigen::Vector3d::Zero();
        for (const auto &pt : all_face_points)
            face_centroid += pt;
        face_centroid /= all_face_points.size();

        Eigen::Vector3d dir_outward = (face_centroid - _centroid).normalized();
        if (face_normal.dot(dir_outward) > 0)
        {
            face_normal = -face_normal;
        }
        face_normal.normalize();

        const size_t upper_count = upper_points.size();
        const size_t lower_count = lower_points.size();
        const size_t min_count = std::min(upper_count, lower_count);

        // ========== 用最外侧的距离作为参考 ==========
        double reference_distance = (lower_points[min_count - 1] - upper_points[min_count - 1]).norm();

        // ========== 确定采样高度 ==========
        std::vector<double> sample_heights;

        // 第一行：half_interval
        sample_heights.push_back(half_interval);

        // 中间行：每隔 interval_m
        double current_height = half_interval + interval_m;
        while (current_height < reference_distance - half_interval + 1e-6)
        {
            sample_heights.push_back(current_height);
            current_height += interval_m;
        }

        // 最后一行：确保距离下边至少 half_interval
        double last_height = reference_distance - half_interval;
        if (sample_heights.empty() ||
            std::abs(sample_heights.back() - last_height) > 1e-6)
        {
            sample_heights.push_back(last_height);
        }

        // ========== 计算统一的姿态（基于第一行的方向） ==========
        std::vector<Eigen::Vector3d> first_row_points;
        for (size_t curve_idx = 0; curve_idx < upper_count; ++curve_idx)
        {
            double pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
            double t = half_interval / pair_dist;
            t = std::clamp(t, 0.0, 1.0);

            Eigen::Vector3d pt = (1.0 - t) * upper_points[curve_idx] +
                                 t * lower_points[curve_idx];
            first_row_points.push_back(pt);
        }

        // 计算切线方向（从第一个点到最后一个点）
        Eigen::Vector3d tangent = (first_row_points.back() - first_row_points.front()).normalized();
        tangent = tangent - tangent.dot(face_normal) * face_normal;
        tangent.normalize();

        // 统一的姿态
        Eigen::Quaterniond unified_orientation = calculateOrientation(tangent, face_normal);

        // ========== 生成刷胶轨迹 ==========
        for (size_t h_idx = 0; h_idx < sample_heights.size(); ++h_idx)
        {
            double sample_height = sample_heights[h_idx];
            bool forward = (h_idx % 2 == 0); // 偶数索引从左到右

            // 生成该行的所有点
            std::vector<Eigen::Vector3d> row_points;
            for (size_t curve_idx = 0; curve_idx < upper_count; ++curve_idx)
            {
                double pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
                double t = sample_height / pair_dist;
                t = std::clamp(t, 0.0, 1.0);

                Eigen::Vector3d pt = (1.0 - t) * upper_points[curve_idx] +
                                     t * lower_points[curve_idx];
                row_points.push_back(pt);
            }

            // 添加该行的所有点（使用统一姿态）
            if (forward)
            {
                // 从左到右
                for (const auto &pt : row_points)
                {
                    Pose pose;
                    pose.position = pt;
                    pose.orientation = unified_orientation;
                    side_trajectory.push_back(pose);
                }
            }
            else
            {
                // 从右到左
                for (int i = row_points.size() - 1; i >= 0; --i)
                {
                    Pose pose;
                    pose.position = row_points[i];
                    pose.orientation = unified_orientation;
                    side_trajectory.push_back(pose);
                }
            }

            // ========== 行与行之间的连接轨迹 ==========
            if (h_idx < sample_heights.size() - 1)
            {
                // 从当前行的终点连接到下一行的起点
                Eigen::Vector3d current_row_end = forward ? row_points.back() : row_points.front();

                // 生成下一行的点
                double next_sample_height = sample_heights[h_idx + 1];
                bool next_forward = ((h_idx + 1) % 2 == 0);

                std::vector<Eigen::Vector3d> next_row_points;
                for (size_t curve_idx = 0; curve_idx < upper_count; ++curve_idx)
                {
                    double pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
                    double t = next_sample_height / pair_dist;
                    t = std::clamp(t, 0.0, 1.0);

                    Eigen::Vector3d pt = (1.0 - t) * upper_points[curve_idx] +
                                         t * lower_points[curve_idx];
                    next_row_points.push_back(pt);
                }

                Eigen::Vector3d next_row_start = next_forward ? next_row_points.front() : next_row_points.back();

                // 用第一个点对和最后一个点对的连线来连接
                Eigen::Vector3d connection_start = current_row_end;
                Eigen::Vector3d connection_end = next_row_start;

                // 线性插值连接
                int connection_points = 10; // 可调整连接点数
                for (int i = 0; i <= connection_points; ++i)
                {
                    double alpha = static_cast<double>(i) / connection_points;
                    Eigen::Vector3d connection_pt = (1.0 - alpha) * connection_start + alpha * connection_end;

                    Pose pose;
                    pose.position = connection_pt;
                    pose.orientation = unified_orientation;
                    side_trajectory.push_back(pose);
                }
            }
        }

// ========== 回程轨迹 ==========
        double last_row_height = sample_heights.back();
        bool last_row_forward = (sample_heights.size() - 1) % 2 == 0;

        std::vector<Eigen::Vector3d> last_row_points;
        for (size_t curve_idx = 0; curve_idx < upper_count; ++curve_idx)
        {
            double pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
            double t = last_row_height / pair_dist;
            t = std::clamp(t, 0.0, 1.0);

            Eigen::Vector3d pt = (1.0 - t) * upper_points[curve_idx] +
                                 t * lower_points[curve_idx];
            last_row_points.push_back(pt);
        }

        // 判断机械臂在哪一边
        // 如果最后一行是从右到左（forward=true），则终点在左边
        if (!last_row_forward)
        {
            // ========== 机械臂在右边：需要回程 ==========
            // 1. 反向走最后一行（从右回到左）
            for (int i = last_row_points.size() - 1; i >= 0; --i)
            {
                Pose pose;
                pose.position = last_row_points[i];
                pose.orientation = unified_orientation;
                side_trajectory.push_back(pose);
            }
        }
        // 如果在左边，不需要回程

        // 2. 向上走到 half_interval 高度
        std::vector<Eigen::Vector3d> upward_row_points;
        for (size_t curve_idx = 0; curve_idx < upper_count; ++curve_idx)
        {
            double pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
            double t = half_interval / pair_dist;
            t = std::clamp(t, 0.0, 1.0);

            Eigen::Vector3d pt = (1.0 - t) * upper_points[curve_idx] +
                                 t * lower_points[curve_idx];
            upward_row_points.push_back(pt);
        }
        Eigen::Vector3d upward_start;
        if (last_row_forward)
            // 向上走：从当前位置（左边）连接到第一行的起点
            upward_start = last_row_points.back();
        else
            upward_start = last_row_points.front();


        Eigen::Vector3d upward_end = first_row_points.back();

        // 线性插值连接
        int upward_connection_points = 10; // 可调整连接点数
        for (int i = 0; i <= upward_connection_points; ++i)
        {
            double alpha = static_cast<double>(i) / upward_connection_points;
            Eigen::Vector3d upward_pt = (1.0 - alpha) * upward_start + alpha * upward_end;

            Pose pose;
            pose.position = upward_pt;
            pose.orientation = unified_orientation;
            side_trajectory.push_back(pose);
        }
    }

    return side_trajectory;
}
// ========== 辅助函数：生成单行轨迹 ==========
std::vector<Pose> CurveProcessor::generateRowTrajectory(
    const std::vector<Eigen::Vector3d> &upper_points,
    const std::vector<Eigen::Vector3d> &lower_points,
    double sample_height,
    bool forward,
    const Eigen::Vector3d &face_normal)
{
    std::vector<Pose> row_trajectory;
    const size_t upper_count = upper_points.size();

    if (forward)
    {
        // ========== 从左到右 ==========
        for (size_t curve_idx = 0; curve_idx < upper_count - 1; ++curve_idx)
        {
            size_t next_idx = curve_idx + 1;

            // 当前列的插值点
            double curr_pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
            double t_curr = sample_height / curr_pair_dist;
            t_curr = std::clamp(t_curr, 0.0, 1.0);
            Eigen::Vector3d pt1 = (1.0 - t_curr) * upper_points[curve_idx] +
                                  t_curr * lower_points[curve_idx];

            // 下一列的插值点
            double next_pair_dist = (lower_points[next_idx] - upper_points[next_idx]).norm();
            double t_next = sample_height / next_pair_dist;
            t_next = std::clamp(t_next, 0.0, 1.0);
            Eigen::Vector3d pt2 = (1.0 - t_next) * upper_points[next_idx] +
                                  t_next * lower_points[next_idx];

            // 计算切线（沿着上下边的方向）
            Eigen::Vector3d tangent = (pt2 - pt1).normalized();
            // 投影到面上（去除法线方向分量）
            tangent = tangent - tangent.dot(face_normal) * face_normal;
            tangent.normalize();

            Pose pose;
            pose.position = pt1;
            pose.orientation = calculateOrientation(tangent, face_normal);
            row_trajectory.push_back(pose);
        }
    }
    else
    {
        // ========== 从右到左 ==========
        for (size_t curve_idx = upper_count - 1; curve_idx > 0; --curve_idx)
        {
            size_t prev_idx = curve_idx - 1;

            // 当前列的插值点
            double curr_pair_dist = (lower_points[curve_idx] - upper_points[curve_idx]).norm();
            double t_curr = sample_height / curr_pair_dist;
            t_curr = std::clamp(t_curr, 0.0, 1.0);
            Eigen::Vector3d pt1 = (1.0 - t_curr) * upper_points[curve_idx] +
                                  t_curr * lower_points[curve_idx];

            // 前一列的插值点
            double prev_pair_dist = (lower_points[prev_idx] - upper_points[prev_idx]).norm();
            double t_prev = sample_height / prev_pair_dist;
            t_prev = std::clamp(t_prev, 0.0, 1.0);
            Eigen::Vector3d pt2 = (1.0 - t_prev) * upper_points[prev_idx] +
                                  t_prev * lower_points[prev_idx];

            // 计算切线（从右到左，但方向仍然是从左指向右）
            Eigen::Vector3d tangent = (pt2 - pt1).normalized();
            tangent = tangent - tangent.dot(face_normal) * face_normal;
            tangent.normalize();

            Pose pose;
            pose.position = pt1;
            pose.orientation = calculateOrientation(tangent, face_normal);
            row_trajectory.push_back(pose);
        }
    }

    return row_trajectory;
}

// ========== 侧面IK轨迹函数（传递间隔参数） ==========
std::vector<Eigen::Matrix4d> CurveProcessor::generateSideFaceIKTrajectory(std::string file, double interval_mm)
{
    std::vector<Eigen::Matrix4d> ik_trajectory;
    std::vector<Pose> pose_trajectory;

    // 传递间隔参数到原始侧面轨迹生成函数
    const std::vector<Pose> &raw_side_poses = generateSideFaceTrajectory(interval_mm);
    if (raw_side_poses.empty())
    {
        std::cerr << "警告：侧面轨迹为空" << std::endl;
        return ik_trajectory;
    }

    // 插补参数（保持不变）
    const double mm_scale = 1000.0;    // 米→毫米转换
    const double z_offset = -2358.0;   // 机械臂基座Z轴偏移
    const double max_step_dist = 10.0; // 最大步长(mm)
    const double max_step_angle = 0.1; // 最大旋转步长(rad)
    const double min_interp_steps = 1; // 最小插值点数

    // 构造变换矩阵的辅助函数（保持不变）
    auto createTransform = [&](const Pose &pose)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(pose.orientation.w(), pose.orientation.x(),
                             pose.orientation.y(), pose.orientation.z());
        q.normalize();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(
            pose.position.x() * mm_scale,
            pose.position.y() * mm_scale,
            pose.position.z() * mm_scale + z_offset);
        return T;
    };

    // 初始化第一个点（保持不变）
    ik_trajectory.push_back(createTransform(raw_side_poses[0]));
    Pose first_pose = raw_side_poses[0];
    first_pose.position *= mm_scale;
    first_pose.position.z() += z_offset;
    pose_trajectory.push_back(first_pose);

    // 轨迹插补处理（保持不变）
    for (size_t i = 1; i < raw_side_poses.size(); ++i)
    {
        const Pose &prev_pose = raw_side_poses[i - 1];
        const Pose &curr_pose = raw_side_poses[i];

        Eigen::Vector3d prev_pos(prev_pose.position * mm_scale + Eigen::Vector3d(0, 0, z_offset));
        Eigen::Vector3d curr_pos(curr_pose.position * mm_scale + Eigen::Vector3d(0, 0, z_offset));

        Eigen::Quaterniond prev_q(prev_pose.orientation.w(), prev_pose.orientation.x(),
                                  prev_pose.orientation.y(), prev_pose.orientation.z());
        Eigen::Quaterniond curr_q(curr_pose.orientation.w(), curr_pose.orientation.x(),
                                  curr_pose.orientation.y(), curr_pose.orientation.z());
        prev_q.normalize();
        curr_q.normalize();

        double dist = (curr_pos - prev_pos).norm();
        double angle = prev_q.angularDistance(curr_q);
        int steps = std::max(
            static_cast<int>(dist / max_step_dist),
            static_cast<int>(angle / max_step_angle));
        steps = std::max(steps, static_cast<int>(min_interp_steps));

        for (int j = 1; j <= steps; ++j)
        {
            double t = static_cast<double>(j) / steps;

            Eigen::Vector3d interp_pos = prev_pos + t * (curr_pos - prev_pos);
            Eigen::Quaterniond interp_q = prev_q.slerp(t, curr_q);
            interp_q.normalize();

            Pose interp_pose;
            interp_pose.orientation = interp_q;
            interp_pose.position = interp_pos;

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = interp_q.toRotationMatrix();
            T.block<3, 1>(0, 3) = interp_pos;
            ik_trajectory.push_back(T);

            pose_trajectory.push_back(interp_pose);
        }
    }

    // 保存CSV（保持不变）
    if (file != " ")
    {
        savePosesToCSV(file, pose_trajectory);
    }

    return ik_trajectory;
}
