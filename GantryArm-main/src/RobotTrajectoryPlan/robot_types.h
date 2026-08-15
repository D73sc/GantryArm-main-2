#pragma once
#include <cmath>
#include <array>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define M_PI 3.1415926

struct Point6D {
    double x, y, z;     // 位置坐标
    double rx, ry, rz;  // 旋转角度(弧度)

    // 默认构造函数
    Point6D() : x(0), y(0), z(0), rx(0), ry(0), rz(0) {}

    // 构造函数
    Point6D(double x, double y, double z, double rx, double ry, double rz)
        : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz) {}
};

// 机械臂位置结构体
struct RobotPose {
    Point6D tcp_pose;           // TCP在世界坐标系中的位姿
    Point6D world_pose;         // 世界坐标系原点位置（相对于基座标系）
    Eigen::Matrix4d flange_matrix4d;
    std::array<double, 8> joint_positions;  // 关节角度
    double timestamp;           // 时间戳

    // 默认构造函数
    RobotPose() : timestamp(0.0) {}

    // 构造函数
    RobotPose(const Point6D& tcp)
        : tcp_pose(tcp), timestamp(0.0) {}
};

// 静态方法：将Point6D转换为Eigen::Matrix4d变换矩阵(内旋ZYX RX*RY*RZ)
static Eigen::Matrix4d toEigenMatrix(const Point6D& point) {
    double cx = std::cos(point.rx), sx = std::sin(point.rx);
    double cy = std::cos(point.ry), sy = std::sin(point.ry);
    double cz = std::cos(point.rz), sz = std::sin(point.rz);

    Eigen::Matrix4d matrix;
    matrix << cy * cz, -cy * sz, sy, point.x,
        cx* sz + sx * sy * cz, cx* cz - sx * sy * sz, -sx * cy, point.y,
        sx* sz - cx * sy * cz, sx* cz + cx * sy * sz, cx* cy, point.z,
        0, 0, 0, 1;

    return matrix;
}
static Eigen::Matrix4d toEigenMatrixXYZ(const Point6D& point) {
    double cx = std::cos(point.rx), sx = std::sin(point.rx);
    double cy = std::cos(point.ry), sy = std::sin(point.ry);
    double cz = std::cos(point.rz), sz = std::sin(point.rz);

    // 计算每个单轴旋转矩阵元素
    // 内旋XYZ顺序对应矩阵为: R = Rz * Ry * Rx
    // 其解析表达式如下：
    Eigen::Matrix4d matrix;
    matrix << cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, point.x,
        sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, point.y,
        -sy,     cy * sx,                cy * cx,                 point.z,
        0,       0,                      0,                       1;

    return matrix;
}
// 静态方法：从Eigen::Matrix4d转换为Point6D
static Point6D fromEigenMatrix(const Eigen::Matrix4d& matrix) {
    Point6D point;

    // 提取位置
    point.x = matrix(0, 3);
    point.y = matrix(1, 3);
    point.z = matrix(2, 3);

    // 提取旋转角度(内旋ZYX)
    point.ry = std::asin(std::clamp(matrix(0, 2), -1.0, 1.0));

    if (std::abs(std::cos(point.ry)) > 1e-6) {
        point.rx = std::atan2(-matrix(1, 2), matrix(2, 2));
        point.rz = std::atan2(-matrix(0, 1), matrix(0, 0));
    }
    else {
        point.rx = 0;
        if (point.ry > 0) {
            point.rz = std::atan2(matrix(1, 0), matrix(1, 1));
        }
        else {
            point.rz = std::atan2(-matrix(1, 0), matrix(1, 1));
        }
    }

    return point;
}

static Point6D fromEigenMatrixXYZ(const Eigen::Matrix4d& matrix) {
    Point6D point;

    // 提取位置
    point.x = matrix(0, 3);
    point.y = matrix(1, 3);
    point.z = matrix(2, 3);

    // 提取旋转矩阵3x3部分
    const Eigen::Matrix3d R = matrix.block<3,3>(0,0);

    // 内旋XYZ顺序回推欧拉角
    // pitch = asin(-R(2,0))
    double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    double cos_pitch = std::cos(pitch);

    double roll, yaw;
    if (std::abs(cos_pitch) > 1e-6) { // 非奇异
        roll = std::atan2(R(2,1), R(2,2));
        yaw = std::atan2(R(1,0), R(0,0));
    } else { // 奇异
        roll = 0;
        if (pitch > 0) {
            yaw = std::atan2(-R(0,1), R(1,1));
        } else {
            yaw = std::atan2(R(0,1), R(1,1));
        }
    }

    point.rx = roll;
    point.ry = pitch;
    point.rz = yaw;

    return point;
}

// 静态方法：角度转弧度
static double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// 静态方法：弧度转角度
static double radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

// 静态方法：打印Point6D
static void print(const Point6D& point) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Position: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    std::cout << "Rotation: (" << radToDeg(point.rx) << "°, "
        << radToDeg(point.ry) << "°, " << radToDeg(point.rz) << "°)\n";
}

// 静态方法：打印Eigen变换矩阵
static void printEigenMatrix(const Eigen::Matrix4d& matrix) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Transform Matrix (Eigen):\n";
    std::cout << matrix << "\n";
}

// 静态方法：点变换 (使用Eigen)
static Point6D transform(const Point6D& point, const Eigen::Matrix4d& transform_matrix) {
    Eigen::Matrix4d point_matrix = toEigenMatrix(point);
    Eigen::Matrix4d result = transform_matrix * point_matrix;
    return fromEigenMatrix(result);
}

// 从变换矩阵提取位置
static Eigen::Vector3d getPosition(const Eigen::Matrix4d& matrix) {
    return matrix.block<3, 1>(0, 3);
}

// 从变换矩阵提取旋转矩阵
static Eigen::Matrix3d getRotation(const Eigen::Matrix4d& matrix) {
    return matrix.block<3, 3>(0, 0);
}

static bool areThetasEqual(const std::array<double, 8>& a, const std::array<double, 8>& b,const double epsilon=1e-5) {
    for (size_t i = 0; i < 8; ++i) {
        if (std::abs(a[i] - b[i]) > epsilon) {
            return false;
        }
    }
    return true;
}

static Eigen::Matrix4d poseToTransformMatrix(
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    Eigen::Quaterniond q(qw, qx, qy, qz);

    if (q.norm() < 1e-6) {
        throw std::invalid_argument("四元数模长过小");
    }
    q.normalize();

    T.block<3, 3>(0, 0) = q.toRotationMatrix();

    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;

    return T;
}

// filename: csv 文件路径
// isInputInMillimeter: true 表示 CSV 中位置单位是毫米，false 表示米，需要乘以 1000 转换成毫米单位
// zOffsetMm: z 轴的偏移值，默认为 -2358 毫米
static std::vector<Eigen::Matrix4d> loadTrajectoryFromCSV(
    const std::string& filename,
    bool isInputInMillimeter = false,
    double zOffsetMm = -2358.0)
{
    std::vector<Eigen::Matrix4d> trajectory;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return trajectory;
    }

    std::string line;

    // 跳过标题行
    if (!std::getline(file, line)) {
        std::cerr << "文件为空或者读取失败: " << filename << std::endl;
        return trajectory;
    }

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;  // 跳过空行
        }

        // 将逗号替换为空格，方便使用 istringstream 解析
        std::replace(line.begin(), line.end(), ',', ' ');

        std::istringstream iss(line);
        double x, y, z, qx, qy, qz, qw;

        if (iss >> x >> y >> z >> qx >> qy >> qz >> qw) {
            double scale = isInputInMillimeter ? 1.0 : 1000.0;

            double x_mm = x * scale;
            double y_mm = y * scale;
            double z_mm = z * scale + zOffsetMm;

            try {
                Eigen::Matrix4d pose = poseToTransformMatrix(x_mm, y_mm, z_mm, qx, qy, qz, qw);
                trajectory.push_back(pose);
            }
            catch (const std::exception& e) {
                std::cerr << "四元数错误，跳过该行: " << line << " 原因: " << e.what() << std::endl;
            }
        }
        else {
            std::cerr << "格式错误，无法解析该行: " << line << std::endl;
        }
    }

    file.close();

    std::cout << "共加载轨迹点数: " << trajectory.size() << std::endl;
    return trajectory;
}

// 将单个位姿保存到CSV文件（追加模式）
static bool savePoseToCSV(const std::string& csvPath, const Eigen::Matrix4d& pose, bool append = true) {
    std::ofstream file;

    if (append) {
        file.open(csvPath, std::ios::app);  // 追加模式
    } else {
        file.open(csvPath, std::ios::out);  // 覆盖模式
    }

    if (!file.is_open()) {
        std::cerr << "Error: Cannot open CSV file for writing: " << csvPath << std::endl;
        return false;
    }

    // 写入4x4矩阵，每行一个位姿，16个数值用逗号分隔
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file << pose(i, j);
            if (i != 3 || j != 3) {  // 不是最后一个元素
                file << ",";
            }
        }
    }
    file << std::endl;

    file.close();
    return true;
}

static bool savePoint6DToCSV(const std::string& csvPath, const Point6D& point, bool append = true) {
    std::ofstream file;
    if (append) {
        file.open(csvPath, std::ios::app);  // 追加模式
    } else {
        file.open(csvPath, std::ios::out);  // 覆盖模式
    }

    if (!file.is_open()) {
        std::cerr << "Error: Cannot open CSV file for writing: " << csvPath << std::endl;
        return false;
    }

    if (!append) {
        // 写表头
        file << "x,y,z,rx,ry,rz\n";
    }

    // 写数据行
    file << point.x << "," << point.y << "," << point.z << ","
         << point.rx << "," << point.ry << "," << point.rz << "\n";

    file.close();
    return true;
}

static bool readPoint6DFromCSV(const std::string& csvPath, std::vector<Point6D>& points) {
    std::ifstream file(csvPath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open CSV file for reading: " << csvPath << std::endl;
        return false;
    }
    std::string line;

    std::getline(file, line); // 先读表头行，丢弃
    points.clear();

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        Point6D point;
        std::string token;
        // 依次解析6个数值
        bool success = true;
        double values[6];

        for (int i = 0; i < 6; i++) {
            if (!std::getline(ss, token, ',')) {
                success = false;
                break;
            }
            try {
                values[i] = std::stod(token);
            } catch (const std::exception&) {
                success = false;
                break;
            }
        }

        if (!success) {
            std::cerr << "Warning: skipping malformed line: " << line << std::endl;
            continue;
        }
        // 创建Point6D
        point = Point6D(values[0], values[1], values[2], values[3], values[4], values[5]);
        points.push_back(point);
    }

    file.close();
    return true;
}

static bool saveVectorToCSV(const std::string& csvPath, const std::vector<double>& data,
                            bool append = true, const std::string& header = "") {
    std::ofstream file;
    if (append) {
        file.open(csvPath, std::ios::app);  // 追加模式
    } else {
        file.open(csvPath, std::ios::out);  // 覆盖模式
    }

    if (!file.is_open()) {
        std::cerr << "Error: Cannot open CSV file for writing: " << csvPath << std::endl;
        return false;
    }

    if (!append && !header.empty()) {
        // 写入表头（可选）
        file << header << "\n";
    }

    // 写入数据，逗号分隔
    for (size_t i = 0; i < data.size(); ++i) {
        file << data[i];
        if (i != data.size() - 1) {
            file << ",";
        }
    }
    file << "\n";

    file.close();
    return true;
}

static bool readVectorsFromCSV(const std::string& csvPath,
                               std::vector<std::vector<double>>& data,
                               bool skipHeader = false) {
    std::ifstream file(csvPath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open CSV file for reading: " << csvPath << std::endl;
        return false;
    }

    data.clear();

    std::string line;
    // 跳过表头行（如果指定）
    if (skipHeader) {
        if (!std::getline(file, line)) {
            // 文件为空
            return true;
        }
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;  // 跳过空行

        std::istringstream ss(line);
        std::string token;
        std::vector<double> row;
        bool success = true;

        // 按逗号分割读取每个数字
        while (std::getline(ss, token, ',')) {
            try {
                double val = std::stod(token);
                row.push_back(val);
            } catch (const std::exception& e) {
                std::cerr << "Warning: invalid number \"" << token << "\" in line, skipping line: " << line << std::endl;
                success = false;
                break;
            }
        }

        if (success && !row.empty()) {
            data.push_back(row);
        }
    }

    file.close();
    return true;
}
