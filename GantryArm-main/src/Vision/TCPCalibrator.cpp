#include "TCPCalibrator.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

// CalibrationPoint 构造函数实现
CalibrationPoint::CalibrationPoint(const Point6D& flange_pose, const Point6D& theoretical_tcp)
    : robot_flange_pose(flange_pose), theoretical_tcp_pose(theoretical_tcp) {}

// TCPResult 构造函数实现
TCPResult::TCPResult() : accuracy(0.0), point_count(0) {}

// TCPCalibrator 方法实现
TCPCalibrator::TCPCalibrator() : reference_point_set_(false) {}

void TCPCalibrator::setTheoreticalTCP(const Point6D& tcp_offset) {
    theoretical_tcp_ = tcp_offset;
    std::cout << "设置理论TCP偏移: ";
    print(theoretical_tcp_);
}

void TCPCalibrator::setTheoreticalTCP(double x, double y, double z, double rx, double ry, double rz) {
    setTheoreticalTCP(Point6D(x, y, z, rx, ry, rz));
}

void TCPCalibrator::setReferencePoint(const Point6D& ref_point) {
    reference_point_ = ref_point;
    reference_point_set_ = true;
    std::cout << "设置固定参考点: ";
    print(reference_point_);
}

void TCPCalibrator::setReferencePoint(double x, double y, double z) {
    setReferencePoint(Point6D(x, y, z, 0, 0, 0));
}

void TCPCalibrator::addCalibrationPose(const Point6D& robot_tcp_pose) {
    // 计算基于理论TCP的TCP位姿
    Eigen::Matrix4d robot_tcp_matrix = toEigenMatrix(robot_tcp_pose);
    Eigen::Matrix4d tcp_offset_matrix_inverse = toEigenMatrix(theoretical_tcp_).inverse();
    Eigen::Matrix4d theoretical_flange_pose_matrix = robot_tcp_matrix * tcp_offset_matrix_inverse;
    Point6D flange_pose=fromEigenMatrix(theoretical_flange_pose_matrix);
    calibration_points_.emplace_back(flange_pose,robot_tcp_pose);

    std::cout << "添加标定姿态 " << calibration_points_.size()
        << "，理论TCP位置: (" << std::fixed << std::setprecision(3)
        << robot_tcp_pose.x << ", " << robot_tcp_pose.y
        << ", " << robot_tcp_pose.z << ")\n";
    std::cout << "添加标定姿态 " << calibration_points_.size()
              << "，理论法兰盘位置: (" << std::fixed << std::setprecision(3)
              << flange_pose.x << ", " << flange_pose.y
              << ", " << flange_pose.z << ")\n";
}

Point6D TCPCalibrator::calculateReferencePoint() const {
    if (calibration_points_.empty()) {
        return Point6D();
    }

    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto& point : calibration_points_) {
        sum_x += point.theoretical_tcp_pose.x;
        sum_y += point.theoretical_tcp_pose.y;
        sum_z += point.theoretical_tcp_pose.z;
    }

    size_t n = calibration_points_.size();
    return Point6D(sum_x / n, sum_y / n, sum_z / n, 0, 0, 0);
}

std::optional<TCPResult> TCPCalibrator::calibrateTCP() {
    if (calibration_points_.size() < 4) {
        std::cout << "标定姿态不足，至少需要4个姿态，当前只有 "
            << calibration_points_.size() << " 个姿态\n";
        return std::nullopt;
    }

    // 如果没有设置参考点，则计算参考点
    if (!reference_point_set_) {
        reference_point_ = calculateReferencePoint();
        std::cout << "自动计算参考点: ";
        print(reference_point_);
    }

    size_t n = calibration_points_.size();
    Eigen::MatrixXd A(3 * n, 3); // 求解TCP位置修正
    Eigen::VectorXd b(3 * n);

    // 参考点位置
    Eigen::Vector3d ref_pos(reference_point_.x, reference_point_.y, reference_point_.z);

    for (size_t i = 0; i < n; ++i) {
        const auto& point = calibration_points_[i];

        // 将法兰姿态转换为变换矩阵
        Eigen::Matrix4d flange_matrix = toEigenMatrix(point.robot_flange_pose);
        Eigen::Matrix3d R = getRotation(flange_matrix);
        Eigen::Vector3d flange_pos = getPosition(flange_matrix);

        // 理论TCP偏移
        Eigen::Vector3d theoretical_tcp_offset(
            theoretical_tcp_.x, theoretical_tcp_.y, theoretical_tcp_.z
        );

        // 构建线性方程组
        // 目标：R * (theoretical_tcp + correction) + flange_pos = reference_point
        // 即：R * correction = reference_point - (R * theoretical_tcp + flange_pos)
        A.block<3, 3>(3 * i, 0) = R;

        Eigen::Vector3d theoretical_tcp_world = R * theoretical_tcp_offset + flange_pos;
        b.segment<3>(3 * i) = ref_pos - theoretical_tcp_world;
    }

    // 最小二乘求解TCP修正量
    Eigen::Vector3d tcp_correction = A.colPivHouseholderQr().solve(b);

    // 计算实际TCP
    Point6D actual_tcp;
    actual_tcp.x = theoretical_tcp_.x + tcp_correction.x();
    actual_tcp.y = theoretical_tcp_.y + tcp_correction.y();
    actual_tcp.z = theoretical_tcp_.z + tcp_correction.z();
    actual_tcp.rx = theoretical_tcp_.rx; 
    actual_tcp.ry = theoretical_tcp_.ry;
    actual_tcp.rz = theoretical_tcp_.rz;

    // 计算RMS误差
    double rms_error = calculateRMSError(actual_tcp);

    // 构造结果
    TCPResult result;
    result.theoretical_tcp = theoretical_tcp_;
    result.actual_tcp = actual_tcp;
    result.tcp_correction = Point6D(tcp_correction.x(), tcp_correction.y(), tcp_correction.z(), 0, 0, 0);
    result.reference_point = reference_point_;
    result.accuracy = rms_error;
    result.point_count = n;

    current_tcp_result_ = result;
    return result;
}

double TCPCalibrator::calculateRMSError(const Point6D& actual_tcp) const {
    double sum_squared_error = 0.0;
    Eigen::Vector3d ref_pos(reference_point_.x, reference_point_.y, reference_point_.z);

    for (const auto& point : calibration_points_) {
        Eigen::Matrix4d flange_matrix = toEigenMatrix(point.robot_flange_pose);
        Eigen::Matrix3d R = getRotation(flange_matrix);
        Eigen::Vector3d flange_pos = getPosition(flange_matrix);

        // 使用实际TCP计算预测位置
        Eigen::Vector3d actual_tcp_offset(actual_tcp.x, actual_tcp.y, actual_tcp.z);
        Eigen::Vector3d predicted_tcp = R * actual_tcp_offset + flange_pos;

        Eigen::Vector3d error = predicted_tcp - ref_pos;
        sum_squared_error += error.squaredNorm();
    }

    return std::sqrt(sum_squared_error / calibration_points_.size());
}

Point6D TCPCalibrator::calculateActualTCPPose(const Point6D& flange_pose) const {
    if (!current_tcp_result_) {
        std::cout << "TCP未标定，使用理论TCP\n";
        Eigen::Matrix4d flange_matrix = toEigenMatrix(flange_pose);
        Eigen::Matrix4d tcp_offset_matrix = toEigenMatrix(theoretical_tcp_);
        Eigen::Matrix4d tcp_pose_matrix = flange_matrix * tcp_offset_matrix;
        return fromEigenMatrix(tcp_pose_matrix);
    }

    Eigen::Matrix4d flange_matrix = toEigenMatrix(flange_pose);
    Eigen::Matrix4d tcp_offset_matrix = toEigenMatrix(current_tcp_result_->actual_tcp);
    Eigen::Matrix4d tcp_pose_matrix = flange_matrix * tcp_offset_matrix;
    return fromEigenMatrix(tcp_pose_matrix);
}

double TCPCalibrator::validateTCP() const {
    if (!current_tcp_result_) {
        std::cout << "TCP未标定，无法验证\n";
        return -1.0;
    }

    Point6D predicted_tcp_pose = calculateActualTCPPose(calibration_points_[0].robot_flange_pose);

    Eigen::Vector3d predicted_pos(predicted_tcp_pose.x, predicted_tcp_pose.y, predicted_tcp_pose.z);
    Eigen::Vector3d ref_pos(reference_point_.x, reference_point_.y, reference_point_.z);

    double error = (predicted_pos - ref_pos).norm();
    std::cout << "TCP验证误差: " << error << " mm\n";

    return error;
}

std::optional<TCPResult> TCPCalibrator::getCurrentTCPResult() const {
    return current_tcp_result_;
}

Point6D TCPCalibrator::getTheoreticalTCP() const {
    return theoretical_tcp_;
}

std::optional<Point6D> TCPCalibrator::getReferencePoint() const {
    if (reference_point_set_ || !calibration_points_.empty()) {
        return reference_point_;
    }
    return std::nullopt;
}

void TCPCalibrator::clearAll() {
    calibration_points_.clear();
    current_tcp_result_.reset();
    reference_point_set_ = false;
    std::cout << "已清除所有数据\n";
}

void TCPCalibrator::clearCalibrationData() {
    calibration_points_.clear();
    std::cout << "已清除标定数据\n";
}

size_t TCPCalibrator::getCalibrationPointCount() const noexcept {
    return calibration_points_.size();
}

void TCPCalibrator::printCalibrationInfo() const {
    std::cout << "=== TCP标定信息 ===\n";
    std::cout << "理论TCP: ";
    print(theoretical_tcp_);

    if (auto ref = getReferencePoint()) {
        std::cout << "参考点: ";
        print(ref.value());
    }

    std::cout << "标定姿态数量: " << calibration_points_.size() << "\n";

    if (current_tcp_result_) {
        std::cout << "标定状态: 已完成\n";
        std::cout << "实际TCP: ";
        print(current_tcp_result_->actual_tcp);
        std::cout << "TCP修正: ";
        print(current_tcp_result_->tcp_correction);
        std::cout << "标定精度: " << current_tcp_result_->accuracy << " mm\n";
    }
    else {
        std::cout << "标定状态: 未完成\n";
    }
    std::cout << "==================\n";
}

void TCPCalibrator::printCalibrationPoints() const {
    std::cout << "=== 标定姿态列表 ===\n";
    std::cout << "理论TCP: ";
    print(theoretical_tcp_);

    if (auto ref = getReferencePoint()) {
        std::cout << "参考点: ";
        print(ref.value());
    }

    std::cout << "\n";

    for (size_t i = 0; i < calibration_points_.size(); ++i) {
        std::cout << "标定姿态 " << i + 1 << ":\n";
        std::cout << "  法兰位姿:\n";
        std::cout << "    ";
        print(calibration_points_[i].robot_flange_pose);

        std::cout << "  理论TCP位姿:\n";
        std::cout << "    ";
        print(calibration_points_[i].theoretical_tcp_pose);

        // 如果有参考点，计算理论TCP到参考点的距离
        if (reference_point_set_) {
            double distance = std::sqrt(
                std::pow(calibration_points_[i].theoretical_tcp_pose.x - reference_point_.x, 2) +
                std::pow(calibration_points_[i].theoretical_tcp_pose.y - reference_point_.y, 2) +
                std::pow(calibration_points_[i].theoretical_tcp_pose.z - reference_point_.z, 2)
            );
            std::cout << "  到参考点距离: " << std::fixed << std::setprecision(3)
                << distance << " mm\n";
        }
        std::cout << "\n";
    }
    std::cout << "==================\n";
}

bool TCPCalibrator::saveCalibrationData(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "无法打开文件进行写入: " << filename << "\n";
        return false;
    }

    // 写入头部信息
    file << "# TCP Calibration Data\n";
    file << "# Theoretical TCP (x,y,z,rx,ry,rz): "
        << theoretical_tcp_.x << "," << theoretical_tcp_.y << "," << theoretical_tcp_.z << ","
        << theoretical_tcp_.rx << "," << theoretical_tcp_.ry << "," << theoretical_tcp_.rz << "\n";

    if (reference_point_set_) {
        file << "# Reference Point (x,y,z): "
            << reference_point_.x << "," << reference_point_.y << "," << reference_point_.z << "\n";
    }
    else {
        file << "# Reference Point: auto-calculated\n";
    }

    file << "# Columns: flange_x,flange_y,flange_z,flange_rx,flange_ry,flange_rz,"
        << "theoretical_tcp_x,theoretical_tcp_y,theoretical_tcp_z,"
        << "theoretical_tcp_rx,theoretical_tcp_ry,theoretical_tcp_rz\n";

    // 写入列标题
    file << "flange_x,flange_y,flange_z,flange_rx,flange_ry,flange_rz,"
        << "theoretical_tcp_x,theoretical_tcp_y,theoretical_tcp_z,"
        << "theoretical_tcp_rx,theoretical_tcp_ry,theoretical_tcp_rz\n";

    // 写入数据
    file << std::fixed << std::setprecision(6);
    for (const auto& point : calibration_points_) {
        // 法兰位姿
        file << point.robot_flange_pose.x << ","
            << point.robot_flange_pose.y << ","
            << point.robot_flange_pose.z << ","
            << point.robot_flange_pose.rx << ","
            << point.robot_flange_pose.ry << ","
            << point.robot_flange_pose.rz << ",";

        // 理论TCP位姿
        file << point.theoretical_tcp_pose.x << ","
            << point.theoretical_tcp_pose.y << ","
            << point.theoretical_tcp_pose.z << ","
            << point.theoretical_tcp_pose.rx << ","
            << point.theoretical_tcp_pose.ry << ","
            << point.theoretical_tcp_pose.rz << "\n";
    }

    std::cout << "标定数据已保存到: " << filename << " (共"
        << calibration_points_.size() << "个姿态)\n";
    return true;
}

bool TCPCalibrator::loadCalibrationData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "无法打开文件进行读取: " << filename << "\n";
        return false;
    }

    calibration_points_.clear();
    std::string line;

    // 读取头部信息
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (line.find("# Theoretical TCP") != std::string::npos) {
            // 解析理论TCP
            size_t colon_pos = line.find(": ");
            if (colon_pos != std::string::npos) {
                std::string tcp_data = line.substr(colon_pos + 2);
                std::istringstream iss(tcp_data);
                std::string token;
                std::vector<double> values;

                while (std::getline(iss, token, ',')) {
                    try {
                        values.push_back(std::stod(token));
                    }
                    catch (const std::exception& e) {
                        std::cout << "解析理论TCP数据出错: " << e.what() << "\n";
                        return false;
                    }
                }

                if (values.size() == 6) {
                    theoretical_tcp_ = Point6D(values[0], values[1], values[2],
                        values[3], values[4], values[5]);
                    std::cout << "加载理论TCP: ";
                    print(theoretical_tcp_);
                }
            }
        }
        else if (line.find("# Reference Point") != std::string::npos &&
            line.find("auto-calculated") == std::string::npos) {
            // 解析参考点
            size_t colon_pos = line.find(": ");
            if (colon_pos != std::string::npos) {
                std::string ref_data = line.substr(colon_pos + 2);
                std::istringstream iss(ref_data);
                std::string token;
                std::vector<double> values;

                while (std::getline(iss, token, ',')) {
                    try {
                        values.push_back(std::stod(token));
                    }
                    catch (const std::exception& e) {
                        continue;
                    }
                }

                if (values.size() == 3) {
                    reference_point_ = Point6D(values[0], values[1], values[2], 0, 0, 0);
                    reference_point_set_ = true;
                    std::cout << "加载参考点: ";
                    print(reference_point_);
                }
            }
        }
        else if (line.find("#") != 0) {
            // 到达数据行
            break;
        }
    }

    // 跳过列标题（如果当前行是标题行）
    if (line.find("flange_x") != std::string::npos) {
        std::getline(file, line);
    }

    size_t line_count = 0;
    do {
        if (line.empty() || line[0] == '#') continue;

        line_count++;
        std::istringstream iss(line);
        std::string token;
        std::vector<double> values;

        // 解析CSV行
        while (std::getline(iss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            }
            catch (const std::exception& e) {
                std::cout << "第" << line_count << "行解析数据出错: " << e.what() << "\n";
                continue;
            }
        }

        if (values.size() != 12) {
            std::cout << "第" << line_count << "行数据格式错误，期望12列，实际"
                << values.size() << "列\n";
            continue;
        }

        Point6D flange_pose(values[0], values[1], values[2],
            values[3], values[4], values[5]);
        Point6D theoretical_tcp_pose(values[6], values[7], values[8],
            values[9], values[10], values[11]);

        calibration_points_.emplace_back(flange_pose, theoretical_tcp_pose);

    } while (std::getline(file, line));

    std::cout << "从文件 " << filename << " 加载了 "
        << calibration_points_.size() << " 个标定姿态\n";

    // 如果没有设置参考点，自动计算
    if (!reference_point_set_ && !calibration_points_.empty()) {
        reference_point_ = calculateReferencePoint();
        std::cout << "自动计算参考点: ";
        print(reference_point_);
    }

    return true;
}

bool TCPCalibrator::saveTCPResult(const std::string& filename) const {
    if (!current_tcp_result_) {
        std::cout << "没有TCP标定结果可保存\n";
        return false;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "无法打开文件进行写入: " << filename << "\n";
        return false;
    }

    const auto& result = current_tcp_result_.value();

    // 写入头部信息
    file << "# TCP Calibration Result\n";
    file << "# Point Count: " << result.point_count << "\n";
    file << "# RMS Error: " << result.accuracy << " mm\n";
    file << "# Columns: theoretical_tcp_x,theoretical_tcp_y,theoretical_tcp_z,"
        << "theoretical_tcp_rx,theoretical_tcp_ry,theoretical_tcp_rz,"
        << "actual_tcp_x,actual_tcp_y,actual_tcp_z,"
        << "actual_tcp_rx,actual_tcp_ry,actual_tcp_rz,"
        << "correction_x,correction_y,correction_z,"
        << "correction_rx,correction_ry,correction_rz,"
        << "reference_x,reference_y,reference_z,"
        << "rms_error,point_count\n";

    // 写入列标题
    file << "theoretical_tcp_x,theoretical_tcp_y,theoretical_tcp_z,"
        << "theoretical_tcp_rx,theoretical_tcp_ry,theoretical_tcp_rz,"
        << "actual_tcp_x,actual_tcp_y,actual_tcp_z,"
        << "actual_tcp_rx,actual_tcp_ry,actual_tcp_rz,"
        << "correction_x,correction_y,correction_z,"
        << "correction_rx,correction_ry,correction_rz,"
        << "reference_x,reference_y,reference_z,"
        << "rms_error,point_count\n";

    // 写入数据
    file << std::fixed << std::setprecision(6);
    file << result.theoretical_tcp.x << "," << result.theoretical_tcp.y << "," << result.theoretical_tcp.z << ","
        << result.theoretical_tcp.rx << "," << result.theoretical_tcp.ry << "," << result.theoretical_tcp.rz << ","
        << result.actual_tcp.x << "," << result.actual_tcp.y << "," << result.actual_tcp.z << ","
        << result.actual_tcp.rx << "," << result.actual_tcp.ry << "," << result.actual_tcp.rz << ","
        << result.tcp_correction.x << "," << result.tcp_correction.y << "," << result.tcp_correction.z << ","
        << result.tcp_correction.rx << "," << result.tcp_correction.ry << "," << result.tcp_correction.rz << ","
        << result.reference_point.x << "," << result.reference_point.y << "," << result.reference_point.z << ","
        << result.accuracy << "," << result.point_count << "\n";

    std::cout << "TCP标定结果已保存到: " << filename << "\n";
    std::cout << "  实际TCP: (" << result.actual_tcp.x << ", "
        << result.actual_tcp.y << ", " << result.actual_tcp.z << ")\n";
    std::cout << "  标定精度: " << result.accuracy << " mm\n";

    return true;
}

bool TCPCalibrator::loadTCPResult(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "无法打开文件进行读取: " << filename << "\n";
        return false;
    }

    std::string line;

    // 跳过头部注释和列标题
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.find("theoretical_tcp_x") != std::string::npos) continue;
        break;
    }

    if (line.empty()) {
        std::cout << "TCP结果文件为空或格式错误\n";
        return false;
    }

    std::istringstream iss(line);
    std::string token;
    std::vector<double> values;

    while (std::getline(iss, token, ',')) {
        try {
            values.push_back(std::stod(token));
        }
        catch (const std::exception& e) {
            std::cout << "解析TCP结果出错: " << e.what() << "\n";
            return false;
        }
    }

    if (values.size() < 21) { // 至少需要21列数据
        std::cout << "TCP结果文件格式错误，数据列不足\n";
        return false;
    }

    TCPResult result;
    result.theoretical_tcp = Point6D(values[0], values[1], values[2],
        values[3], values[4], values[5]);
    result.actual_tcp = Point6D(values[6], values[7], values[8],
        values[9], values[10], values[11]);
    result.tcp_correction = Point6D(values[12], values[13], values[14],
        values[15], values[16], values[17]);
    result.reference_point = Point6D(values[18], values[19], values[20], 0, 0, 0);
    result.accuracy = values[21];
    result.point_count = (values.size() >= 23) ? static_cast<size_t>(values[22]) : 0;

    // 同步加载的数据到成员变量
    theoretical_tcp_ = result.theoretical_tcp;
    reference_point_ = result.reference_point;
    reference_point_set_ = true;
    current_tcp_result_ = result;

    std::cout << "TCP标定结果已从文件 " << filename << " 加载\n";
    std::cout << "  理论TCP: (" << result.theoretical_tcp.x << ", "
        << result.theoretical_tcp.y << ", " << result.theoretical_tcp.z << ")\n";
    std::cout << "  实际TCP: (" << result.actual_tcp.x << ", "
        << result.actual_tcp.y << ", " << result.actual_tcp.z << ")\n";
    std::cout << "  标定精度: " << result.accuracy << " mm\n";

    return true;
}

bool TCPCalibrator::loadCalibrationPosesFromCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开CSV文件: " << filename << std::endl;
        return false;
    }

    std::string line;
    std::vector<Eigen::Matrix4d> matrices;  // 存储所有矩阵
    Eigen::Matrix4d currentMatrix;

    // 跳过第一行标头
    if (!std::getline(file, line)) {
        std::cerr << "CSV文件为空" << std::endl;
        return false;
    }

    // 读取数据行（每行代表一个4x4矩阵，含16个元素）
    int matrixIndex = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string value;
        std::vector<double> matrixData;
        matrixData.reserve(16);  // 预留16个元素空间

        // 解析一行中的16个元素
        for (int i = 0; i < 16; ++i) {
            if (!std::getline(iss, value, ',')) {
                std::cerr << "CSV格式错误，矩阵 " << (matrixIndex + 1)
                          << " 元素不足（需要16个，实际" << (i + 1) << "个）" << std::endl;
                return false;
            }
            try {
                matrixData.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "解析矩阵 " << (matrixIndex + 1) << " 的数值失败: " << e.what() << std::endl;
                return false;
            }
        }

        // 检查是否有多余元素
        if (std::getline(iss, value, ',')) {
            std::cerr << "CSV格式错误，矩阵 " << (matrixIndex + 1) << " 元素过多（超过16个）" << std::endl;
            return false;
        }

        // 将16个元素按行优先填充到4x4矩阵
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                currentMatrix(row, col) = matrixData[row * 4 + col];
            }
        }
        std::cout << "成功从CSV文件加载第"<<matrixIndex<<"姿态数据" << std::endl;

        matrices.push_back(currentMatrix);
        matrixIndex++;
    }
    std::cout << "CSV文件实际读取到 " << matrices.size() << " 个" << std::endl;

    // 处理每个矩阵，转换为Point6D并添加到校准器
    for (const auto& mat : matrices) {
        Point6D pose = fromEigenMatrix(mat);
        addCalibrationPose(pose);
    }

    std::cout << "成功从CSV文件加载姿态数据: " << filename << std::endl;
    return true;
}
