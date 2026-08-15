#include "ForceTorqueGravityCompensator.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

ForceTorqueGravityCompensator::ForceTorqueGravityCompensator() {
}

// std::vector<double> ForceTorqueGravityCompensator::parseLineToDoubles(const std::string& line, char delimiter) {
//     std::vector<double> values;
//     std::stringstream ss(line);
//     std::string value;
//     while (std::getline(ss, value, delimiter)) {
//         if (!value.empty()) {
//             values.push_back(std::stod(value));
//         }
//     }
//     return values;
// }

// std::vector<Eigen::Matrix3d> ForceTorqueGravityCompensator::loadRotation(const std::string& path, char delimiter) {
//     std::vector<Eigen::Matrix3d> rotation_list;
//     std::ifstream file(path);
//     if (!file.is_open()) throw std::runtime_error("无法打开文件: " + path);

//     std::string line;
//     while (std::getline(file, line)) {
//         auto values = parseLineToDoubles(line, delimiter);
//         if (values.size() < 3) continue;

//         double roll = values[0], pitch = values[1], yaw = values[2];
//         Eigen::Matrix3d Rx = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//         Eigen::Matrix3d Ry = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
//         Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
//         Eigen::Matrix3d rotation = Rz * Ry * Rx;
//         rotation_list.push_back(rotation);
//     }
//     return rotation_list;
// }

// Eigen::MatrixXd ForceTorqueGravityCompensator::loadForceTorque(const std::string& path, char delimiter) {
//     std::ifstream file(path);
//     if (!file.is_open()) throw std::runtime_error("无法打开文件: " + path);

//     std::vector<Eigen::VectorXd> rows;
//     std::string line;
//     while (std::getline(file, line)) {
//         auto values = parseLineToDoubles(line, delimiter);
//         if (values.empty()) continue;
//         Eigen::VectorXd row(values.size());
//         for (size_t i = 0; i < values.size(); ++i) row(i) = values[i];
//         rows.push_back(row);
//     }

//     int n_rows = rows.size();
//     int n_cols = rows[0].size();
//     Eigen::MatrixXd mat(n_rows, n_cols);
//     for (int i = 0; i < n_rows; ++i) {
//         mat.row(i) = rows[i].transpose();
//     }
//     return mat.transpose();
// }

Eigen::MatrixXd ForceTorqueGravityCompensator::matTrans(const Eigen::Vector3d& v) {
    Eigen::MatrixXd m(3, 6);
    m << 0, v.z(), -v.y(), 1.0, 0.0, 0.0,
        -v.z(), 0, v.x(), 0.0, 1.0, 0.0,
        v.y(), -v.x(), 0.0, 0.0, 0.0, 1.0;
    return m;
}

void ForceTorqueGravityCompensator::identifyGravityParams(const Eigen::MatrixXd& ft_data, const std::vector<Eigen::Matrix3d>& rotation_list) {
    int data_cols = ft_data.cols();
    Eigen::VectorXd M_all(3 * data_cols);
    Eigen::VectorXd F_all(3 * data_cols);
    Eigen::VectorXd p(6);
    Eigen::VectorXd l(6);

    // 填充M_all（力矩部分）
    for (int i = 0; i < data_cols; ++i) {
        M_all.segment(i * 3, 3) = ft_data.col(i).tail(3);
    }
    // 构造F矩阵
    Eigen::MatrixXd F_mat_1(3 * data_cols, 6);
    for (int i = 0; i < data_cols; ++i) {
        F_mat_1.block(i * 3, 0, 3, 6) = matTrans(ft_data.col(i).head(3));
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu_f(F_mat_1);
    int F_mat_rank = lu_f.rank();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_f(F_mat_1);
    Eigen::VectorXd sing_vals_f = svd_f.singularValues();
    double cond_num_f = sing_vals_f(0) / sing_vals_f(sing_vals_f.size() - 1);
    p = F_mat_1.colPivHouseholderQr().solve(M_all);
    Eigen::VectorXd residual_f = F_mat_1 * p - M_all;

    std::cout << "质心解析：矩阵秩= " << F_mat_rank << ", 条件数= " << cond_num_f << std::endl;
    std::cout << "残差: " << residual_f.transpose() << std::endl;

    // 加权最小二乘
    Eigen::VectorXd weight_f = 1 / (residual_f.array().square() + 1e-8);
    Eigen::MatrixXd W_F = weight_f.asDiagonal();
    Eigen::MatrixXd FWF = F_mat_1.transpose() * W_F * F_mat_1;
    Eigen::MatrixXd FWM = F_mat_1.transpose() * W_F * M_all;
    p = FWF.ldlt().solve(FWM);
    residual_f = F_mat_1 * p - M_all;
    std::cout << "加权最小二乘残差: " << residual_f.transpose() << std::endl;

    // 构造R矩阵(用于求零点)
    Eigen::MatrixXd R_mat_1(3 * data_cols, 6);
    for (int i = 0; i < data_cols; ++i) {
        Eigen::MatrixXd R_mat_temp(3, 6);
        R_mat_temp.block(0, 0, 3, 3) = rotation_list[i].transpose();
        R_mat_temp.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
        R_mat_1.block(i * 3, 0, 3, 6) = R_mat_temp;
    }
    for (int i = 0; i < data_cols; ++i) {
        F_all.segment(i * 3, 3) = ft_data.col(i).head(3);
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu_R(R_mat_1);
    int R_mat_rank = lu_R.rank();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_R(R_mat_1);
    Eigen::VectorXd sing_vals_R = svd_R.singularValues();
    double cond_num_R = sing_vals_R(0) / sing_vals_R(sing_vals_R.size() - 1);
    l = R_mat_1.colPivHouseholderQr().solve(F_all);
    Eigen::VectorXd residual_R = R_mat_1 * l - F_all;
    std::cout << "负载重力矩阵秩= " << R_mat_rank << ", 条件数= " << cond_num_R << std::endl;
    std::cout << "残差: " << residual_R.transpose() << std::endl;

    // 加权最小二乘
    Eigen::VectorXd weight_R = 1 / (residual_R.array().square() + 1e-9);
    Eigen::MatrixXd W_R = weight_R.asDiagonal();
    Eigen::MatrixXd RWR = R_mat_1.transpose() * W_R * R_mat_1;
    Eigen::MatrixXd RWF = R_mat_1.transpose() * W_R * F_all;
    l = RWR.ldlt().solve(RWF);
    residual_R = R_mat_1 * l - F_all;
    std::cout << "加权最小二乘残差: " << residual_R.transpose() << std::endl;

    id_param_.L = l.head(3);
    id_param_.G = id_param_.L.norm();
    id_param_.U = std::asin(-l(1) / id_param_.G);
    id_param_.V = std::atan2(-l(0), l(2));
    id_param_.zero_point.head(3) = l.tail(3);

    id_param_.zero_point(3) = p(3) - l(4) * p(2) + l(5) * p(1);
    id_param_.zero_point(4) = p(4) - l(5) * p(0) + l(3) * p(2);
    id_param_.zero_point(5) = p(5) - l(3) * p(1) + l(4) * p(0);
    id_param_.mass_center = p.head(3);

    std::cout << "辨识结果: \n重力 G=" << id_param_.G << "\n负载质量(kg)=" << id_param_.G / 9.80665 << std::endl;
    std::cout << "重心位置=" << id_param_.mass_center.transpose() << std::endl;
    std::cout << "零点偏移=" << id_param_.zero_point.transpose() << std::endl;
    std::cout << "安装角 U=" << id_param_.U << ", V=" << id_param_.V << std::endl;
}

Vector6d ForceTorqueGravityCompensator::compensate(const Vector6d& ft_data_ori, const Eigen::Matrix3d& Rotation,bool debug) {
    Vector6d G_ft;
    if(debug)
        std::cout << "负载基坐标力分量: " << id_param_.L.transpose() << std::endl;

    G_ft.head(3) = Rotation.transpose() * id_param_.L;

    Eigen::Matrix3d G_cross;
    G_cross << 0, -id_param_.mass_center(2), id_param_.mass_center(1),
        id_param_.mass_center(2), 0, -id_param_.mass_center(0),
        -id_param_.mass_center(1), id_param_.mass_center(0), 0;
    G_ft.tail(3) = G_cross * G_ft.head(3);
    Vector6d compensated = ft_data_ori - id_param_.zero_point - G_ft;

    if(debug)
    {
        std::cout << "补偿前力数据: " << ft_data_ori.transpose() << std::endl;
        std::cout << "重力补偿项: " << G_ft.transpose() << std::endl;
        std::cout << "补偿后力数据: " << compensated.transpose() << std::endl;
    }


    return compensated;
}
