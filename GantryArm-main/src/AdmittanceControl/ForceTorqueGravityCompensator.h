#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct identification_param
{
    double G;
    Eigen::Vector3d L;
    double U, V;
    Eigen::Vector3d mass_center;
    Vector6d zero_point;
};

class ForceTorqueGravityCompensator {
public:
    ForceTorqueGravityCompensator();

    // /** 从文件加载旋转矩阵列表（RPY格式txt） */
    // std::vector<Eigen::Matrix3d> loadRotation(const std::string& path, char delimiter = ',');

    // /** 从文件加载六维力矩数据，返回6*N矩阵 */
    // Eigen::MatrixXd loadForceTorque(const std::string& path, char delimiter = ',');

    /** 根据传感器六维数据和旋转矩阵，进行负载重力参数辨识 */
    void identifyGravityParams(const Eigen::MatrixXd& ft_data, const std::vector<Eigen::Matrix3d>& rotation_list);

    /** 输入6维原始力数据和旋转矩阵，返回补偿后的6维力数据 */
    Vector6d compensate(const Vector6d& ft_data_ori, const Eigen::Matrix3d& Rotation,bool debug=false);

    /** 获取辨识结果 */
    const identification_param& getIdentificationParam() const { return id_param_; }

    //设置辨识结果
    void setIdentification_param(identification_param param){id_param_=param;}

private:
    identification_param id_param_;

    /** 从三维向量构建4x6的矩阵块（反对称矩阵和单位矩阵部分），用于计算质心 */
    Eigen::MatrixXd matTrans(const Eigen::Vector3d& v);

    // /** 辅助：读取txt文件中的单行数值向量 */
    // std::vector<double> parseLineToDoubles(const std::string& line, char delimiter);

};
