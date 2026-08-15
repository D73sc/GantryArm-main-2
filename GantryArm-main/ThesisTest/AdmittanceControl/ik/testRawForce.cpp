//#include <Eigen/Dense>
//#include <fstream>
//#include <sstream>
//#include <vector>
//#include <iostream>
//using Pose = Eigen::Isometry3d;
//
//using Vector6d = Eigen::Matrix<double, 6, 1>;
//
///**
// * @brief 负载参数结构体
// * G 负载重量，L 负载在基座标系下XYZ3个方向上的分量
// * U,V 基座标系安装的rx、ry的倾角
// * mass_center 负载在六维力传感器下的质心
// * zero_point 六维力传感器的零点
// */
//struct identification_param
//{
//    double G;
//    Eigen::Vector3d L;
//    double U, V;
//    Eigen::Vector3d mass_center;
//    Vector6d zero_point;
//};
//
//
///**
// * @brief 从txt文件获取Rotation数据，形式为N*3,N其实代表了采集了多少个点，由RPY转为旋转矩阵
// *
// * @param path txt文件路径
// * @param delimiter 每行数据间的分隔符
// * @return std::vector<Eigen::Matrix3d> 返回旋转矩阵列表
// */
//std::vector<Eigen::Matrix3d>  load_rotation(const std::string& path, char delimiter = ',')
//{
//    std::vector<Eigen::Matrix3d> rotation_list;
//    std::ifstream file(path);
//    if (!file.is_open()) throw std::runtime_error("无法打开文件: " + path);
//
//    std::string line; //用于保存从文件中读取的一整行文本
//    while (std::getline(file, line)) {
//        std::stringstream ss(line);//创建一个字符串流，把整行字符串变成一个“可流式读取”的输入流。
//        std::string value;
//        std::vector<double> values;
//        while (std::getline(ss, value, delimiter)) values.push_back(std::stod(value));//从字符串流中按 分隔符（如 ','）读取出一个个字符串
//        // 将rpy再转为旋转矩阵
//        double roll = values[0], pitch = values[1], yaw = values[2];
//        Eigen::Matrix3d Rx = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
//        Eigen::Matrix3d Ry = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
//        Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
//        Eigen::Matrix3d rotation = Rz * Ry * Rx;
//        rotation_list.push_back(rotation);
//    }
//    return rotation_list;
//}
//
//
//
///**
// * @brief 从txt文件获取FT数据 形式为N*6
// *
// * @param path txt文件路径
// * @param delimiter 每行数据间的分隔符
// * @return Eigen::MatrixXd 返回的FT数据矩阵并更改其格式为6*N
// */
//Eigen::MatrixXd load_ft(const std::string& path, char delimiter = ',')
//{
//    std::ifstream file(path);
//    if (!file.is_open()) throw std::runtime_error("无法打开文件: " + path);
//
//    std::vector<Eigen::VectorXd> rows;
//    std::string line; //用于保存从文件中读取的一整行文本
//    while (std::getline(file, line)) {
//        std::stringstream ss(line);//创建一个字符串流，把整行字符串变成一个“可流式读取”的输入流。
//        std::string value;
//        std::vector<double> values;
//        while (std::getline(ss, value, delimiter)) values.push_back(std::stod(value));//从字符串流中按 分隔符（如 ','）读取出一个个字符串
//        Eigen::VectorXd row(values.size());
//        for (size_t i = 0; i < values.size(); ++i) row(i) = values[i];
//        rows.push_back(row);
//    }
//
//    int n_rows = rows.size();
//    int n_cols = rows[0].size();
//    Eigen::MatrixXd mat(n_rows, n_cols);
//    for (int i = 0; i < n_rows; ++i) mat.row(i) = rows[i].transpose();
//    return mat.transpose();
//}
//
//
///**
// * @brief 由向量计算其反对称矩阵，并扩充单位矩阵为6*6，对应公式5中的矩阵块
// *
// * @param v
// * @return Eigen::MatrixXd
// */
//Eigen::MatrixXd mat_trans(const Eigen::Vector3d& v) {
//    Eigen::MatrixXd m(3, 6);
//    m << 0, v.z(), -v.y(), 1.0, 0.0, 0.0,
//        -v.z(), 0, v.x(), 0.0, 1.0, 0.0,
//        v.y(), -v.x(), 0.0, 0.0, 0.0, 1.0;
//    return m;
//}
//
//
///**
// * @brief 计算负载辨识参数
// *
// * @param ft_data 力/力矩数据矩阵 6*N
// * @param Rotation_list 旋转矩阵列表
// * @return identification_param
// */
//identification_param identification_6dofSensor_gravity(const Eigen::MatrixXd& ft_data, const std::vector<Eigen::Matrix3d>& Rotation_list)
//{
//    identification_param identification_param_;
//    int data_clos = ft_data.cols();
//    Eigen::VectorXd M_all(3 * data_clos);
//    Eigen::VectorXd F_all(3 * data_clos);
//    Eigen::VectorXd p(6);// X，Y，Z，K1,K2,K3，对应公式8
//    Eigen::VectorXd l(6);//Lx，Ly，Lz，Fx0, Fy0, Fz0， 对应公式22
//    // ----------------------计算重心坐标与k1,k2，k3---------------------------
//    // 获取力矩数据
//    for (size_t i = 0; i < data_clos; i++)
//    {
//        M_all.segment(i * 3, 3) = ft_data.col(i).tail(3);
//    }
//    //填充F矩阵 对应公式6
//    Eigen::MatrixXd F_mat_1(3 * data_clos, 6);
//    for (size_t i = 0; i < data_clos; i++)
//    {
//        F_mat_1.block(i * 3, 0, 3, 6) = mat_trans(ft_data.col(i).head(3));
//    }
//    // A X = B，分析一下A矩阵的条件数和秩，可以看出来你采集的数据分布的质量，秩评估数据信息量是否足够，列满秩则有唯一解，条件数（max奇异值/min奇异值）衡量对输入数据（包括𝐵或𝐴的误差/扰动）敏感度,越大则表明小的扰动会对解X造成大的相对变化，即数值不稳定，会放大误差
//    // 对应公式9 计算矩阵的秩、条件数、最小二乘拟合的残差    
//    Eigen::FullPivLU<Eigen::MatrixXd> lu_f(F_mat_1);
//    int F_mat_rank = lu_f.rank();
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd_f(F_mat_1);
//    Eigen::VectorXd sing_vals_f = svd_f.singularValues();//计算奇异值
//    double cond_num_f = sing_vals_f(0) / sing_vals_f(sing_vals_f.size() - 1);//计算条件数
//    p = F_mat_1.colPivHouseholderQr().solve(M_all); //最小二乘
//    Eigen::VectorXd residual_f = F_mat_1 * p - M_all;
//    std::cout << "求解质心的矩阵的秩为: " << F_mat_rank << "  条件数为：" << cond_num_f << "  \n最小二乘拟合的残差为：" << residual_f.transpose() << std::endl;
//
//    // -----------加权最小二乘：
//    Eigen::VectorXd weight_f = 1 / (residual_f.array().square() + 1e-8);
//    Eigen::MatrixXd W_F = weight_f.asDiagonal();
//    Eigen::MatrixXd FWF = F_mat_1.transpose() * W_F * F_mat_1;
//    Eigen::MatrixXd FWM = F_mat_1.transpose() * W_F * M_all;
//    p = FWF.ldlt().solve(FWM);
//    residual_f = F_mat_1 * p - M_all;
//    std::cout << "求解质心的矩阵：加权最小二乘拟合的残差为：" << residual_f.transpose() << std::endl;
//
//    // -----------------------零点计算 Lx，Ly，Lz，Fx0, Fy0, Fz0------------------------------
//    // 填充R矩阵 对应公式20
//    Eigen::MatrixXd R_mat_1(3 * data_clos, 6);
//    for (int i = 0; i < data_clos; i++)
//    {
//        Eigen::MatrixXd R_mat_temp(3, 6);
//        R_mat_temp.block(0, 0, 3, 3) = Rotation_list[i].transpose();
//        R_mat_temp.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
//        R_mat_1.block(i * 3, 0, 3, 6) = R_mat_temp;
//    }
//    // 获取力数据
//    for (size_t i = 0; i < data_clos; i++)
//    {
//        F_all.segment(i * 3, 3) = ft_data.col(i).head(3);
//    }
//    // 计算矩阵的秩、条件数、最小二乘拟合的残差
//    Eigen::FullPivLU<Eigen::MatrixXd> lu_R(R_mat_1);
//    int R_mat_rank = lu_R.rank();
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd_R(R_mat_1);
//    Eigen::VectorXd sing_vals_R = svd_R.singularValues();//计算奇异值
//    double cond_num_R = sing_vals_R(0) / sing_vals_R(sing_vals_R.size() - 1);//计算条件数
//    l = R_mat_1.colPivHouseholderQr().solve(F_all);//最小二乘
//    Eigen::VectorXd residual_R = R_mat_1 * l - F_all;
//    std::cout << "求解负载重力的矩阵的秩为: " << R_mat_rank << "  条件数为：" << cond_num_R << "  \n最小二乘拟合的残差为：" << residual_R.transpose() << std::endl;
//
//    // -----------加权最小二乘：
//    Eigen::VectorXd weight_R = 1 / (residual_R.array().square() + 1e-9);
//    Eigen::MatrixXd W_R = weight_R.asDiagonal();
//    Eigen::MatrixXd RWR = R_mat_1.transpose() * W_R * R_mat_1;
//    Eigen::MatrixXd RWF = R_mat_1.transpose() * W_R * F_all;
//    l = RWR.ldlt().solve(RWF);
//    residual_R = R_mat_1 * l - F_all;
//    std::cout << "求解负载重力的矩阵：加权最小二乘拟合的残差为：" << residual_R.transpose() << std::endl;
//    // ------------
//    // 索引结果
//    std::cout << "l: \n" << l.transpose() << std::endl;
//    identification_param_.L = l.head(3);
//    identification_param_.G = l.head(3).norm();
//    identification_param_.U = asin(-l(1) / identification_param_.G);
//    identification_param_.V = atan(-l(0) / l(2));
//    identification_param_.zero_point.head(3) = l.tail(3);
//
//    // 对应公式24 计算零点的扭矩
//    identification_param_.zero_point(3) = p(3) - l(4) * p(2) + l(5) * p(1);
//    identification_param_.zero_point(4) = p(4) - l(5) * p(0) + l(3) * p(2);
//    identification_param_.zero_point(5) = p(5) - l(3) * p(1) + l(4) * p(0);
//    identification_param_.mass_center = p.head(3);
//
//    // 打印输出
//    std::cout << "辨识得到的负载重力为：\n" << identification_param_.G << "\n" <<
//        "辨识得到的负载质量为：\n" << identification_param_.G / 9.80665 << "\n" <<
//        "六维力传感器坐标系下质心为:\n" << identification_param_.mass_center.transpose() << "\n" <<
//        "零点为:\n" << identification_param_.zero_point.transpose() << "\n" <<
//        "U为:\n" << identification_param_.U << "\n" <<
//        "V为:\n" << identification_param_.V << std::endl;
//    return identification_param_;
//}
//
///**
// * @brief 实时计算补偿值
// *
// * @param ft_data_ori 当前在六维力传感器坐标系下六维力数据，
// * @param Rotation 当前末端到基坐标系的旋转矩阵
// * @param identification_param_ 辨识得到的参数（负载重力,质心坐标，零点数据，U，V）
// * @return std::vector<double> 补偿后的六维力数据，在六维力传感器坐标系下
// */
//std::vector<double>  gravity_comp(const Vector6d& ft_data_ori, const Eigen::Matrix3d& Rotation, identification_param& identification_param_)
//{
//    std::vector<double> ft_data_res;
//    Eigen::VectorXd G_ft(6);
//    std::cout << "负载在基坐标系下的力分量：" << identification_param_.L.transpose() << "\n";
//
//    // 负载在六维力传感器坐标系下的XYZ力分量
//    G_ft.head(3) = Rotation.transpose() * identification_param_.L;
//
//    // 求负载在六维力传感器坐标系下的转矩分量，对应公式1
//    Eigen::Matrix3d G_cross;
//    G_cross << 0, -identification_param_.mass_center(2), identification_param_.mass_center(1),
//        identification_param_.mass_center(2), 0, -identification_param_.mass_center(0),
//        -identification_param_.mass_center(1), identification_param_.mass_center(0), 0;
//    G_ft.tail(3) = G_cross * G_ft.head(3);
//
//    // 计算补偿后的六维力数据 对应公式28、29
//    for (size_t i = 0; i < 6; i++)
//    {
//        double data = ft_data_ori(i) - identification_param_.zero_point(i) - G_ft(i);
//        ft_data_res.push_back(data);
//    }
//
//    // 打印输出
//    std::cout << "补偿前的力传感器的数值为：" << "\n";
//    for (size_t i = 0; i < ft_data_ori.size(); i++) {
//        std::cout << ft_data_ori[i] << " ";
//    }
//    std::cout << std::endl;
//
//    std::cout << "补偿后的力传感器的数值为：-----" << "\n";
//    for (double ft : ft_data_res) {
//        std::cout << ft << " ";
//    }
//    std::cout << std::endl;
//
//    Eigen::Vector3d g_ex(ft_data_res[0], ft_data_res[1], ft_data_res[2]);
//    std::cout << "末端受到的合力为" << g_ex.norm() << "\n";
//    std::cout << "末端的增加物体的质量为" << g_ex.norm() / 9.8 << "\n";
//    return ft_data_res;
//}
//
//
//
//int main(int argc, char** argv)
//{
//    // 加载力数据、旋转矩阵数据
//    auto rotation_list = load_rotation("data/rotation_data_averages.txt");
//    auto ft_list = load_ft("data/ft_data_averages.txt");
//    auto identification_param = identification_6dofSensor_gravity(ft_list, rotation_list);
//
//    // 基于 rotation_list 和 ft_list 做补偿测试，补偿结果应该是全零
//    for (int i = 0; i < rotation_list.size(); i++)
//    {
//        Vector6d ft_data = ft_list.col(i);
//        auto res = gravity_comp(ft_data, rotation_list[i], identification_param);
//    }
//    return 0;
//}
//
