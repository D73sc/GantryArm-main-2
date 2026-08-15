//#include "ForceTorqueGravityCompensator.h"
//#include <iostream>
//
//int main() {
//    ForceTorqueGravityCompensator compensator;
//
//    auto rotation_list = compensator.loadRotation("data/rotation_data_averages.txt");
//    auto ft_list = compensator.loadForceTorque("data/ft_data_averages.txt");
//    for (size_t i = 0; i < rotation_list.size(); ++i) {
//        std::cout << "Rotation matrix " << (i + 1) << ":\n";
//        std::cout << rotation_list[i] << "\n\n";
//    }
//    std::cout << ft_list << std::endl;
//
//    compensator.identifyGravityParams(ft_list, rotation_list);
//
//    for (size_t i = 0; i < rotation_list.size(); ++i) {
//        std::cout << "----- 补偿第 " << i + 1 << " 组 -----" << std::endl;
//        Vector6d ft_data = ft_list.col(i);
//        Vector6d compensated = compensator.compensate(ft_data, rotation_list[i]);
//    }
//    return 0;
//}
