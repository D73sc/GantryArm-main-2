//#include "CameraParamsCalibration.h"
//#include <iostream>
//#include <Eigen/Dense>
//#include <string>
//
//int main() {
//    // 1. 初始化标定器
//    cv::Size board_size(8, 11);  // 棋盘格内角点数量 (宽度x高度)
//    float square_size = 0.006f; // 棋盘格方块尺寸(米)
//
//    CameraParamsCalibrator calibrator(board_size, square_size);
//
//    calibrator.setFixedDepth(118);
//
//    // 2. 相机内参标定
//    std::vector<std::string> calibration_images;
//    for (int i = 0; i < 17; i++) {
//        std::string img_path = "../../../lib/data/calibration/00" + std::to_string(i) + ".jpg";
//        calibration_images.push_back(img_path);
//    }
//
//    if (!calibrator.calibrateCamera(calibration_images)) {
//        std::cerr << "相机内参标定失败" << std::endl;
//        return -1;
//    }
//    return 0;
//}
//
