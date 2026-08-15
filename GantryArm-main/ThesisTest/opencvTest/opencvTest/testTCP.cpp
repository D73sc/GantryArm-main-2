//#include "TCPCalibrator.h"
//#include <iostream>
//
//int main() {
//    TCPCalibrator calibrator;
//
//    std::cout << "=== 正确的TCP标定流程 ===\n";
//
//    // 1. 设置理论TCP偏移（比如根据工具的CAD尺寸）
//    calibrator.setTheoreticalTCP(0, 0, 150.0); // 工具长度150mm，沿Z轴
//
//    // 2. 可选：设置已知的参考点（如果不设置会自动计算）
//    calibrator.setReferencePoint(500.0, 300.0, 200.0);
//
//    // 3. 添加不同姿态的标定数据（都接触同一个参考点）
//    calibrator.addCalibrationPose(Point6D(500.0, 300.0, 350.0, 0, 0, 0));
//    calibrator.addCalibrationPose(Point6D(480.0, 320.0, 370.0, degToRad(30), degToRad(15), degToRad(45)));
//    calibrator.addCalibrationPose(Point6D(520.0, 280.0, 360.0, degToRad(-15), degToRad(30), degToRad(-30)));
//    calibrator.addCalibrationPose(Point6D(460.0, 340.0, 380.0, degToRad(45), degToRad(-20), degToRad(60)));
//
//    // 4. 执行标定
//    if (auto result = calibrator.calibrateTCP()) {
//        std::cout << "\nTCP标定成功!\n";
//        calibrator.printCalibrationInfo();
//
//        // 保存结果
//        calibrator.saveTCPResult("tcp_result.csv");
//
//        // 5. 验证标定结果
//        Point6D test_pose(510.0, 290.0, 365.0, degToRad(20), degToRad(10), degToRad(25));
//        calibrator.validateTCP(test_pose);
//    }
//    else {
//        std::cout << "TCP标定失败!\n";
//    }
//
//    return 0;
//}