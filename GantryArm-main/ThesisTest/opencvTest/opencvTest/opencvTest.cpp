//#include <iostream>
//#include "CameraCalibration.h"
//int main()
//{
//    CameraCalibration* calibrator = new CameraCalibration();
//    CameraCalibration::ChessboardParams params;
//    params.boardSize = cv::Size(8, 11);
//    params.squareSize = 6.0f;
//    calibrator->setChessboardParams(params);
//    std::string xmlPath = "data/calibration/camera_params.xml";
//    calibrator->loadCalibrationData(xmlPath);
//    calibrator->loadHandEyeData("data/calibration/hand_eye_calibration.xml");
//}
////棋盘格检测TCP、内参
////void MainWindow::on_btnTestChess_clicked()
////{
////    // QString currentPath = QDir::currentPath();
////    // QString dataQFile = QDir(currentPath).filePath("lib/data/photos/calibration/");
////    // // 2. 对特定图像检测角点
////    // cv::Mat image = cv::imread(dataQFile.toStdString()+"13.jpg");
////
////    cv::Mat image = cv::imread(ui->LineEditPhotoAddress->text().toStdString());
////
////    std::vector<cv::Point2f> imagePoints;
////    bool found = calibrator->detectFeaturePoints(image, imagePoints, "CHESSBOARD");
////
////    if (found) {
////        // 3. 将图像坐标转换到相机坐标系
////        std::vector<cv::Point3f> cameraPoints = calibrator->pixel2Camera(imagePoints, 115);
////        getPosition();
////        Eigen::Vector3d camera(80.05, -7.73, -115);          // 相机位置
////
////        // 构建相机相对于末端执行器的变换矩阵
////        Eigen::Matrix4d T_end_camera = Eigen::Matrix4d::Identity();
////        T_end_camera.block<3, 1>(0, 3) = camera;  // 设置平移部分
////
////        // 右乘得到相机相对于基座的变换
////        Eigen::Matrix4d T_base_camera_eigen = current_pose * T_end_camera;
////
////        // 转换为 cv::Mat
////
////        cv::Mat T_base_camera = cv::Mat::zeros(4, 4, CV_64F);
////        for (int i = 0; i < 4; i++) {
////            for (int j = 0; j < 4; j++) {
////                T_base_camera.at<double>(i, j) = T_base_camera_eigen(i, j);
////            }
////        }
////        // 5. 转换到机械臂基座标系
////        std::vector<cv::Point3f> basePoints = calibrator->camera2Base(cameraPoints, T_base_camera_eigen);
////
////        // 6. 可视化结果
////        calibrator->visualizeResults(image, imagePoints, basePoints, T_base_camera);
////    }
////}
////
////为了后续配准的圆心检测
////void MainWindow::on_btnTestCircle_clicked()
////{
////    if (ui->LineEditPhotoAddress->text() == "")
////        return;
////
////    cv::Mat image = cv::imread(ui->LineEditPhotoAddress->text().toStdString());
////
////    // QString currentPath = QDir::currentPath();
////    // QString dataQFile = QDir(currentPath).filePath("lib/data/photos/calibration/");
////    // cv::Mat image = cv::imread(dataQFile.toStdString()+"C1.jpg");
////
////
////    std::vector<cv::Point2f> imagePoints;
////    bool found = calibrator->detectFeaturePoints(image, imagePoints, "CIRCLE");
////
////    if (found) {
////        // 3. 将图像坐标转换到相机坐标系
////        std::vector<cv::Point3f> cameraPoints = calibrator->pixel2Camera(imagePoints, 115);
////        getPosition();
////        Eigen::Vector3d camera(80.05, -7.73, -115);          // 相机位置
////
////        //// 构建相机相对于末端执行器的变换矩阵
////        //Eigen::Matrix4d T_end_camera = Eigen::Matrix4d::Identity();
////        //T_end_camera.block<3, 1>(0, 3) = camera;  // 设置平移部分
////
////        //// 右乘得到相机相对于基座的变换
////        //Eigen::Matrix4d T_base_camera_eigen = current_pose * T_end_camera;
////
////        //// 转换为 cv::Mat
////
////        //cv::Mat T_base_camera = cv::Mat::zeros(4, 4, CV_64F);
////        //for (int i = 0; i < 4; i++) {
////        //    for (int j = 0; j < 4; j++) {
////        //        T_base_camera.at<double>(i, j) = T_base_camera_eigen(i, j);
////        //    }
////        //}
////        // 5. 转换到机械臂基座标系
////        // std::vector<cv::Point3f> basePoints = calibrator->camera2Base(cameraPoints, T_base_camera_eigen);
////        auto basePoints = calibrator->pixelToRobotBase(imagePoints, CameraCalibration::eigenToCvMat(current_pose));
////
////        // 6. 可视化结果
////        //calibrator->visualizeResults(image, imagePoints, basePoints, T_base_camera);
////        std::stringstream ss;
////        ss << "Base Points (" << basePoints.size() << " points):\n";
////        ss << std::fixed << std::setprecision(3);
////
////        for (size_t i = 0; i < basePoints.size(); ++i) {
////            ss << "[" << i << "] ("
////                << basePoints[i].x << ", "
////                << basePoints[i].y << ", "
////                << basePoints[i].z << ")\n";
////        }
////
////        // 直接在调用时转换
////        UpdateUI(QString::fromStdString(ss.str()));
////    }
////
////}
//
//
////应该存在问题，写一个结果验证
////void MainWindow::on_btn_EyeInHandCalibration_clicked()
////{
////    // 准备手眼标定数据
////    std::vector<cv::Mat> handEyeImages;
////    std::vector<cv::Mat> robotPoses;
////    std::vector<Eigen::Matrix4d> eigenPoses = CameraCalibration::readPosesFromCSV("lib/data/calibration/robot_poses.csv");
////
////    // 加载图像和对应的机器人位姿
////    for (int i = 0; i < 17; i++) {
////        std::cout << "lib/data/calibration/00" + std::to_string(i) + ".jpg";
////        cv::Mat img = cv::imread("lib/data/calibration/00" + std::to_string(i) + ".jpg");
////        handEyeImages.push_back(img);
////
////        // 机器人位姿 (4x4变换矩阵)
////        cv::Mat pose = CameraCalibration::eigenToCvMat(eigenPoses[i]);
////        robotPoses.push_back(pose);
////    }
////
////    // 执行手眼标定
////    bool success = calibrator->performHandEyeCalibration(handEyeImages, robotPoses);
////
////    if (success) {
////        // 保存结果
////        calibrator->saveHandEyeData("lib/data/calibration/hand_eye_calibration.xml");
////
////        // 使用结果进行坐标变换
////        std::vector<cv::Point2f> pixelPoints = { cv::Point2f(320, 240) };
////        cv::Mat currentPose = CameraCalibration::eigenToCvMat(eigenPoses[0]); // 当前机器人位姿
////
////        auto basePoints = calibrator->pixelToRobotBase(pixelPoints, currentPose);
////
////        std::cout << "机器人基座坐标: " << basePoints[0] << std::endl;
////    }
////}
