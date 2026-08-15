#pragma once
#include "robot_types.h" // 包含你提供的头文件
#include <vector>
#include <optional>
#include <string>
#include "calibration_types.h"
// TCP标定数据点
struct CalibrationPoint {
    Point6D robot_flange_pose;        // 机械臂末端法兰位姿
    Point6D theoretical_tcp_pose;     // 基于理论TCP计算出的TCP位姿

    CalibrationPoint() = default;

    CalibrationPoint(const Point6D& flange_pose, const Point6D& theoretical_tcp);
};

class TCPCalibrator {
private:
    std::vector<CalibrationPoint> calibration_points_;
    std::optional<TCPResult> current_tcp_result_;
    Point6D theoretical_tcp_;        // 理论TCP偏移
    Point6D reference_point_;        // 固定参考点位置
    bool reference_point_set_;       // 参考点是否已设置

    // 计算参考点位置（所有TCP理论位置的质心）
    Point6D calculateReferencePoint() const;

    // 计算RMS误差
    double calculateRMSError(const Point6D& actual_tcp) const;

public:
    TCPCalibrator();
    // 设置理论TCP偏移（相对于法兰坐标系）
    void setTheoreticalTCP(const Point6D& tcp_offset);
    void setTheoreticalTCP(double x, double y, double z, double rx = 0, double ry = 0, double rz = 0);

    // 设置固定参考点（如果已知的话）
    void setReferencePoint(const Point6D& ref_point);
    void setReferencePoint(double x, double y, double z);

    // 添加标定姿态（机械臂到达参考点的不同姿态）
    void addCalibrationPose(const Point6D& robot_tcp_pose);

    // 执行TCP标定
    std::optional<TCPResult> calibrateTCP();

    // 保存标定数据到CSV
    bool saveCalibrationData(const std::string& filename) const;

    // 从CSV文件加载标定数据
    bool loadCalibrationData(const std::string& filename);

    // 保存TCP标定结果
    bool saveTCPResult(const std::string& filename) const;

    // 加载TCP标定结果
    bool loadTCPResult(const std::string& filename);

    // 获取当前TCP结果
    std::optional<TCPResult> getCurrentTCPResult() const;

    // 获取理论TCP
    Point6D getTheoreticalTCP() const;

    // 获取参考点
    std::optional<Point6D> getReferencePoint() const;

    // 清除所有数据
    void clearAll();

    // 清除标定数据但保留设置
    void clearCalibrationData();

    // 获取标定点数量
    size_t getCalibrationPointCount() const noexcept;

    // 验证TCP精度
    double validateTCP() const;

    // 使用标定后的TCP计算实际TCP位置
    Point6D calculateActualTCPPose(const Point6D& flange_pose) const;

    // 打印标定信息
    void printCalibrationInfo() const;

    // 打印所有标定点
    void printCalibrationPoints() const;
    bool loadCalibrationPosesFromCSV(const std::string &filename);
};
