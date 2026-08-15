#pragma once

#include <Eigen/Dense>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include "AdmittanceController.h"
#include "TrajectoryOptimizer.h"
struct SensorDataFrame {
    double timestamp_ms;
    std::array<double, 6> sensorData;
    std::array<double, 8> jointPos;
    Point6D admittanceOffset;
    std::array<double, 3> targetPos;
};
/**
 * @brief 力控执行器，负责导纳控制+轨迹规划的循环执行
 *
 * 构造时传入导纳控制器和轨迹规划器对象指针（外部管理生命周期）
 * 运动发送和等待由外部通过回调实现，保持解耦
 */
class ForceControlExecutor
{
public:
    struct ControlStatus {
        size_t current_index = 0;
        Eigen::Matrix4d plannedPose = Eigen::Matrix4d::Identity();  // 当前轨迹参考位姿
        Point6D admittanceOffset;                                    // 当前导纳偏移（绝对）
        Eigen::Matrix4d actualTargetPose = Eigen::Matrix4d::Identity(); // 叠加偏移后实际目标位姿
        std::string toString() const {
            std::ostringstream oss;
            oss << "PointIndex: " << (current_index + 1)
                << " | Offset (m, rad): ["
                << admittanceOffset.x << ", "
                << admittanceOffset.y << ", "
                << admittanceOffset.z << ", "
                << admittanceOffset.rx << ", "
                << admittanceOffset.ry << ", "
                << admittanceOffset.rz << "]";

            oss << " | Planned Pos (mm): ["
                << plannedPose(0, 3) << ", "
                << plannedPose(1, 3) << ", "
                << plannedPose(2, 3) << "]";

            oss << " | Actual Pos (mm): ["
                << actualTargetPose(0, 3) << ", "
                << actualTargetPose(1, 3) << ", "
                << actualTargetPose(2, 3) << "]";

            return oss.str();
        }
    };
    using JointTrajectory = std::vector<std::array<double, 8>>;
    using LogCallback = std::function<void(const std::string&)>;
    using SendMotionFunc = std::function<void(const JointTrajectory&)>;
    using GetCurrentJointsFunc = std::function<std::array<double, 8>()>;  // 新增回调类型

    ForceControlExecutor(AdmittanceController* admittanceController,
        TrajectoryOptimizer* trajectoryOptimizer);
    ~ForceControlExecutor();

    // 禁止复制和赋值
    ForceControlExecutor(const ForceControlExecutor&) = delete;
    ForceControlExecutor& operator=(const ForceControlExecutor&) = delete;

    // 设置控制周期，单位毫秒，默认50ms
    void setControlPeriod(int period_ms);

    // 设置预定轨迹（弧形等路径），线程安全
    void setPlannedTrajectory(const std::vector<Eigen::Matrix4d>& traj);

    std::vector<Eigen::Matrix4d>getPlannedTrajectory(){return plannedTrajectory_;}

    void setCurrentTrajectoryIndex(int index){currentTrajectoryIndex_=index;}

    // 外部更新传感器数据，线程安全
    void updateSensorData(const Eigen::Matrix<double, 6, 1>& data);

    // 设置回调
    void setLogCallback(LogCallback cb);
    void setSendMotionCallback(SendMotionFunc func);
    void setIsMotionCompleteCallback(std::function<bool()> func);
    void setGetCurrentJointsCallback(GetCurrentJointsFunc func);  // 新增设置回调接口

    // 启动和停止控制线程
    void start();
    void stop();

    // 线程状态查询
    bool isRunning() const { return running_.load(); }
    // 获取当前控制状态（线程安全）
    ControlStatus getControlStatus() const;

    using StatusCallback = std::function<void(const std::string&)>;
    void setStatusCallback(StatusCallback cb);
    std::vector<std::array<double, 8>> computeSingleForceControlJoint();
    std::vector<bool> detectTurningPointsFromTrajectory(const std::vector<Eigen::Matrix4d> &plannedTrajectory, double turning_angle_threshold=0.1);

    size_t estimateCurrentExecutedIndex();
    void saveSensorDataToCSV();
private:
    void controlLoop();
    AdmittanceController* admittanceController_;
    TrajectoryOptimizer* trajectoryOptimizer_;

    std::atomic<bool> running_;
    std::thread controlThread_;

    // 控制参数
    int controlPeriodMs_ = 1000;
    Eigen::Matrix4d globalOffset_ = Eigen::Matrix4d::Identity();
    std::vector<bool> turningPoints_; // 用detectTurningPointsFromTrajectory计算后赋值

    mutable std::mutex sensorDataMutex_;
    Eigen::Matrix<double, 6, 1> latestSensorData_;

    mutable std::mutex trajMutex_;
    std::vector<Eigen::Matrix4d> plannedTrajectory_;
    size_t currentTrajectoryIndex_ = 0;
    size_t lastExecutedQueueIdx_ = 0;  // 上次确认执行到的队列索引

    // 回调函数
    LogCallback logCallback_;
    SendMotionFunc sendMotionFunc_;
    std::function<bool()> checkMotionCompleteFunc_;
    GetCurrentJointsFunc getCurrentJointsFunc_;  // 新增回调成员

    // Data logging
    std::vector<SensorDataFrame> sensorDataLog_;
    std::chrono::steady_clock::time_point controlStartTime_;

    // 类成员变量（建议 std::atomic<bool> 线程安全）
    std::atomic<bool> motionInProgress_{false};
    // 安全日志打印
    void log(const std::string& msg);

    mutable std::mutex queueMutex_;
    std::deque<std::array<double, 8>> sentJointQueue_;
    size_t sentJointQueueGlobalStartIndex_ = 0;

    mutable std::mutex statusMutex_;
    ControlStatus currentStatus_;
    StatusCallback statusCallback_;
    bool waitForMotionCompletion(std::function<bool ()> isMotionComplete, int timeoutMs, int pollIntervalMs = 10);
    bool waitForMotionStart(std::function<bool ()> isMotionComplete, int timeoutMs, int pollIntervalMs=10);
    void reorthogonalizeMatrix(Eigen::Matrix4d &mat);
    Point6D toPoint6D(const Eigen::Vector3d &pos, const Eigen::Vector3d &rot);
    Eigen::Matrix4d toEigenMatrix(const Point6D &offset);
    std::string getCurrentTimestamp();
};
