#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#pragma execution_character_set("utf-8")

#include <QMainWindow>
#include <QLabel>
#include <QTime>
#include <QSettings>
#include "QDebug"
#include "loghandler.h"
#include <QStandardItemModel>
#include <QStandardItem>
#include <QSignalMapper>
#include <vector>
#include "zmotion.h"
#include "zmcaux.h"
#include "zmotioncontrol.h"
#include "stlfileloader.h"
#include "ddr6robotwidget.h"
#include <QMessageBox>
#include <cstdlib>
#include <QRandomGenerator>
#include <QThread>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "RobotArm.h"
#include "CollisionChecker.h"
#include "PlaneAlignmentCompensator.h"
#include "robot_types.h"
#include "TrajectoryOptimizer.h"
#include <QFile>
#include <string>
// #include <QtSerialPort/QSerialPort>
// #include <QtSerialPort/QSerialPortInfo>
// #include <QSerialPort>
#include "LanserSerialPortManager.h"
#include <QtEndian>
#include <QTextStream>
#include <QStandardPaths>  
#include <Eigen/StdVector>  
#include <QFileDialog>
#include <QFileInfo>
#include <filesystem>
#include <algorithm>
#include "curve_processor_eigen.h"
#include "dahengTwoCams_qt_vs.h"
#include "CameraCalibration.h"
//视觉
#include "CameraParamsCalibration.h"
#include "HandEyeCalibration.h"
#include "TCPCalibrator.h"
#include "FeatureDetector.h"
#include "forceform.h"
//力控
#include "admittancecontrollerform.h"
using namespace Eigen;
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
    

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private slots:

    void on_btnBackward_pressed(int i);

    void on_btnForward_pressed(int i);

    void on_btnZero_clicked(int i);

    void on_btnInit_clicked(int i);

    void on_btn_released(int i);

    void on_btnInitBus_clicked();

    void on_btnConnectSimulation_clicked();

    void on_btnConnect_clicked();

    void on_btnEnableAxis_clicked();

    void on_btnDisableAxis_clicked();

    void on_btnDisconnect_clicked();

    void UpdateUILog(QString str);

    void UpdateUIChanged();

    void on_btnAxisConfirm_clicked();

    void on_btnAxisSave_clicked();

    void on_cBoxAxisNum_editTextChanged(const QString &arg1);

    void onMvGetTimerOut();

    void on_btnInitAxis_clicked();

    void on_btnType0_clicked();

    void on_btnType65_clicked();

    void on_btnXYZ_Pressed(int i);

    void on_btnXYZ_Released(int i);

    void on_btnEmergencyStop_clicked();

    ////////////串口函数/////////////////////
    void on_btnOpenCOM_clicked();
    // 发送数据的函数
    void on_btnSendData_clicked();
    //刷新串口
    void on_btnRefreshPortsbtn_clicked();
    // void SeriesTimer_end();
    // 获取到数据
    // void receiveInfo();
    /////////////////////////////////////////////
    void on_btnAutoSendData_clicked();

    void on_btnSaveDH_clicked();

    void on_btnSetDH_clicked();

    void on_btnLoadData_clicked();

    void on_btnCurrentPosition_clicked();

    void on_btnLastPosition_clicked();

    void on_btnNextPosi_clicked();

    void on_btnClearData_clicked();

    void on_btnClearDH_clicked();

    void on_spbDHNum_editingFinished();

    void on_btnOpenData_clicked();

    void on_btnSetAxisContinuity_clicked();

    void on_btnSetAxisNotContinuity_clicked();

    void on_btnExecuteTrajectory_clicked();

    void on_btnRecordUpperBlock_clicked();

    void on_btnRecordLowerBlock_clicked();

    void on_btnSaveBlock_clicked();

    void on_btnSetRealBlock_clicked();

    void on_spbBlockGroupNum_editingFinished();

    void on_btnLoadSWCurve_clicked();

    void on_btnCalcTrajectory_clicked();

    void on_btnMoveReverseTrajectory_clicked();

    void on_btnMoveTrajectory_clicked();

    void on_btnTestChess_clicked();

    void on_btnTestCircle_clicked();

    void on_btnOpenPhoto_clicked();

    void on_btnRecordCircleUpperBlock_clicked();

    void on_btnRecordCircleLowerBlock_clicked();

    void on_btnSaveCircleBlock_clicked();

    void on_btnSetRealCircleBlock_clicked();

    void on_spbBlockGroupCircleNum_editingFinished();

    void on_btnCalculatePlaneAlignmentTrajectory_clicked();

    void on_btnPlaneAlignment_clicked();

    void on_btnLoadCalcTrajectory_clicked();

    void on_pushButton_clicked();

    void on_btnCameraParamsCalibrator_clicked();

    void on_btnTCPCalibration_clicked();

    void on_btnEyeInHandCalibration_clicked();

    void on_btnSaveCalibrationData_clicked();

    void on_btnGetShuaTraj_clicked();

    void on_btnGetTieTraj_clicked();



private:
    Ui::MainWindow *ui;
    //初始化ui文件
    void btnInit();
    void iniInit();
    void timerInit();
    QTimer* mvGetTimer = nullptr;//ui刷新定时器
    void setLED(QLabel* label, int color, int size);//ui灯
    QSettings *iniRead;//ini文件管理
    /////////////////正运动//////////////////////
    ZMotionControl* zm;
    void ZMotionInit();
    //回调函数
    void handleDataFetched(ZmotionStatus* packet);
    AxisStatus *allAxisStatus;//当前正运动控制器参数
    AxisStatus *pastallAxisStatus;//上一个正运动控制器参数
    uint32 statusIOEmergencyStop=-1;//急停
    ///////////////激光测距仪相关//////////////////
    Eigen::Matrix4d adjustedPose;
    ////////////////轨迹规划//////////////////////
    void robotInit();
    void MDHInit();
    RobotArm* arm;//正逆解
    CollisionSystem* collision_system;//碰撞检测
    std::unique_ptr<TrajectoryOptimizer> GAOptimizer;//轨迹规划优化器
    PlaneAlignmentCompensator* compensator;//激光传感器垂直平面
    CurveProcessor* processor=nullptr;//曲线处理
    std::unique_ptr<AsyncOptimizeTask> asyncOptimizeTask;

    // 实时机械臂位姿
    RobotPose robotPose;
    QTimer* jogTimer = nullptr;
    int currentDirection=0;
    void onJogTimerTimeout();
    Point6D lastTargetPose;  // 上个目标的Point6D (理想位姿)
    std::array<double, 8> pendingThetas;  // 上个目标关节 (for 检查)
    bool isWaitingForArrival = false;

    std::vector<std::array<double, 8>> jointTrajectory;//关节轨迹
    std::vector<Eigen::Matrix4d> trajectory;//轨迹
    bool isOptimizing=false;//是否在做轨迹规划
    std::vector<std::array<double, 3>> excel_xyz;

    //串口
    void SerialPortInit();
    LanserSerialPortManager *m_serialManager;
    void onDataReceived(QString hexData);
    void onErrorReceived(const QString &error);
    void onDataUpdated(const QVector<double> value);

    //力控实验
    void AdmittanceControllerInit();
    AdmittanceControllerForm *admittanceControllerForm;

    //六维力传感器串口
    ForceForm *forceForm;
    // QSerialPort* m_serialPort; //串口类
    // QStringList m_portNameList;
    // QTimer *Serialtimer;
    // float hexToFloat(const QString &hexStr);
    // float stationValues[4];
    // QByteArray convertHexStringToByteArray(const QString &hexStr);
    // QByteArray m_receiveBuffer;
    // void processModbusFrame();
    // void processReadHoldingRegisters();
    // bool validateModbusCRC(const QByteArray &data);
    // QElapsedTimer m_frameTimer;
    // const int FRAME_TIMEOUT_MS = 50; // Modbus RTU典型帧间隔
    // // 获取所有可用的串口列表
    // QStringList getPortNameList();//
    // quint16 calculateCRC(const QByteArray &data);


    ///////////////////相机///////////////////
    void cameraInit();//视觉初始化
    dahengTwoCams_qt_vs* dahengForm;
    ///////////////////视觉///////////////////
    CameraParamsCalibrator* cameraParamsCalibrator;
    HandEyeCalibrator* handEyeCalibration;
    TCPCalibrator* tCPCalibrator;
    FeatureDetector* featureDetector;
    CalibrationResult calibrationResult;
    CameraParams cameraParams;
    ChessboardParams chessboardParams;
    FeatureDetector::DetectionParams detectionParams;

    CameraCalibration* calibrator;



signals:
    void UpdateUI(QString str);
    void positionUpdated(const RobotPose& pose);
    void sendTrajectory(const std::vector<Eigen::Matrix4d> &trajectory);
    void isIdle(bool idle);
    void sendCurrentEndMoveJoint(const std::array<double, 8> currentEndMoveJoint);
public slots:
    //用于调用的轨迹规划
    void onCaculateTrajectoryPoints(const std::vector<Eigen::Matrix4d>& trajPoints);
    //取消轨迹规划计算
    void onCancelCaculateTrajectory();
    //是否轨迹规划成功
    bool onTrajectoryPlanningFinished(std::vector<std::array<double, 8>>& outJointTrajectory);
    //运动
    void onMoveJointTraj(const std::vector<std::array<double, 8>>& jointTraj);
    void getIdle();
    void getCurrentEndMoveJoint();

    void onRequestTrajectory();  // 响应子页面请求
    //获取当前坐标
    void getPosition();
};

#endif // MAINWINDOW_H
