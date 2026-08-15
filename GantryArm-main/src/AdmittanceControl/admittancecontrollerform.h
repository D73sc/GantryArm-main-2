#ifndef ADMITTANCECONTROLLERFORM_H
#define ADMITTANCECONTROLLERFORM_H

#include <QWidget>
#include <QFileDialog>
#include <QDebug>
#include <QSettings>
#include "AdmittanceController.h"
#include "ForceTorqueGravityCompensator.h"
#include <QMessageBox>
#include "TrajectoryOptimizer.h"
#include "ForceControlExecutor.h"
namespace Ui {
class AdmittanceControllerForm;
}

class AdmittanceControllerForm : public QWidget
{
    Q_OBJECT

public:
    explicit AdmittanceControllerForm(QWidget *parent = nullptr);
    ~AdmittanceControllerForm();

private slots:
    void on_btnOpenData_clicked();

    void on_btnLoadData_clicked();

    void on_btnLoadmmData_clicked();

    void on_btnClearPosiData_clicked();

    void on_btnSavePosiData_clicked();

    void on_btnClearSensorData_clicked();

    void on_btnSaveSensorData_clicked();

    void on_btnForceTorqueGravityCompensator_clicked();

    void on_btnClearData_clicked();

    void on_btnAdmittanceControllerOnce_clicked();

    void on_btnMoveTrajectoryOnce_clicked();

    void on_btnSaveResult_clicked();

    void on_btnLoadResult_clicked();

    void on_btnSetParams_clicked();

    void on_btnSaveParams_clicked();

    void on_btnLoadParams_clicked();

    void on_spbCurrentPosi_editingFinished();

    void on_btnStopForceExecute_clicked();

    void on_btnStartForceExecute_clicked();

    void on_btnCurrentPosition_clicked();

    void on_btnLoadMainWindowData_clicked();

public:
    void setOptimizer(TrajectoryOptimizer* optimizer);

private:
    Ui::AdmittanceControllerForm *ui;
    TrajectoryOptimizer* optimizer_;  // 持有指针但不拥有生命周期
    ForceControlExecutor* forceExecutor;
    AdmittanceController* controller;
    std::vector<std::array<double, 8>> jointOnce;//用于单次运动

    std::vector<Eigen::Matrix4d> trajectory;//轨迹
    RobotPose robotPose;
    QVector<double> forceData;
    ForceTorqueGravityCompensator* compensator;
    QSettings *iniRead;//ini文件管理
    identification_param loadIdentificationParam(QSettings* iniRead);
    void saveIdentificationParam(QSettings* iniWrite, const identification_param& param);
    identification_param id_param;
    Point6D offset;//力控计算的位移
    void saveIntegralParams(QSettings *iniWrite, const IntegralParams &params);
    IntegralParams loadIntegralParams(QSettings *iniRead);
    IntegralParams integralParams;
    bool idle_=false;
    std::array<double, 8> currentEndMoveJoint_;
    void updateCurrentPosiSpin();
signals:
    void requestPosition();  // 请求主窗口刷新
    void UpdateUI(QString str);
    void MoveJointTraj(const std::vector<std::array<double, 8>>& jointTraj);
    //是否停止运动
    void requestIdle();
    void requestTrajectory();
    //获取当前运动结束位置
    void requestCurrentEndMoveJoint();

public slots:
    void onPositionUpdated(const RobotPose& pose);
    void onForceDataReady(const QVector<double> &data);
    void receiveTrajectory(const std::vector<Eigen::Matrix4d> &traj);
    void receiveIdle(const bool idle);
    void receiveCurrentEndMoveJoint(const std::array<double, 8> currentEndMoveJoint);
};

#endif // ADMITTANCECONTROLLERFORM_H
