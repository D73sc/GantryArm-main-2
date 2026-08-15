#ifndef FORCEFORM_H
#define FORCEFORM_H

#include <QWidget>
#include "ForceSerialPortManager.h"
#include <QDebug>
namespace Ui {
class ForceForm;
}

class ForceForm : public QWidget
{
    Q_OBJECT

public:
    explicit ForceForm(QWidget *parent = nullptr);
    ~ForceForm();

private slots:
    void on_btnRefreshPortsbtn_Force_clicked();

    void on_btnOpenCOM_Force_clicked();

    void on_btnSendData_Force_clicked();

    void on_btnOnceSendData_Force_clicked();

    void on_btnStartAutoSendData_Force_clicked();

    void on_btnSetHz_clicked();

    void on_btnInitSensor_clicked();

    void on_btnSaveData_clicked();

    void on_btnExportData_clicked();

private:
    Ui::ForceForm *ui;
    //六维力传感器串口
    void SerialPortInit();
    ForceSerialPortManager *m_serialManager;
    void onDataReceived(QString hexData);
    void onErrorReceived(const QString &error);
    void onForceDataUpdated(const QVector<double> &forceData);
signals:
    void forceDataReady(const QVector<double> &data);


};

#endif // FORCEFORM_H
