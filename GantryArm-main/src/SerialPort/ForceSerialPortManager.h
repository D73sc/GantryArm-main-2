#ifndef ForceSerialPortManager_H
#define ForceSerialPortManager_H

#include "BaseSerialPortManager.h"
#include <QByteArray>
#include <QString>

class ForceSerialPortManager : public BaseSerialPortManager
{
    Q_OBJECT

public:
    explicit ForceSerialPortManager(QObject *parent = nullptr);
    ~ForceSerialPortManager() override = default;

    // 指令集合 - 直接调用即可
    void sendSingleDataRequest();      // 获取一次数据: AT+GOD
    void startContinuousData();        // 开启连续采集: AT+GSD
    void stopContinuousData();         // 停止连续采集: AT+GSD=STOP
    void sendCustomCommand(const QString &cmd); // 自定义命令
    void setContinuousHZ(float hz);
    void initSensor();

protected:
    void processDataFrame() override;
    void sendPollingRequest(quint8 stationAddress) override;
    void stopCollectData(){
        stopContinuousData();
    }

private:
    // 工具函数
    QByteArray commandToAscii(const QString &cmd);
    float bytesToFloat(const QByteArray &data);
    bool parseForceDataPacket(const QByteArray &packet);

    // 协议常量
    static constexpr quint8 PACKET_HEADER_1 = 0xAA;
    static constexpr quint8 PACKET_HEADER_2 = 0x55;
    static constexpr int CHANNEL_COUNT = 6;
    static constexpr int CHANNEL_DATA_LENGTH = 4;
    static constexpr int HEADER_LENGTH = 4; // AA 55 + 长度2字节
};

#endif // ForceSerialPortManager_H
