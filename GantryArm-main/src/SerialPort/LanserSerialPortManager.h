#ifndef LANSERSERIALPORTMANAGER_H
#define LANSERSERIALPORTMANAGER_H

#include "BaseSerialPortManager.h"

class LanserSerialPortManager : public BaseSerialPortManager
{
    Q_OBJECT
public:
    explicit LanserSerialPortManager(QObject *parent = nullptr);
    ~LanserSerialPortManager() override = default;

protected:
    void processDataFrame() override;
    void sendPollingRequest(quint8 stationAddress) override;

private:
    bool validateLanserCRC(const QByteArray &data);
    bool processReadHoldingRegisters();
    quint16 calculateCRC(const QByteArray &data);
    float hexToFloat(const QString &hexStr);

    static constexpr quint8 FUNCTION_CODE_READ_HOLDING = 0x03;
    static constexpr quint16 REGISTER_ADDRESS = 0x003B;
    static constexpr quint16 REGISTER_LENGTH = 0x0002;

};

#endif // LANSERSERIALPORTMANAGER_H
