#include "LanserSerialPortManager.h"
#include <QDebug>

LanserSerialPortManager::LanserSerialPortManager(QObject *parent)
    : BaseSerialPortManager(parent)
{
}

void LanserSerialPortManager::processDataFrame()
{
    // 循环处理缓冲区中的所有完整帧
    while (m_receiveBuffer.size() >= 4) {
        quint8 function = static_cast<quint8>(m_receiveBuffer[1]);

        switch (function) {
        case FUNCTION_CODE_READ_HOLDING:
            if (!processReadHoldingRegisters()) {
                // 如果处理失败（数据不完整或错误），跳出循环等待更多数据
                return;
            }
            break;
        default:
            // 未知功能码，移除第一个字节继续查找
            m_receiveBuffer.remove(0, 1);
            break;
        }
    }
}

bool LanserSerialPortManager::processReadHoldingRegisters()
{
    if (m_receiveBuffer.size() < 3) {
        return false;  // 数据不足，等待更多数据
    }

    quint8 slaveAddress = static_cast<quint8>(m_receiveBuffer[0]);
    quint8 functionCode = static_cast<quint8>(m_receiveBuffer[1]);
    quint8 byteCount = static_cast<quint8>(m_receiveBuffer[2]);

    if (functionCode != FUNCTION_CODE_READ_HOLDING) {
        m_receiveBuffer.remove(0, 1);
        return true;  // 继续处理下一帧
    }

    int expectedLength = 3 + byteCount + 2;  // 地址+功能码+字节数+数据+CRC

    if (m_receiveBuffer.size() < expectedLength) {
        return false;  // 数据不完整，等待更多数据
    }

    QByteArray completeFrame = m_receiveBuffer.left(expectedLength);

    // CRC校验
    if (!validateLanserCRC(completeFrame)) {
        qWarning() << "CRC validation failed, removing first byte";
        m_receiveBuffer.remove(0, 1);
        return true;  // 继续处理下一帧
    }

    int stationIndex = slaveAddress - 1;

    if (stationIndex >= 0 && stationIndex < m_stationValues.size()) {
        QByteArray registerData = completeFrame.mid(3, byteCount);

        if (byteCount == 4) {
            if (registerData.toHex().toUpper() == "FFFFFFFF") {
                m_stationValues[stationIndex] = std::numeric_limits<float>::quiet_NaN();
                // m_stationValues[stationIndex] = 999.99;

                // emit invalidDataReceived(stationIndex);
            } else {
                QByteArray highWord = registerData.mid(0, 2);
                QByteArray lowWord = registerData.mid(2, 2);
                QByteArray swappedData = lowWord + highWord;

                QString floatHex = swappedData.toHex().toUpper();
                float value = hexToFloat(floatHex) / 100.0;
                m_stationValues[stationIndex] = value;
                // emit stationDataUpdated(stationIndex, value);

                // qDebug() << "Station" << stationIndex << "value:" << value;
            }
        } else {
            qWarning() << "Unexpected byte count:" << byteCount;
        }
    } else {
        qWarning() << "Invalid station index:" << stationIndex << "for address:" << slaveAddress;
    }

    // 移除已处理的完整帧
    m_receiveBuffer.remove(0, expectedLength);
    return true;  // 继续处理下一帧
}

bool LanserSerialPortManager::validateLanserCRC(const QByteArray &data)
{
    if (data.size() < 2) return false;

    quint16 receivedCRC = static_cast<quint8>(data.at(data.size()-2)) | (static_cast<quint8>(data.at(data.size()-1)) << 8);
    QByteArray dataWithoutCRC = data.left(data.size() - 2);
    quint16 calculatedCRC = calculateCRC(dataWithoutCRC);

    return receivedCRC == calculatedCRC;
}

void LanserSerialPortManager::sendPollingRequest(quint8 stationAddress)
{
    QByteArray sendBuf;
    sendBuf.append(char(stationAddress));
    sendBuf.append(char(FUNCTION_CODE_READ_HOLDING));
    sendBuf.append(char((REGISTER_ADDRESS >> 8) & 0xFF));
    sendBuf.append(char(REGISTER_ADDRESS & 0xFF));
    sendBuf.append(char((REGISTER_LENGTH >> 8) & 0xFF));
    sendBuf.append(char(REGISTER_LENGTH & 0xFF));

    quint16 crc = calculateCRC(sendBuf);
    sendBuf.append(char(crc & 0xFF));
    sendBuf.append(char((crc >> 8) & 0xFF));

    sendData(sendBuf);
}

quint16 LanserSerialPortManager::calculateCRC(const QByteArray &data)
{
    quint16 crc = 0xFFFF;
    const quint16 polynomial = 0xA001;
    for (int i = 0; i < data.size(); ++i) {
        crc ^= (quint8)data.at(i);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

float LanserSerialPortManager::hexToFloat(const QString &hexStr)
{
    if (hexStr.length() != 8) {
        qWarning() << "Invalid hex string length:" << hexStr.length();
        return 0.0f;
    }

    if (hexStr == "FFFFFFFF") {
        return std::numeric_limits<float>::quiet_NaN();
    }

    // 分别获取高四位和低四位
    QString high16Hex = hexStr.left(4);   // 前4位：高字
    QString low16Hex = hexStr.right(4);   // 后4位：低字

    bool ok1, ok2;
    quint16 highValue = high16Hex.toUShort(&ok1, 16);
    quint16 lowValue = low16Hex.toUShort(&ok2, 16);

    if (!ok1 || !ok2) {
        qWarning() << "Failed to parse hex - High:" << high16Hex << "Low:" << low16Hex;
        return 0.0f;
    }

    // 组合32位值：高字 * 65536 + 低字
    quint32 combinedValue = (static_cast<quint32>(highValue) << 16) | lowValue;

    // 转换为浮点数，除以1000
    float result = static_cast<float>(combinedValue) ;

    // 可选调试输出
    // qDebug() << "Hex:" << hexStr << "High:" << highValue << "Low:" << lowValue
    //          << "Combined:" << combinedValue << "Result:" << result;

    return result;
}
