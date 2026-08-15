#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_dahengTwoCams_qt_vs.h"
#include "CDeviceProcess.h"
#include <QDir>
#include <QDateTime>
#include <QFileDialog>
#define  DEVICE_CONTS 2               //最多允许同时操作2台设备

class dahengTwoCams_qt_vs : public QMainWindow
{
    Q_OBJECT

public:
    dahengTwoCams_qt_vs(QWidget *parent = Q_NULLPTR);

private:
    Ui::dahengTwoCams_qt_vsClass ui;

	//初始化界面UI
	void __InitUI(CGXFeatureControlPointer objFeatureControlPtr);

	//更新界面
	void __UpdateUI(CDeviceProcess* pDeviceProcess);

	//关闭窗口
	void closeEvent(QCloseEvent *event);

	//初始化API
	void __InitCGXAPI();


	GxIAPICPP::gxdeviceinfo_vector m_vectorDeviceInfo;          // 枚举到的设备信息
	CDeviceProcess* m_pDeviceProcess[DEVICE_CONTS];             // 设备处理类对象
	CDeviceProcess* m_pDeviceProcessCurrent;                    // 当前设备处理类对象
	std::map<int, CDeviceProcess*> m_mapDeviceInformation;      // 用于存储设备处理类
	bool     m_bIsSnapSpeed;                                    // 是否支持采集速度级别
	bool     m_bIsColorFilter;                                  // 是否支持Bayer格式
	double   m_dShutterMax;                                     // 曝光时间最大值
	double   m_dShutterMin;                                     // 曝光时间最小值
	double   m_dGainMax;                                        // 增益最大值
	double   m_dGainMin;                                        // 增益最小值
	int64_t  m_nSnapSpeedMax;                                   // 采集速度级别最大值
	int64_t  m_nSnapSpeedMin;                                   // 采集速度级别最小值
	int      m_nDeviceListCurrent;                              // 当前设备列表序号
	int      m_nCurrentBalanceAutoWhiteSel;                     // 记录白平衡的值
	gxstring m_strBalanceWhiteAuto;                             // 当前自动白平衡

public:
	double		m_dEditShutter;
	double		m_dEditGain;
	int64_t		m_nEditSnapSpeed;
	bool		m_bCheckShowDevice;

    QLabel		*m_pLLabelStaBar;
    QLabel		*m_pRLabelStaBar;
	QTimer		*m_Timer;

	void time_update();

	void on_Btn_refreshDeviceList_clicked();
	void on_Btn_OpenDevice_clicked();
	void on_Btn_CloseDevice_clicked();
	void on_Btn_StartCapture_clicked();
	void on_Btn_StopCapture_clicked();
	void on_Cb_DeviceList_currentIndexChanged(int nIndex);
	void on_Cb_BalanceWhiteAuto_currentIndexChanged(const QString &strCurSel);
	void on_lE_Shutter_editingFinished();
	void on_lE_Gain_editingFinished();
	void on_lE_CaptureSpeed_editingFinished();
	void on_Btn_SaveImage_clicked();
	void on_Ckb_ShowDeviceSN_stateChanged(int state);

private slots:
    void on_btn_openFilePath_clicked();
};
