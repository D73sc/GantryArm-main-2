#include "dahengTwoCams_qt_vs.h"

dahengTwoCams_qt_vs::dahengTwoCams_qt_vs(QWidget *parent)
    : QMainWindow(parent)
    , m_dEditShutter(0)
    , m_dEditGain(0)
    , m_nEditSnapSpeed(0)
    , m_bCheckShowDevice(false)
{
    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    ui.setupUi(this);
    m_pLLabelStaBar = new QLabel(this);
    m_pRLabelStaBar = new QLabel(this);


    statusBar()->addWidget(m_pLLabelStaBar);
    statusBar()->addWidget(m_pRLabelStaBar);



    m_Timer = new QTimer(this);

    m_bIsSnapSpeed = false;
    m_bIsColorFilter = false;
    m_dShutterMax = 0;
    m_dShutterMin = 0;
    m_dGainMax = 0;
    m_dGainMin = 0;
    m_nSnapSpeedMax = 0;
    m_nSnapSpeedMin = 0;
    m_nDeviceListCurrent = 0;
    m_nCurrentBalanceAutoWhiteSel = 0;
    m_strBalanceWhiteAuto = "Off";
    m_pDeviceProcessCurrent = NULL;


    //??????豸??????
    for (int i = 0; i < DEVICE_CONTS; i++)
    {
        m_pDeviceProcess[i] = NULL;
    }

    //????????????
    connect(m_Timer, &QTimer::timeout, this, &dahengTwoCams_qt_vs::time_update);
    connect(ui.pB_refreshDeviceList, &QPushButton::clicked, this, &dahengTwoCams_qt_vs::on_Btn_refreshDeviceList_clicked);
    connect(ui.pB_OpenDevice, &QPushButton::clicked, this, &dahengTwoCams_qt_vs::on_Btn_OpenDevice_clicked);
    connect(ui.pB_CloseDevice, &QPushButton::clicked, this, &dahengTwoCams_qt_vs::on_Btn_CloseDevice_clicked);
    connect(ui.pB_StartCapture, &QPushButton::clicked, this, &dahengTwoCams_qt_vs::on_Btn_StartCapture_clicked);
    connect(ui.pB_StopCapture, &QPushButton::clicked, this, &dahengTwoCams_qt_vs::on_Btn_StopCapture_clicked);
    connect(ui.lE_Shutter, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_Shutter_editingFinished);
    connect(ui.lE_Gain, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_Gain_editingFinished);
    connect(ui.lE_CaptureSpeed, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_CaptureSpeed_editingFinished);
    connect(ui.pB_SaveImage, &QPushButton::clicked, this, &dahengTwoCams_qt_vs::on_Btn_SaveImage_clicked);
    connect(ui.ckB_ShowDeviceSN, &QCheckBox::stateChanged, this, &dahengTwoCams_qt_vs::on_Ckb_ShowDeviceSN_stateChanged);

    //?????API
    __InitCGXAPI();

    //????????????
    connect(ui.cB_DeviceList, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_DeviceList_currentIndexChanged);
    connect(ui.cB_BalanceWhiteAuto, QOverload<const QString &>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_BalanceWhiteAuto_currentIndexChanged);

}

void dahengTwoCams_qt_vs::closeEvent(QCloseEvent *event)
{
    try
    {
        //????map??????????????豸??????
        map<int, CDeviceProcess*>::iterator it;
        for (it = m_mapDeviceInformation.begin(); it != m_mapDeviceInformation.end(); ++it)
        {
            it->second->CloseDevice();

        }

        m_mapDeviceInformation.clear();

    }
    catch (CGalaxyException)
    {
        //do noting

    }
    catch (std::exception)
    {
        //do noting
    }

    try
    {
        //????豸?????
        IGXFactory::GetInstance().Uninit();
    }
    catch (CGalaxyException)
    {
        //do noting
    }

    for (int i = 0; i < DEVICE_CONTS; i++)
    {
        if (m_pDeviceProcess[i] != NULL)
        {
            delete m_pDeviceProcess[i];
            m_pDeviceProcess[i] = NULL;
        }
    }
    event->accept();
}

void dahengTwoCams_qt_vs::__InitCGXAPI()
{
    try
    {
        for (int i = 0; i < DEVICE_CONTS; i++)
        {
            m_pDeviceProcess[i] = new CDeviceProcess;
        }
        //??????豸??
        IGXFactory::GetInstance().Init();

        //????豸
        IGXFactory::GetInstance().UpdateDeviceList(1000, m_vectorDeviceInfo);

        //δ?????豸
        if (m_vectorDeviceInfo.size() <= 0)
        {
            return;
        }

        //???豸??????????豸?б???
        for (uint32_t i = 0; i < m_vectorDeviceInfo.size(); i++)
        {
            //?????豸??????2???????2???豸???
            if (i >= DEVICE_CONTS)
            {
                break;
            }

            gxstring strDeviceInformation = "";
            strDeviceInformation = m_vectorDeviceInfo[i].GetDisplayName();
            ui.cB_DeviceList->addItem(strDeviceInformation.c_str());
            m_mapDeviceInformation.insert(map<int, CDeviceProcess*>::value_type(i, m_pDeviceProcess[i]));

            //???????
            __UpdateUI(m_pDeviceProcess[i]);

        }

        //????????
        m_Timer->start(1000);
        ui.cB_DeviceList->setCurrentIndex(0);

        //????????????豸??????????
        m_pDeviceProcessCurrent = m_mapDeviceInformation[0];

    }
    catch (CGalaxyException& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
    catch (std::exception& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }

    // TODO: Add your specialized code here and/or call the base class

}

void dahengTwoCams_qt_vs::__InitUI(CGXFeatureControlPointer objFeatureControlPtr)
{
    if (objFeatureControlPtr.IsNull())
    {
        return;
    }
    //???????????????????????????
    disconnect(ui.cB_BalanceWhiteAuto, QOverload<const QString &>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_BalanceWhiteAuto_currentIndexChanged);
    disconnect(ui.lE_Shutter, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_Shutter_editingFinished);
    disconnect(ui.lE_Gain, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_Gain_editingFinished);
    disconnect(ui.lE_CaptureSpeed, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_CaptureSpeed_editingFinished);

    bool bIsBalanceWhiteAutoRead = false;        // ??????????????
    bool bBalanceWhiteAuto = false;        // ??????????????

    //??????Bayer???
    m_bIsColorFilter = objFeatureControlPtr->IsImplemented("PixelColorFilter");

    if (m_bIsColorFilter)
    {
        //??????????????????
        //??????????
        bBalanceWhiteAuto = objFeatureControlPtr->IsImplemented("BalanceWhiteAuto");

        //???????????
        bIsBalanceWhiteAutoRead = objFeatureControlPtr->IsReadable("BalanceWhiteAuto");
        if (bBalanceWhiteAuto)
        {
            if (bIsBalanceWhiteAutoRead)
            {
                m_strBalanceWhiteAuto = objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")->GetValue();
            }
            int nCursel = 0;
            gxstring strCurEnumList = "";
            GxIAPICPP::gxstring_vector vectorEnumEntryList;

            //???????豸?????
            strCurEnumList = objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")->GetValue();

            //????豸?????????
            vectorEnumEntryList = objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")->GetEnumEntryList();

            //??????????б?
            ui.cB_BalanceWhiteAuto->clear();
            for (uint32_t i = 0; i < vectorEnumEntryList.size(); i++)
            {
                std::string strEnumList = vectorEnumEntryList[i].c_str();
                ui.cB_BalanceWhiteAuto->addItem(strEnumList.c_str());
                if (strCurEnumList == vectorEnumEntryList[i])
                {
                    nCursel = i;
                }

            }
            m_nCurrentBalanceAutoWhiteSel = nCursel;
            ui.cB_BalanceWhiteAuto->setCurrentIndex(nCursel);
        }
    }


    //???????????
    QString  strShutterTimeRange = "";          // ??????Χ
    gxstring strShutterTimeUint = "";          // ??????λ

    m_dEditShutter = objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetValue();
    strShutterTimeUint = objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetUnit();
    m_dShutterMax = objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMax();
    m_dShutterMin = objFeatureControlPtr->GetFloatFeature("ExposureTime")->GetMin();

    strShutterTimeRange = QString().sprintf("ExposureTime(%.4f~%.4f)%s", m_dShutterMin, m_dShutterMax, strShutterTimeUint.c_str());
    ui.lB_Static_ShutterTime->setText(strShutterTimeRange);

    //?????????
    QString  strGainRange = "";                // ???????Χ
    gxstring strGainUint = "";                // ????λ

    m_dEditGain = objFeatureControlPtr->GetFloatFeature("Gain")->GetValue();
    strGainUint = objFeatureControlPtr->GetFloatFeature("Gain")->GetUnit();
    m_dGainMax = objFeatureControlPtr->GetFloatFeature("Gain")->GetMax();
    m_dGainMin = objFeatureControlPtr->GetFloatFeature("Gain")->GetMin();


    strGainRange = QString().sprintf("Gain(%.4f~%.4f)%s", m_dGainMin, m_dGainMax, strGainUint.c_str());
    ui.lB_Static_Gain->setText(strGainRange);

    //??????????????
    //??????????????
    m_bIsSnapSpeed = objFeatureControlPtr->IsImplemented("AcquisitionSpeedLevel");
    if (m_bIsSnapSpeed)
    {
        QString  strSnapSpeedRange = "";
        m_nEditSnapSpeed = objFeatureControlPtr->GetIntFeature("AcquisitionSpeedLevel")->GetValue();
        m_nSnapSpeedMax = objFeatureControlPtr->GetIntFeature("AcquisitionSpeedLevel")->GetMax();
        m_nSnapSpeedMin = objFeatureControlPtr->GetIntFeature("AcquisitionSpeedLevel")->GetMin();

        strSnapSpeedRange = QString().sprintf("AcquisitionSpeedLevel(%lld~%lld)", m_nSnapSpeedMin, m_nSnapSpeedMax);
        ui.lB_Static_CaptureSpeed->setText(strSnapSpeedRange);
        ui.lE_CaptureSpeed->setText(QString().sprintf("%11d", m_nEditSnapSpeed));
    }
    else
    {
        //??±???
        m_nEditSnapSpeed = NULL;
    }

    //??????豸???????豸??????????
    m_bCheckShowDevice = m_pDeviceProcessCurrent->GetShowSN();

    //???????
    ui.lE_Shutter->setText(QString().sprintf("%.4f", m_dEditShutter));
    ui.lE_Gain-> setText(QString().sprintf("%.4f", m_dEditGain));
    ui.ckB_ShowDeviceSN->setChecked(m_bCheckShowDevice);

    //????????????????
    connect(ui.cB_BalanceWhiteAuto, QOverload<const QString &>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_BalanceWhiteAuto_currentIndexChanged);
    connect(ui.lE_Shutter, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_Shutter_editingFinished);
    connect(ui.lE_Gain, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_Gain_editingFinished);
    connect(ui.lE_CaptureSpeed, &QLineEdit::editingFinished, this, &dahengTwoCams_qt_vs::on_lE_CaptureSpeed_editingFinished);

}

void dahengTwoCams_qt_vs::__UpdateUI(CDeviceProcess* pDeviceProcess)
{
    if (pDeviceProcess == NULL)
    {
        return;
    }

    bool IsOpen = pDeviceProcess->IsOpen();
    bool IsSnap = pDeviceProcess->IsSnap();
    ui.pB_OpenDevice->setEnabled(!IsOpen);
    ui.pB_CloseDevice->setEnabled(IsOpen);
    ui.pB_StartCapture->setEnabled(IsOpen && !IsSnap);
    ui.pB_StopCapture->setEnabled(IsOpen && IsSnap);
    ui.lE_Shutter->setEnabled(IsOpen);
    ui.lE_Gain->setEnabled(IsOpen);
    ui.lE_CaptureSpeed->setEnabled(IsOpen && !IsSnap&& m_bIsSnapSpeed);
    ui.cB_BalanceWhiteAuto->setEnabled(IsOpen&& m_bIsColorFilter);
    ui.ckB_ShowDeviceSN->setEnabled(IsOpen);
    ui.lE_ImageName->setEnabled(IsOpen && IsSnap);
    ui.pB_SaveImage->setEnabled(IsOpen && IsSnap);
}


void dahengTwoCams_qt_vs::time_update()
{
    try
    {
        //??????????????Once,???ó?????????????????????off
        //??????????????豸????????????????UI?????????
        QString strCurText = "";
        if (m_strBalanceWhiteAuto == "Once")
        {
            //????豸?????????
            m_strBalanceWhiteAuto = m_pDeviceProcessCurrent->m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")
                ->GetValue();
            //????豸??????????
            GxIAPICPP::gxstring_vector vectorEnumEntryList;
            vectorEnumEntryList = m_pDeviceProcessCurrent->m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")
                ->GetEnumEntryList();
            //?ж?????????????Off
            if (m_strBalanceWhiteAuto == "Off")
            {
                for (uint32_t i = 0; i < vectorEnumEntryList.size(); i++)
                {
                    strCurText = ui.cB_BalanceWhiteAuto->itemText(i);
                    if (strCurText == "Off")
                    {
                        ui.cB_BalanceWhiteAuto->setCurrentIndex(i);
                        break;
                    }
                }

            }

        }
        m_Timer->stop();
    }
    catch (CGalaxyException)
    {
        m_Timer->stop();
        return;
    }
    catch (std::exception)
    {
        m_Timer->stop();
        return;
    }
}

void dahengTwoCams_qt_vs::on_Btn_refreshDeviceList_clicked()
{
    try
    {
        //????map??????????????豸??????
        map<int, CDeviceProcess*>::iterator it;
        for (it = m_mapDeviceInformation.begin(); it != m_mapDeviceInformation.end(); ++it)
        {
            it->second->CloseDevice();

        }
        //?????豸???????
        m_mapDeviceInformation.clear();
        m_vectorDeviceInfo.clear();

    }
    catch (CGalaxyException)
    {
        //do noting

    }
    catch (std::exception)
    {
        //do noting
    }

    //??????ж???
    for (int i = 0; i < DEVICE_CONTS; i++)
    {
        if (m_pDeviceProcess[i] != NULL)
        {
            delete m_pDeviceProcess[i];
            m_pDeviceProcess[i] = NULL;
        }
    }

    disconnect(ui.cB_DeviceList, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_DeviceList_currentIndexChanged);

    try
    {
        for (int i = 0; i < DEVICE_CONTS; i++)
        {
            m_pDeviceProcess[i] = new CDeviceProcess;
        }

        //????豸
        IGXFactory::GetInstance().UpdateDeviceList(1000, m_vectorDeviceInfo);

        //δ?????豸
        if (m_vectorDeviceInfo.size() <= 0)
        {
            return;
        }

        ui.cB_DeviceList->clear();
        //???豸??????????豸?б???
        for (uint32_t i = 0; i < m_vectorDeviceInfo.size(); i++)
        {
            //?????豸??????2???????2???豸???
            if (i >= DEVICE_CONTS)
            {
                break;
            }

            gxstring strDeviceInformation = "";
            strDeviceInformation = m_vectorDeviceInfo[i].GetDisplayName();
            ui.cB_DeviceList->addItem(strDeviceInformation.c_str());
            m_mapDeviceInformation.insert(map<int, CDeviceProcess*>::value_type(i, m_pDeviceProcess[i]));

            //???????
            __UpdateUI(m_pDeviceProcess[i]);

        }

        ui.cB_DeviceList->setCurrentIndex(0);

        connect(ui.cB_DeviceList, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_DeviceList_currentIndexChanged);

        //????????????豸??????????
        m_pDeviceProcessCurrent = m_mapDeviceInformation[0];

    }
    catch (CGalaxyException& e)
    {
        connect(ui.cB_DeviceList, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_DeviceList_currentIndexChanged);
        m_pRLabelStaBar->setText(e.what());
        return;
    }
    catch (std::exception& e)
    {
        connect(ui.cB_DeviceList, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &dahengTwoCams_qt_vs::on_Cb_DeviceList_currentIndexChanged);
        m_pRLabelStaBar->setText(e.what());
        return;
    }

}

void dahengTwoCams_qt_vs::on_Btn_OpenDevice_clicked()
{
    try
    {
        //??????????????????
        CGXBitmap *pBitmap;

        switch (m_nDeviceListCurrent)
        {
        case 0:
            pBitmap = (CGXBitmap *)ui.bitMap_LCam;
            break;
        case 1:
            pBitmap = (CGXBitmap *)ui.bitMap_RCam;
            break;
        default: break;
        }

        //????豸????
        int nDeviceIndex = m_nDeviceListCurrent + 1;

        //?????豸
        m_pDeviceProcessCurrent->OpenDevice(m_vectorDeviceInfo[m_nDeviceListCurrent].GetSN(), pBitmap, nDeviceIndex);

        //?????????
        __InitUI(m_pDeviceProcessCurrent->m_objFeatureControlPtr);

        //???????
        __UpdateUI(m_pDeviceProcessCurrent);

    }
    catch (CGalaxyException& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
    catch (std::exception& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
}

void dahengTwoCams_qt_vs::on_Btn_CloseDevice_clicked()
{
    try
    {
        //????豸
        m_pDeviceProcessCurrent->CloseDevice();


        //???????
        __UpdateUI(m_pDeviceProcessCurrent);
    }
    catch (CGalaxyException)
    {
        //do noting
    }
    catch (std::exception)
    {
        //do noting
        return;
    }
}

void dahengTwoCams_qt_vs::on_Btn_StartCapture_clicked()
{
    // TODO: Add your control notification handler code here
    try
    {
        //??????
        m_pDeviceProcessCurrent->StartSnap();

        //???????
        __UpdateUI(m_pDeviceProcessCurrent);

        //?????豸??????????
        m_pDeviceProcessCurrent->RefreshDeviceSN();

    }
    catch (CGalaxyException& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
    catch (std::exception& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
}

void dahengTwoCams_qt_vs::on_Btn_StopCapture_clicked()
{
    try
    {
        //?????
        m_pDeviceProcessCurrent->StopSnap();

        //???????
        __UpdateUI(m_pDeviceProcessCurrent);

    }
    catch (CGalaxyException& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
    catch (std::exception& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }
}

void dahengTwoCams_qt_vs::on_Cb_DeviceList_currentIndexChanged(int nIndex)
{
    try
    {
        m_nDeviceListCurrent = nIndex;

        m_pDeviceProcessCurrent = m_mapDeviceInformation[m_nDeviceListCurrent];

        //?ж????豸????????
        bool bIsOpen = m_pDeviceProcessCurrent->IsOpen();
        if (!bIsOpen)
        {
            __UpdateUI(m_pDeviceProcessCurrent);
        }

        else
        {
            //??????
            __InitUI(m_pDeviceProcessCurrent->m_objFeatureControlPtr);

            //???????
            __UpdateUI(m_pDeviceProcessCurrent);

        }
    }
    catch (CGalaxyException& e)
    {
        m_pRLabelStaBar->setText(e.what());

        return;

    }
    catch (std::exception& e)
    {
        m_pRLabelStaBar->setText(e.what());
        return;
    }

}

void dahengTwoCams_qt_vs::on_Cb_BalanceWhiteAuto_currentIndexChanged(const QString &strCurSel)
{
    try
    {
        int      nCurSelBalanceWhiteAuto = 0;      // ?????????????????
        string  strCurText		= strCurSel.toStdString();				   // ??????????????
        gxstring strCurEnumList = "";

        nCurSelBalanceWhiteAuto = ui.cB_BalanceWhiteAuto->currentIndex();
        strCurEnumList = &strCurText[0];

        //?????????????
        m_strBalanceWhiteAuto = strCurEnumList;

        //???????????????????豸??
        m_pDeviceProcessCurrent->m_objFeatureControlPtr->GetEnumFeature("BalanceWhiteAuto")->SetValue(strCurEnumList);

        //????????????
        m_nCurrentBalanceAutoWhiteSel = nCurSelBalanceWhiteAuto;

    }
    catch (CGalaxyException& e)
    {
        ui.cB_BalanceWhiteAuto->setCurrentIndex(m_nCurrentBalanceAutoWhiteSel);
        m_pRLabelStaBar->setText(e.what());
        return;
    }
    catch (std::exception& e)
    {
        ui.cB_BalanceWhiteAuto->setCurrentIndex(m_nCurrentBalanceAutoWhiteSel);
        m_pRLabelStaBar->setText(e.what());
        return;
    }
}


void dahengTwoCams_qt_vs::on_lE_Shutter_editingFinished()
{
    double dShutterOld = m_dEditShutter;           // ??????????
    if (!m_pDeviceProcessCurrent->IsOpen())
    {
        return;
    }
    try
    {
        //???????
        m_dEditShutter = ui.lE_Shutter->text().toDouble();

        //?ж????????????????????????Χ??
        if (m_dEditShutter < m_dShutterMin)
        {
            m_dEditShutter = m_dShutterMin;
        }

        if (m_dEditShutter > m_dShutterMax)
        {
            m_dEditShutter = m_dShutterMax;
        }

        //???????????豸??
        m_pDeviceProcessCurrent->m_objFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(m_dEditShutter);

    }
    catch (CGalaxyException& e)
    {
        m_dEditShutter = dShutterOld;
        ui.lE_Shutter->setText(QString().sprintf("%.4f", m_dEditShutter));
        m_pRLabelStaBar->setText(e.what());
    }
    catch (std::exception& e)
    {
        m_dEditShutter = dShutterOld;
        ui.lE_Shutter->setText(QString().sprintf("%.4f", m_dEditShutter));
        m_pRLabelStaBar->setText(e.what());
    }
}

void dahengTwoCams_qt_vs::on_lE_Gain_editingFinished()
{
    double dGainOld = m_dEditGain;         // ????????
    if (!m_pDeviceProcessCurrent->IsOpen())
    {
        return;
    }

    try
    {
        //???????
        m_dEditGain = ui.lE_Gain->text().toDouble();

        //?ж??????????????????????Χ??
        if (m_dEditGain < m_dGainMin)
        {
            m_dEditGain = m_dGainMin;
        }

        if (m_dEditGain > m_dGainMax)
        {
            m_dEditGain = m_dGainMax;
        }

        m_pDeviceProcessCurrent->m_objFeatureControlPtr->GetFloatFeature("Gain")->SetValue(m_dEditGain);

    }
    catch (CGalaxyException& e)
    {
        m_dEditGain = dGainOld;
        ui.lE_Gain->setText(QString().sprintf("%.4f", m_dEditGain));
        m_pRLabelStaBar->setText(e.what());
    }
    catch (std::exception& e)
    {
        m_dEditGain = dGainOld;
        ui.lE_Gain->setText(QString().sprintf("%.4f", m_dEditGain));
        m_pRLabelStaBar->setText(e.what());
    }

}

void dahengTwoCams_qt_vs::on_lE_CaptureSpeed_editingFinished()
{
    int64_t     nSnapSpeedOld = m_nEditSnapSpeed;      // ?????????????

    try
    {
        m_nEditSnapSpeed = ui.lE_CaptureSpeed->text().toLongLong();

        if (m_nEditSnapSpeed < m_nSnapSpeedMin)
        {
            m_nEditSnapSpeed = m_nSnapSpeedMin;
        }

        if (m_nEditSnapSpeed > m_nSnapSpeedMax)
        {
            m_nEditSnapSpeed = m_nSnapSpeedMax;
        }
        m_pDeviceProcessCurrent->m_objFeatureControlPtr->GetIntFeature("AcquisitionSpeedLevel")->SetValue(m_nEditSnapSpeed);

    }
    catch (CGalaxyException& e)
    {
        m_nEditSnapSpeed = nSnapSpeedOld;
        ui.lE_CaptureSpeed->setText(QString().sprintf("%11d", m_nEditSnapSpeed));
        m_pRLabelStaBar->setText(e.what());
    }
    catch (std::exception& e)
    {
        m_nEditSnapSpeed = nSnapSpeedOld;
        ui.lE_CaptureSpeed->setText(QString().sprintf("%11d", m_nEditSnapSpeed));
        m_pRLabelStaBar->setText(e.what());
    }

}

void dahengTwoCams_qt_vs::on_Btn_SaveImage_clicked()
{
    // 获取当前时间并格式化为字符串
    QString timeStamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");

    // 构建保存路径
    QString fullPath =timeStamp;

    QString qstrImageName = ui.lE_ImageName->text();


        if (m_vectorDeviceInfo.size() == 2)
        {
            if (m_pDeviceProcess[0]->IsOpen() && m_pDeviceProcess[0]->IsSnap() &&
                m_pDeviceProcess[1]->IsOpen() && m_pDeviceProcess[1]->IsSnap())
            {
                m_pDeviceProcess[0]->setSaveImage(true, fullPath);
                m_pDeviceProcess[1]->setSaveImage(true, fullPath);
            }
        }
        else
        {
            m_pDeviceProcessCurrent->setSaveImage(true, fullPath);
        }

}

void dahengTwoCams_qt_vs::on_Ckb_ShowDeviceSN_stateChanged(int state)
{
    switch (state)
    {
    case Qt::Unchecked:
        m_bCheckShowDevice = false;
        break;
    case Qt::Checked:
        m_bCheckShowDevice = true;
        break;
    default:
        break;
    }

    m_pDeviceProcessCurrent->SetShowSN(m_bCheckShowDevice);
}

void dahengTwoCams_qt_vs::on_btn_openFilePath_clicked()
{
    QFileDialog dlg(this);
    QString currentPath = QDir::currentPath();
    QString dataQFile = QDir(currentPath).filePath("lib/data/photos");

    // 打开文件对话框，让用户选择一个 CSV 文件
    QString folderPath = QFileDialog::getExistingDirectory(
        nullptr,                    // 父窗口指针
        tr("Select Folder"),       // 对话框标题
        dataQFile,                 // 默认目录
        QFileDialog::ShowDirsOnly  // 选项：只显示目录
        );

    // 检查是否选择了文件
    if (!folderPath.isEmpty()) {
        // 将文件路径赋值给变量
        // QString csvfilePath = fileName;
        ui.lE_ImageFilePath->setText(folderPath);
        m_pDeviceProcessCurrent->setSavePath(folderPath);

    } else {
    }
}

