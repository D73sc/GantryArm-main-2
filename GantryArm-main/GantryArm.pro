DEFINES -= UNICODE
DEFINES += UMBCS
QMAKE_CXXFLAGS -= -Zc:strictStrings
CODECFORTR = UTF-8

QT       += core gui opengl serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GantryArm
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
# 强制使用Unicode
DEFINES += UNICODE _UNICODE
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++17

SOURCES += \
    src/AdmittanceControl/AdmittanceController.cpp \
    src/AdmittanceControl/ForceControlExecutor.cpp \
    src/AdmittanceControl/ForceTorqueGravityCompensator.cpp \
    src/AdmittanceControl/admittancecontrollerform.cpp \
    src/Camera/CDeviceProcess.cpp \
    src/Camera/CGXBitmap.cpp \
    src/Camera/dahengTwoCams_qt_vs.cpp \
    src/RobotTrajectoryPlan/CollisionChecker.cpp \
    src/RobotTrajectoryPlan/PlaneAlignmentCompensator.cpp \
    src/RobotTrajectoryPlan/RobotArm.cpp \
    src/RobotTrajectoryPlan/TrajectoryOptimizer.cpp \
    src/RobotTrajectoryPlan/curve_processor_eigen.cpp \
    src/RobotView/ddr6robotwidget.cpp \
    src/RobotView/rrglwidget.cpp \
    src/RobotView/stlfileloader.cpp \
    src/SerialPort/BaseSerialPortManager.cpp \
    src/SerialPort/ForceSerialPortManager.cpp \
    src/SerialPort/LanserSerialPortManager.cpp \
    src/SerialPort/forceform.cpp \
    src/Vision/CameraParamsCalibration.cpp \
    src/Vision/FeatureDetector.cpp \
    src/Vision/HandEyeCalibration.cpp \
    src/Vision/TCPCalibrator.cpp \
    src/Zmotion/zmcaux.cpp \
    src/Zmotion/zmotioncontrol.cpp \
    src/loghandler.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/Vision/CameraCalibration.cpp

HEADERS += \
    src/AdmittanceControl/AdmittanceController.h \
    src/AdmittanceControl/ForceControlExecutor.h \
    src/AdmittanceControl/ForceTorqueGravityCompensator.h \
    src/AdmittanceControl/admittancecontrollerform.h \
    src/Camera/CDeviceProcess.h \
    src/Camera/CGXBitmap.h \
    src/Camera/dahengTwoCams_qt_vs.h \
    src/RobotTrajectoryPlan/CollisionChecker.h \
    src/RobotTrajectoryPlan/PlaneAlignmentCompensator.h \
    src/RobotTrajectoryPlan/RobotArm.h \
    src/RobotTrajectoryPlan/TrajectoryOptimizer.h \
    src/RobotTrajectoryPlan/curve_processor_eigen.h \
    src/RobotTrajectoryPlan/robot_types.h \
    src/RobotView/ddr6robotwidget.h \
    src/RobotView/rrglwidget.h \
    src/RobotView/stlfileloader.h \
    src/SerialPort/BaseSerialPortManager.h \
    src/SerialPort/ForceSerialPortManager - 副本.h \
    src/SerialPort/ForceSerialPortManager.h \
    src/SerialPort/LanserSerialPortManager.h \
    src/SerialPort/forceform.h \
    src/Vision/CameraParamsCalibration.h \
    src/Vision/FeatureDetector.h \
    src/Vision/HandEyeCalibration.h \
    src/Vision/TCPCalibrator.h \
    src/Vision/calibration_types.h \
    src/Zmotion/zmcaux.h \
    src/Zmotion/zmotion.h \
    src/Zmotion/zmotioncontrol.h \
    src/loghandler.h \
    src/mainwindow.h \
    src/Vision/CameraCalibration.h \

FORMS += \
    src/AdmittanceControl/admittancecontrollerform.ui \
    src/Camera/CGXBitmap.ui \
    src/Camera/dahengTwoCams_qt_vs.ui \
    src/SerialPort/forceform.ui \
    src/mainwindow.ui \

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -L$$PWD/./lib/ \
            -lzmotion
Debug: {
    # LIBS += -L$$PWD/./bin/opencv/build/x64/vc15/lib/opencv_world455d
    LIBS += -L$$PWD/./lib/ \
            -lopencv_world453d

}

Release: {
    # LIBS += -L$$PWD/./bin/opencv/build/x64/vc15/lib/opencv_world455
    LIBS += -L$$PWD/./lib/ \
            -lopencv_world453

}
# win32: LIBS += -L$$PWD/./lib/ \
#             -lzmotion\
#             -lopencv_world490\
#             -lrealsense2\
#             -L$$PWD/./lib/x86/ \
#             -lOpenGL32\
#             -lGlU32\


win32 {
    contains(QMAKE_TARGET.arch, x86_64) {
        # 64-bit Windows
        LIBS += -L$$PWD/./lib/x64/ \
                -lOpenGL32 \
                -lGlU32 \
                -GxIAPICPPEx
    } else {
        # 32-bit Windows
        LIBS += -L$$PWD/./lib/x86/ \
                -lOpenGL32 \
                -lGlU32\
                -GxIAPICPPEx
    }
}






INCLUDEPATH += $$PWD/bin/eigen-3.4.0\
                $$PWD/bin/opencv453/include\
                $$PWD/bin/opencv453/include/opencv2\
                $$PWD/bin/inc\
                $$PWD/src/RobotTrajectoryPlan\
                $$PWD/src/Vision\
                $$PWD/src/Camera\
                $$PWD/src/RobotView\
                $$PWD/src/Zmotion\
                $$PWD/src/SerialPort\
                $$PWD/src/AdmittanceControl





DEPENDPATH += $$PWD/.

RESOURCES += \
    src/Camera/dahengTwoCams_qt_vs.qrc
