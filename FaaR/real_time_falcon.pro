#-------------------------------------------------
#
# Project created by QtCreator 2016-07-06T09:45:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = real_time_falcon
TEMPLATE = app


SOURCES += main.cpp\
    src/comm/FalconCommLibUSB.cpp \
    src/core/FalconDevice.cpp \
    src/core/FalconFirmware.cpp \
    src/firmware/FalconFirmwareNovintSDK.cpp \
    src/kinematic/FalconKinematicStamper.cpp \
    src/OMD/rungekutta.cpp \
    src/OMD/optosource.cpp \
    src/util/FalconCLIBase.cpp \
    src/util/FalconDeviceBoostThread.cpp \
    src/threads/falconthreads.cpp \
    controlForGui.cpp \
    src/threads/optothread.cpp \
    qcustomplot.cpp \
    src/trajectoryGeneration/maketrajectory.cpp \
    faar.cpp \
    pdcontrol.cpp


HEADERS  +=\
            falcon/comm/FalconCommFTD2XX.h \
    falcon/kinematic/FalconKinematicStamper.h \
    falcon/core/FalconComm.h \
    falcon/core/FalconCore.h \
    falcon/core/FalconDevice.h \
    falcon/core/FalconFirmware.h \
    falcon/core/FalconGeometry.h \
    falcon/core/FalconGrip.h \
    falcon/core/FalconKinematic.h \
    falcon/core/FalconLogger.h \
    falcon/firmware/FalconFirmwareNovintSDK.h \
    falcon/gmtl/External/OpenSGConvert.h \
    falcon/gmtl/Fit/GaussPointsFit.h \
    falcon/gmtl/Misc/MatrixConvert.h \
    falcon/gmtl/Numerics/Eigen.h \
    falcon/gmtl/Util/Assert.h \
    falcon/gmtl/Util/Meta.h \
    falcon/gmtl/Util/StaticAssert.h \
    falcon/gmtl/AABox.h \
    falcon/gmtl/AABoxOps.h \
    falcon/gmtl/AxisAngle.h \
    falcon/gmtl/AxisAngleOps.h \
    falcon/gmtl/Comparitors.h \
    falcon/gmtl/Config.h \
    falcon/gmtl/Containment.h \
    falcon/gmtl/Coord.h \
    falcon/gmtl/CoordOps.h \
    falcon/gmtl/Defines.h \
    falcon/gmtl/EulerAngle.h \
    falcon/gmtl/EulerAngleOps.h \
    falcon/gmtl/Frustum.h \
    falcon/gmtl/FrustumOps.h \
    falcon/gmtl/Generate.h \
    falcon/gmtl/gmtl.h \
    falcon/gmtl/Helpers.h \
    falcon/gmtl/Intersection.h \
    falcon/gmtl/LineSeg.h \
    falcon/gmtl/LineSegOps.h \
    falcon/gmtl/Math.h \
    falcon/gmtl/Matrix.h \
    falcon/gmtl/MatrixOps.h \
    falcon/gmtl/OOBox.h \
    falcon/gmtl/Output.h \
    falcon/gmtl/Plane.h \
    falcon/gmtl/PlaneOps.h \
    falcon/gmtl/Point.h \
    falcon/gmtl/Quat.h \
    falcon/gmtl/QuatOps.h \
    falcon/gmtl/Ray.h \
    falcon/gmtl/RayOps.h \
    falcon/gmtl/Sphere.h \
    falcon/gmtl/SphereOps.h \
    falcon/gmtl/Tri.h \
    falcon/gmtl/TriOps.h \
    falcon/gmtl/Vec.h \
    falcon/gmtl/VecBase.h \
    falcon/gmtl/VecExprMeta.h \
    falcon/gmtl/VecOps.h \
    falcon/gmtl/VecOpsMeta.h \
    falcon/gmtl/Version.h \
    falcon/gmtl/Xforms.h \
    falcon/grip/FalconGripFourButton.h \
    falcon/kinematic/stamper/StamperUtils.h \
    falcon/OMD/optosource.h \
    falcon/OMD/rungekutta.h \
    falcon/util/FalconCLIBase.h \
    falcon/util/FalconDeviceBoostThread.h \
    falcon/util/FalconFirmwareBinaryNvent.h \
    falcon/util/FalconFirmwareBinaryTest.h \
    falcon/threads/falconthreads.h \
    controlForGui.h \
    falcon/threads/optothread.h \
    qcustomplot.h \
    falcon/trajectoryGeneration/maketrajectory.h \
    faar.h \
    pdcontrol.h \
    falconDataStruct.h

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lnifalcon

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/libnifalcon.a

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lnifalcon_cli_base

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/libnifalcon_cli_base.a

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lnifalcon_device_thread

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/libnifalcon_device_thread.a

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lusb-1.0

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/libusb-1.0.a

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lOMD

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/x86_64-linux-gnu/ -lboost_system

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/x86_64-linux-gnu/ -lboost_chrono

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/x86_64-linux-gnu/ -lboost_date_time

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/x86_64-linux-gnu/ -lboost_regex

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/x86_64-linux-gnu/ -lboost_thread

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/x86_64-linux-gnu/ -lboost_program_options

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

LIBS += -L/usr/xenomai/lib -lnative -lxenomai -lpthread -lrt

INCLUDEPATH += /usr/xenomai/include

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lorocos-kdl

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

CONFIG += console

OTHER_FILES +=

RESOURCES += \
    resources.qrc

DISTFILES +=
