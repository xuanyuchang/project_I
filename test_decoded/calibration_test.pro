#-------------------------------------------------
#
# Project created by QtCreator 2015-07-23T09:02:02
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = calibration_test
CONFIG   += console
CONFIG   -= app_bundle
QT += widgets
TEMPLATE = app


SOURCES += main.cpp \
    CalibrationData.cpp \
    structured_light.cpp \
    Application.cpp
INCLUDEPATH+=F:\opencv2.4.4\opencv\release\install\include\
F:\opencv2.4.4\opencv\release\install\include\opencv\
F:\opencv2.4.4\opencv\release\install\include\opencv2
LIBS +=D:\Qt_test\Project02_3D\OKAPI32.lib\
       F:\opencv2.4.4\opencv\release\install\lib\libopencv_core244d.dll.a\
       F:\opencv2.4.4\opencv\release\install\lib\libopencv_highgui244d.dll.a\
       F:\opencv2.4.4\opencv\release\install\lib\libopencv_imgproc244d.dll.a\
       F:\opencv2.4.4\opencv\release\install\lib\libopencv_calib3d244d.dll.a
HEADERS += \
    CalibrationData.hpp \
    structured_light.hpp \
    Application.hpp
