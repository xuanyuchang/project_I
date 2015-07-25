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

TEMPLATE = app


SOURCES += main.cpp \
    CalibrationData.cpp \
    structured_light.cpp
INCLUDEPATH+=D:\opencv\release\install\include\
D:\opencv\release\install\include\opencv\
D:\opencv\release\install\include\opencv2
LIBS +=D:\opencv\release\install\lib\libopencv_core244d.dll.a\
       D:\opencv\release\install\lib\libopencv_highgui244d.dll.a\
       D:\opencv\release\install\lib\libopencv_imgproc244d.dll.a\
       D:\opencv\release\install\lib\libopencv_calib3d244d.dll.a

HEADERS += \
    CalibrationData.hpp \
    structured_light.hpp
