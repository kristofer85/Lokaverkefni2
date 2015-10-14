#-------------------------------------------------
#
# Project created by QtCreator 2015-09-25T20:04:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Lokaverkefni2
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    dataholder.cpp \
    stereocalibrate.cpp \
    stereoscopicimage.cpp \
    reprojectimageto3d.cpp

HEADERS  += mainwindow.h \
    dataholder.h \
    stereocalibrate.h \
    stereoscopicimage.h \
    reprojectimageto3d.h

FORMS    += mainwindow.ui


#INCLUDEPATH += C:/msys64/home/kristinn/opencv/build/install/include
INCLUDEPATH += C:/msys64/mingw32/include
LIBS += \
    #-LC:/msys64/home/kristinn/opencv/build/install/x86/mingw/lib \
    -LC:/msys64/mingw32/lib \
    -lopencv_calib3d \
    #-lopencv_contrib \
    -lopencv_core \
    -lopencv_features2d \
    -lopencv_flann \
    #-lopencv_gpu \
    #-lopencv_haartraining_engined \
    -lopencv_highgui \
    -lopencv_imgproc \
    #-lopencv_legacy \
    -lopencv_hal \
    -lopencv_imgcodecs \
    #-lopencv_nonfree \
    -lopencv_objdetect \
    #-lopencv_ocl \
    -lopencv_photo \
    -lopencv_stitching \
    -lopencv_superres \
    -lopencv_ts \
    -lopencv_video \
    -lopencv_videostab \
    -lopencv_ml \
    -lopencv_objdetect \
    -lopencv_photo \
    -lopencv_shape \
    -lopencv_stitching \
    -lopencv_superres \
    -lopencv_videoio
    #-lopencv_viz \
    #-lopencv_ximgproc300 \
    #-lopencv_bgsegm300 \
    #-lopencv_xfeatures2d300

