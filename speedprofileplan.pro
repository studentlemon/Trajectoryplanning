#-------------------------------------------------
#
# Project created by QtCreator 2021-07-28T10:03:15
#
#-------------------------------------------------
QT -= gui

CONFIG += c++11
CONFIG -= app_bundle
QMAKE_CXXFLAGS += -std=c++11


QT       += core gui
#QMAKE_CXXFLAGS += -std=c++0x
INCLUDEPATH += /usr/include/eigen3
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = speedprofileplan
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
LIBS += /usr/local/lib/libosqp.so \

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    qcustomplot.cpp \
    osqp_problem.cc

HEADERS += \
        mainwindow.h \
    qcustomplot.h \
    data_struct.h \
    util.h \
    types.h \
    scaling.h \
    proj.h \
    polish.h \
    osqp_problem.h \
    osqp_configure.h \
    osqp.h \
    lin_sys.h \
    lin_alg.h \
    kkt.h \
    glob_opts.h \
    error.h \
    data_struct.h \
    ctrlc.h \
    cs.h \
    constants.h \
    auxil.h

FORMS += \
        mainwindow.ui
