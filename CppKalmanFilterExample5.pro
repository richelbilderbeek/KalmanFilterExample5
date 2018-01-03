TEMPLATE = app
CONFIG += console
CONFIG -= qt
QMAKE_CXXFLAGS += -Wall -Wextra -Werror -std=c++0x

SOURCES += main.cpp \
    kalmanfilter.cpp \
    whitenoisesystem.cpp \
    matrix.cpp

HEADERS += \
    kalmanfilter.h \
    whitenoisesystem.h \
    matrix.h
