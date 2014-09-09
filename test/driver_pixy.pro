SOURCES += \
    main.cpp \
    usblink.cpp \
    chirpmon.cpp \
    common/chirp.cpp \
    pixycam.cpp \
    helpFunctions.cpp

HEADERS += \
    usblink.h \
    pixy.h \
    common/link.h \
    chirpmon.h \
    common/chirp.hpp \
    helpFunctions.h \
    pixycam.h

QMAKE_CXXFLAGS_DEBUG += -O0
QMAKE_CXXFLAGS += -Wno-unused-parameter
QMAKE_CXXFLAGS += -mno-ms-bitfields

unix:!macx {
    DEFINES += __LINUX__
    PKGCONFIG += libusb-1.0
    LIBS += -lusb-1.0
    INCLUDEPATH += /usr/include/libusb-1.0

#opencv
INCLUDEPATH += /opt/ros/hydro/include
LIBS += -L/opt/ros/hydro/lib \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc


}
