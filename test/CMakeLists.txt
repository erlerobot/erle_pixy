cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(OpenCV opencv)
	pkg_check_modules(libusb libusb-1.0 )
endif()

FIND_PACKAGE(Qt4 REQUIRED)

SET(qt_SOURCES     main.cpp 
    usblink.cpp 
    chirpmon.cpp 
    common/chirp.cpp

)

SET(qt_HEADERS     usblink.h 
    pixy.h 
    common/link.h 
    chirpmon.h 
    common/chirp.hpp
)



set( CMAKE_CXX_FLAGS " -Wno-unused-parameter -mno-ms-bitfields -O0" )


QT4_WRAP_CPP(qt_HEADERS_MOC ${qt_HEADERS})
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})



include_directories(${opencv_INCLUDE_DIRS})
link_directories( ${opencv_library_dirs})

include_directories(${libusb_INCLUDE_DIRS})
link_directories( ${libusb_library_dirs})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories( ${OpenCV_library_dirs})

add_executable( pixy_cam main.cpp 
    usblink.cpp 
    chirpmon.cpp 
    common/chirp.cpp
    ${qt_HEADERS_MOC}
)


Message(${OpenCV_LIBRARIES})
			   
target_link_libraries(pixy_cam
	usb-1.0
	${QT_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


