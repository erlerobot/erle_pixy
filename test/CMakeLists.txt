cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(libusb libusb-1.0 )
endif()
find_package(OpenCV REQUIRED )

set( CMAKE_CXX_FLAGS " -Wno-unused-parameter -mno-ms-bitfields -O0" )

include_directories(${libusb_INCLUDE_DIRS})
link_directories( ${libusb_library_dirs})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories( ${OpenCV_library_dirs})

add_executable( pixy_cam 
    main.cpp 
    usblink.cpp 
    chirpmon.cpp 
    common/chirp.cpp
    pixycam.cpp
    helpFunctions.cpp
)
			   
target_link_libraries(pixy_cam
    ${libusb_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
