cmake_minimum_required(VERSION 2.8.3)
project(beginner_camera)
include(FindPkgConfig)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
)

if (PKG_CONFIG_FOUND)
    pkg_check_modules(LIBUSB QUIET libusb-1.0)
    if (LIBUSB_INCLUDE_DIRS)
	    MESSAGE("*** libusb FOUND")
	    SET(DEPS "${DEPS} libusb-1.0-0")
    else()
	    MESSAGE("*** libusb NOT FOUND")
    endif()

endif()
find_package(OpenCV REQUIRED)

FIND_PACKAGE( Boost 1.40 COMPONENTS thread REQUIRED )

catkin_package()

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

add_executable(usb_cam src/usb_cam/camera_opencv.cpp)
target_link_libraries(usb_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(usb_cam beginner_camera)

add_executable(gopro_cam src/gopro/gopro.cpp)
target_link_libraries(gopro_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(gopro_cam beginner_camera)

###########

include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories( ${LIBUSB_LIBRARY_DIRS})


add_executable( pixy_cam 
    src/pixy/erle_pixy.cpp
    src/pixy/helpFunctions.cpp
    src/pixy/pixycam.cpp
    src/pixy/usblink.cpp 
    src/pixy/chirpmon.cpp 
    src/pixy/common/chirp.cpp

)

target_link_libraries(pixy_cam
	${LIBUSB_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

install(TARGETS pixy_cam usb_cam gopro_cam
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)