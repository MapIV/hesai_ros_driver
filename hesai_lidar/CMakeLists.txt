cmake_minimum_required(VERSION 3.5)
project(hesai_lidar)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    image_transport
    pcl_conversions
    hesai_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        roslib
        std_msgs
        sensor_msgs
        image_transport
        pcl_conversions
        hesai_msgs
)

###PandarGeneralRaw
add_library(PandarGeneral
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/input.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral_internal.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pcap_reader.cpp
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral.cc
)
target_include_directories(PandarGeneral PRIVATE
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    pcap
)
add_dependencies(PandarGeneral ${catkin_EXPORTED_TARGETS})

###PandarGeneralSDK
add_library(PandarGeneralSDK SHARED
    src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc
    src/HesaiLidar_General_SDK/src/tcp_command_client.c
    src/HesaiLidar_General_SDK/src/util.c
)

target_include_directories(PandarGeneralSDK PRIVATE
    src/HesaiLidar_General_SDK/
    src/HesaiLidar_General_SDK/include/
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(PandarGeneralSDK 
    PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
    pcap
)

add_dependencies(PandarGeneralSDK ${catkin_EXPORTED_TARGETS})

###hesai node
add_executable(hesai_lidar_node
    src/main.cc
)

target_include_directories(hesai_lidar_node PRIVATE
    src/HesaiLidar_General_SDK/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(hesai_lidar_node 
    ${catkin_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    PandarGeneralSDK
)

install(TARGETS
    hesai_lidar_node
    PandarGeneral
    PandarGeneralSDK
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
