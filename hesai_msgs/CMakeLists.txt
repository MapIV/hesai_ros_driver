cmake_minimum_required(VERSION 3.5)
project(hesai_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    DIRECTORY msg
    FILES
        PandarPacket.msg
        PandarScan.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
)
