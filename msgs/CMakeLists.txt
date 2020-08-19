find_package(catkin REQUIRED COMPONENTS genmsg)

add_message_files(
        DIRECTORY msg
        FILES
        ArucoDetection.msg
        ArucoDetections.msg
        CameraDetection.msg
        CameraDetections.msg
        OptimizationStats.msg
)

generate_messages(DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        )
