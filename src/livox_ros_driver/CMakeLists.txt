cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)
# Message-only adaptation for recorded bags, 2026-10-03. No hardware SDK.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES CustomMsg.msg CustomPoint.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
