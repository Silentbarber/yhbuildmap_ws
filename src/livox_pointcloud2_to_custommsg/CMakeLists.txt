cmake_minimum_required(VERSION 3.0.2)
project(livox_pointcloud2_to_custommsg)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs livox_ros_driver)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(pointcloud2_to_custommsg src/pointcloud2_to_custommsg.cpp)
add_dependencies(pointcloud2_to_custommsg ${catkin_EXPORTED_TARGETS})
target_compile_features(pointcloud2_to_custommsg PRIVATE cxx_std_14)
target_link_libraries(pointcloud2_to_custommsg ${catkin_LIBRARIES})
add_executable(imu_g_to_si src/imu_g_to_si.cpp)
target_compile_features(imu_g_to_si PRIVATE cxx_std_14)
target_link_libraries(imu_g_to_si ${catkin_LIBRARIES})

install(TARGETS pointcloud2_to_custommsg imu_g_to_si
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
