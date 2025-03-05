cmake_minimum_required(VERSION 3.8)
project(rigid2d)

# add compile options
add_compile_options(-Wall -Wextra -Wno-psabi)

# Compile as C++17
set(CMAKE_CXX_STANDARD 17)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(landmark_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)




## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${Eigen3_INCLUDE_DIRS})
include_directories(include ${tf2_ros_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  	src/${PROJECT_NAME}/diff_drive.cpp
  	src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
	src/${PROJECT_NAME}/utilities.cpp
	src/${PROJECT_NAME}/waypoints.cpp
)

ament_export_include_directories(include)
ament_target_dependencies(${PROJECT_NAME} Eigen3)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME} Eigen3 rclcpp nav_msgs landmark_msgs sensor_msgs geometry_msgs)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# add_executable(odometer src/odometry_node.cpp)
# add_executable(fake_diff_encoders src/fake_diff_encoders_node.cpp)


ament_target_dependencies(${PROJECT_NAME}_node rclcpp nav_msgs landmark_msgs sensor_msgs geometry_msgs)
# ament_target_dependencies(odometer rclcpp nav_msgs landmark_msgs sensor_msgs geometry_msgs)
# ament_target_dependencies(fake_diff_encoders rclcpp nav_msgs landmark_msgs sensor_msgs geometry_msgs)




## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}
)

# target_link_libraries(odometer
# 	${PROJECT_NAME}
# )

# target_link_libraries(fake_diff_encoders
# 	${PROJECT_NAME}
# )
#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME} LIBRARY DESTINATION lib INCLUDES DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
# install(TARGETS odometer DESTINATION lib/${PROJECT_NAME})
# install(TARGETS fake_diff_encoders DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
# install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

ament_package()
