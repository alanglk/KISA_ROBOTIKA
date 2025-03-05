cmake_minimum_required(VERSION 3.8)
project(landmarks)

add_compile_options(-Wall -Wextra -Wno-psabi)

# Compile as C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()
# don't use gnu extensions
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(landmark_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rigid2d REQUIRED)
#find_package(tsim REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(include ${tf2_ros_INCLUDE_DIRS})
include_directories(include ${Eigen3_INCLUDE_DIRS})

## Declare a C++ library
add_library(landmarks SHARED
	src/${PROJECT_NAME}/ekf_filter.cpp
  	src/${PROJECT_NAME}/landmarks.cpp
)
ament_export_include_directories(include)
ament_target_dependencies(landmarks Eigen3 rigid2d)
ament_export_targets(landmarks HAS_LIBRARY_TARGET)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(analysis src/analysis_node.cpp)
add_executable(draw_map src/draw_map_node.cpp)
add_executable(landmarks_node src/landmarks_node.cpp)
#add_executable(slam src/slam_node.cpp)


# add_dependencies(analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(draw_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(landmarks_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against

target_link_libraries(draw_map
	landmarks
	${rigid2d_LIBRARIES}
)


target_link_libraries(landmarks_node
  	landmarks
	${rigid2d_LIBRARIES}
)

# target_link_libraries(slam
#   	landmarks
# 	${tsim_LIBRARIES}
# 	${rigid2d_LIBRARIES}
# )
ament_target_dependencies(landmarks_node rclcpp std_msgs landmark_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(draw_map rclcpp std_msgs landmark_msgs visualization_msgs)
#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}
	DESTINATION include)

install(TARGETS landmarks
	EXPORT landmarks 
	LIBRARY DESTINATION lib 
	INCLUDES DESTINATION include
)

#install(TARGETS landmarks DESTINATION lib/${PROJECT_NAME})
install(TARGETS landmarks_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS draw_map DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()