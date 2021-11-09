cmake_minimum_required(VERSION 2.8.3)
project(publish_pointcloud)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	rostime 
	sensor_msgs 
	message_filters 
	tf
	octomap_ros
)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_cameracalibrate
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_executable(publish_pointcloud src/publish_pointcloud.cpp)

target_link_libraries(publish_pointcloud
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(pcl_test 
src/main.cpp
src/pcl_test_core.cpp
)

target_link_libraries(pcl_test
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

# add_executable(pcl_outofcore src/pcl_outofcore.cpp)

# target_link_libraries(pcl_outofcore
# 	${catkin_LIBRARIES}
# 	${OpenCV_LIBRARIES}
# 	${PCL_LIBRARIES}
# )

# add_executable (RANSAC src/RANSAC.cpp)
# target_link_libraries (RANSAC ${PCL_LIBRARIES})