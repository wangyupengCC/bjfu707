cmake_minimum_required(VERSION 2.8.8)
project(Map_Location)
set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_STANDARD 11)
set( DBoW3_INCLUDE_DIRS "/usr/local/include")
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so")
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)


include_directories(
 include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${DBoW3_INCLUDE_DIRS}
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(Map_Location SHARED
        src/MapLocation.cpp
        src/LoadImages.cpp
        src/extractor.cpp
        )

target_link_libraries(Map_Location
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        ${DBoW3_LIBS}
        )

# add_executable(MapLocation
#         src/MapLocation_node.cpp
# )
# target_link_libraries(
#         MapLocation
#         ${PROJECT_NAME}
# )
# target_include_directories(Map_Location_lib PUBLIC src)

