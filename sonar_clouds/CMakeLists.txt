cmake_minimum_required(VERSION 2.8.3)
project(sonar_clouds)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_clouds src/sonar_clouds.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sonar_clouds 	${PCL_LIBRARIES}
  										${catkin_LIBRARIES}
						)
