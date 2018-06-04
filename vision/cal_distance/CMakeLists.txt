cmake_minimum_required(VERSION 2.8.3)
project(cal_distance)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(cal_distance
  src/cal_distance.cpp
)
add_dependencies(cal_distance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cal_distance
  ${catkin_LIBRARIES}
)