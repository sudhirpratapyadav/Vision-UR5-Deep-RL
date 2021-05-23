# Specify the minimum version for CMake
cmake_minimum_required(VERSION 2.8)

# Project's name
project(cpp_practice)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

find_package(OpenCV REQUIRED )
find_package(Lua51 REQUIRED)
find_package(gazebo REQUIRED)


# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}" ${LUA_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(code1 ${PROJECT_SOURCE_DIR}/src/code1.cpp)
add_executable(my_try ${PROJECT_SOURCE_DIR}/src/my_try.cpp)
add_executable(gazebo_interface ${PROJECT_SOURCE_DIR}/src/gazebo_interface.cpp)
target_link_libraries (my_try lua5.1)
target_link_libraries(gazebo_interface ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} pthread)
