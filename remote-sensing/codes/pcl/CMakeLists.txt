cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gss_pcl)

find_package(PCL 1.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (reg_growth reg_growth.cpp)
target_link_libraries (reg_growth ${PCL_LIBRARIES})

add_executable (reg_growth_rgb reg_growth_rgb.cpp)
target_link_libraries (reg_growth_rgb ${PCL_LIBRARIES})

add_executable (converter converter.cpp)
target_link_libraries (converter ${PCL_LIBRARIES})
