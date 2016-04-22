cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(point2normal)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (point2normal main.cpp)
target_link_libraries (point2normal ${PCL_LIBRARIES})
