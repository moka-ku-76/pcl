cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(savepcd-separated-threads-with-queues)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (savepcd-separated-threads-with-queues savepcd-separated-threads-with-queues.cpp)
target_link_libraries (savepcd-separated-threads-with-queues ${PCL_LIBRARIES})
