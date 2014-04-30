cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Meshing)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(merge merge.cpp)
target_link_libraries(merge ${PCL_LIBRARIES})

configure_file(../bal1.pcd bal1.pcd COPYONLY)
configure_file(../jobb1.pcd jobb1.pcd COPYONLY)
add_executable(feature_matching feature_matching.cpp)
target_link_libraries(feature_matching ${PCL_LIBRARIES})

add_executable(texturing texturing.cpp)
target_link_libraries(texturing ${PCL_LIBRARIES})

add_executable(meshing meshing.cpp)
target_link_libraries(meshing ${PCL_LIBRARIES})

add_executable(filtering filtering.cpp)
target_link_libraries(filtering ${PCL_LIBRARIES})

add_executable(downsampling downsampling.cpp)
target_link_libraries(downsampling ${PCL_LIBRARIES})
