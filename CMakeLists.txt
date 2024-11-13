cmake_minimum_required(VERSION 3.20)

project(ObjRec_RobotGrasping_System)

set(CMAKE_CXX_STANDARD 17)

message(STATUS "Building ObjRec_RobotGrasping_System with ObjRecognition and GraspPlanning modules.")

add_subdirectory(ObjRecognition)
add_subdirectory(GraspPlanning)


