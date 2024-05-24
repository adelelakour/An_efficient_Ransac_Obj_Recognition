project(Kinect3dObjRec)
cmake_minimum_required(VERSION 3.20)

set(VTK_DIR ${CMAKE_CURRENT_LIST_DIR}/vtk-install/lib/vtk-5.10)

set (MODEL_DIR ${CMAKE_SOURCE_DIR}/models/vtk/)

cmake_policy(SET CMP0053 OLD)
find_package(VTK)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

message("This program uses VTK ${VTK_DIR} .. ")

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)
set(vtk_LIBS vtkIO vtkCommon vtkRendering pthread rt GL GLU glut)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED COMPONENTS highgui)

IF(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. OpenCV is not installed or cmake can not find it. Install OpenCV first and then try again.
If OpenCV is installed but cmake can not find it, set the OPENCV_DIR entry to the folder which contains the file OpenCVConfig.cmake")
ENDIF(NOT OpenCV_FOUND)

if (${CMAKE_BUILD_TYPE_LOWERCASE} MATCHES release)
    SET(CMAKE_CXX_FLAGS "-O3")
endif ()
if(CMAKE_COMPILER_IS_GNUCXX)
    message(STATUS "Add more CXX flags...")
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -fPIE ${CMAKE_CXX_FLAGS}")
endif()


include_directories(/usr/local/include/AndreiUtils)
include_directories(/usr/local/include/nlohmann)

# inherit all CMAKE_CXX_FLAGS in subprojects
add_subdirectory(src/BasicTools)
add_subdirectory(src/BasicToolsVis)
add_subdirectory(src/ObjRecRANSAC)
add_subdirectory(src/ObjRecRANSACVis)
add_subdirectory(src/VtkBasics)
add_subdirectory(src/VtkVisual)

include_directories(${CMAKE_BINARY_DIR} /include/json/include/nlohmann /usr/local/include/AndreiUtils)


message("VTK LIB DIR: ${VTK_LIBRARY_DIRS}")
message("OpenCV LIB DIR: ${OpenCV_LIB_DIR}")
message("Opencv Includes: ${OpenCV_INCLUDE_DIRS}")
message("Opencv Libraries: ${OpenCV_LIBS}")

# link_libraries(BasicToolsVis ObjRecRANSAC ObjRecRANSACVis VtkBasics VtkVisual vtkIO vtkCommon vtkRendering pthread rt GL GLU glut BasicTools ${OpenCV_LIBS} OpenNI)

#add_executable(Kinect3dObjRec ./KinectMotor.cpp ./main.cpp ./utils.cpp)
#target_link_libraries(Kinect3dObjRec BasicToolsVis ObjRecRANSAC ObjRecRANSACVis VtkBasics VtkVisual ${vtk_LIBS} BasicTools ${OpenCV_LIBS} OpenNI)
#target_link_directories(Kinect3dObjRec PUBLIC ${VTK_LIBRARY_DIRS})
#target_include_directories(Kinect3dObjRec PUBLIC ${VTK_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ./src)

#add_executable(YCB_to_VTK ./fromYCBtoVTK.cpp)

add_executable(RealsenseObjRec ./mainRealsense.cpp ./utils.cpp
        CustomTypes.h)
#target_link_libraries(RealsenseObjRec BasicToolsVis ObjRecRANSAC ObjRecRANSACVis VtkBasics VtkVisual ${vtk_LIBS} BasicTools ${OpenCV_LIBS} OpenNI)



target_link_libraries(RealsenseObjRec
        BasicToolsVis
        ObjRecRANSAC
        ObjRecRANSACVis
        VtkBasics
        VtkVisual
        ${vtk_LIBS}
        BasicTools
        ${OpenCV_LIBS}
        ${AndreiUtils_LIBRARY}
        OpenNI
        realsense2
        glfw
        ${OPENGL_LIBRARIES}
        Eigen3::Eigen
)

target_link_directories(RealsenseObjRec PUBLIC ${VTK_LIBRARY_DIRS})
target_include_directories(RealsenseObjRec PUBLIC ${VTK_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ./src)

