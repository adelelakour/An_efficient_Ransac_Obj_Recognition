project(VtkVisual)
cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0053 OLD)

find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)


FILE(GLOB _VTK_VIS_ *.cpp)

include_directories(${VTK_INCLUDE_DIRS} ..)

if((CMAKE_COMPILER_IS_GNUCXX) AND (NOT DEFINED CMAKE_CXX_FLAGS))
	set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

add_library(VtkVisual STATIC
	@_VTK_VIS_@
)
