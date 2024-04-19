project(ObjRecRANSAC)
cmake_minimum_required(VERSION 2.6)

find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)


FILE(GLOB _ORR_CUR_     *.cpp)
FILE(GLOB _ORR_ALGS_    Algorithms/*.cpp)
FILE(GLOB _ORR_DS_      DataStructures/*.cpp)
FILE(GLOB _ORR_DS_HTBS_ DataStructures/HashTableBoxStruct/*.cpp)
FILE(GLOB _ORR_DS_HTKT_ DataStructures/HashTableKdTree/*.cpp)
FILE(GLOB _ORR_DS_OKD_  DataStructures/ORRKdTree/*.cpp)
FILE(GLOB _ORR_DS_ORRO_ DataStructures/ORROctree/*.cpp)
FILE(GLOB _ORR_DS_RI_   DataStructures/RangeImage/*.cpp)
FILE(GLOB _ORR_SH_      Shapes/*.cpp)

include_directories(
	${VTK_INCLUDE_DIRS}
	..
)

add_library(ObjRecRANSAC STATIC
	@_ORR_CUR_@
	@_ORR_ALGS_@
	@_ORR_DS_@
	@_ORR_DS_HTBS_@
	@_ORR_DS_HTKT_@
	@_ORR_DS_OKD_@
	@_ORR_DS_ORRO_@
	@_ORR_DS_RI_@
	@_ORR_SH_@
)

