cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)

PROJECT ( Grasper )

find_package(Simox REQUIRED)
include_directories(/usr/local/include/nlohmann)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)


    FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/GraspPlanner.cpp ${PROJECT_SOURCE_DIR}/GraspPlannerWindow.cpp
            ${PROJECT_SOURCE_DIR}/myGraspPlannerWindow.cpp)
    FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/GraspPlannerWindow.h ${PROJECT_SOURCE_DIR}/myGraspPlannerWindow.h )

    set(GUI_MOC_HDRS
            ${PROJECT_SOURCE_DIR}/GraspPlannerWindow.h
            ${PROJECT_SOURCE_DIR}/myGraspPlannerWindow.h

    )
    set(GUI_UIS
            ${PROJECT_SOURCE_DIR}/GraspPlanner.ui
    )


    SimoxQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")




ENDIF()