# Install script for directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Utilities/vtknetcdf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Development" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vtk-5.10" TYPE STATIC_LIBRARY FILES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/bin/libvtkNetCDF.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Development" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vtk-5.10" TYPE STATIC_LIBRARY FILES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/bin/libvtkNetCDF_cxx.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Development" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vtk-5.10/vtknetcdf" TYPE FILE FILES
    "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Utilities/vtknetcdf/ncconfig.h"
    "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Utilities/vtknetcdf/cxx/netcdfcpp.h"
    "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Utilities/vtknetcdf/cxx/ncvalues.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vtk-5.10/vtknetcdf" TYPE DIRECTORY FILES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Utilities/vtknetcdf/include" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Utilities/vtknetcdf/liblib/cmake_install.cmake")
  include("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Utilities/vtknetcdf/libdispatch/cmake_install.cmake")
  include("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Utilities/vtknetcdf/libsrc/cmake_install.cmake")
  include("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Utilities/vtknetcdf/libsrc4/cmake_install.cmake")
  include("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Utilities/vtknetcdf/cxx/cmake_install.cmake")

endif()

