# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/VolumeRendering/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/VolumeRendering/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-VolumeRendering "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/VolumeRendering" "VTK_VOLUMERENDERING_EXPORT" "vtkVolumeProMapper.h")
set_tests_properties(HeaderTesting-VolumeRendering PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/VolumeRendering/Testing/CMakeLists.txt;12;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/VolumeRendering/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
