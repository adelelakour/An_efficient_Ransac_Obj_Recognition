# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Views/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Views/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-Views "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Views" "VTK_VIEWS_EXPORT" "vtkPipelineGraphSource.h")
set_tests_properties(HeaderTesting-Views PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Views/Testing/CMakeLists.txt;12;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Views/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
