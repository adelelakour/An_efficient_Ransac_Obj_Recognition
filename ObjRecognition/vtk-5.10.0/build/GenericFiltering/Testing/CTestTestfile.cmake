# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/GenericFiltering/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/GenericFiltering/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-GenericFiltering "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/GenericFiltering" "VTK_GENERIC_FILTERING_EXPORT")
set_tests_properties(HeaderTesting-GenericFiltering PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/GenericFiltering/Testing/CMakeLists.txt;8;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/GenericFiltering/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
