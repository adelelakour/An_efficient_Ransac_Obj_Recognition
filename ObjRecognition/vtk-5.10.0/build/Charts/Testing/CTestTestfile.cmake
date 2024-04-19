# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Charts/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Charts/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-Charts "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Charts" "VTK_CHARTS_EXPORT" "vtkContextKeyEvent.h" "vtkContextMouseEvent.h" "vtkContextScenePrivate.h" "vtkOpenGLContextDevice2DPrivate.h")
set_tests_properties(HeaderTesting-Charts PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Charts/Testing/CMakeLists.txt;12;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Charts/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
