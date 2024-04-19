# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Hybrid/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Hybrid/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-Hybrid "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Hybrid" "VTK_HYBRID_EXPORT" "vtk3DS.h" "vtkExodusIIReaderParser.h" "vtkExodusIIReaderPrivate.h" "vtkExodusIIReaderVariableCheck.h" "vtkVRML.h" "vtkX3D.h" "vtkX3DExporterFIWriterHelper.h")
set_tests_properties(HeaderTesting-Hybrid PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Hybrid/Testing/CMakeLists.txt;14;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Hybrid/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
