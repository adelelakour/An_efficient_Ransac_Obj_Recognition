# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/IO/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/IO/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-IO "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/IO" "VTK_IO_EXPORT" "vtkBase64Utilities.h" "vtkMINC.h" "vtkMySQLDatabasePrivate.h" "vtkODBCInternals.h" "vtkOffsetsManagerArray.h" "vtkPLY.h" "vtkPostgreSQLDatabasePrivate.h" "vtkXMLUtilities.h" "vtkXMLWriterC.h" "vtkXMLWriterF.h")
set_tests_properties(HeaderTesting-IO PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/IO/Testing/CMakeLists.txt;12;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/IO/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
