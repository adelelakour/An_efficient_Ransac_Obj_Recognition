# CMake generated Testfile for 
# Source directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Infovis/Testing
# Build directory: /home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Infovis/Testing
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(HeaderTesting-Infovis "/usr/bin/python3.10" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/Testing/HeaderTesting.py" "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Infovis" "VTK_INFOVIS_EXPORT" "vtkArrayNorm.h" "vtkMultiCorrelativeStatisticsAssessFunctor.h" "vtkKMeansAssessFunctor.h" "vtkBoostGraphAdapter.h" "vtkBoostRandomSparseArraySource.h" "vtkSQLDatabaseGraphSource.h" "vtkSQLDatabaseTableSource.h" "vtkStatisticsAlgorithmPrivate.h")
set_tests_properties(HeaderTesting-Infovis PROPERTIES  _BACKTRACE_TRIPLES "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Infovis/Testing/CMakeLists.txt;12;ADD_TEST;/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Infovis/Testing/CMakeLists.txt;0;")
subdirs("Cxx")
