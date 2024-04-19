#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vtksys" for configuration "Release"
set_property(TARGET vtksys APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtksys PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "dl"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtksys.a"
  )

list(APPEND _cmake_import_check_targets vtksys )
list(APPEND _cmake_import_check_files_for_vtksys "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtksys.a" )

# Import target "vtkzlib" for configuration "Release"
set_property(TARGET vtkzlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkzlib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkzlib.a"
  )

list(APPEND _cmake_import_check_targets vtkzlib )
list(APPEND _cmake_import_check_files_for_vtkzlib "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkzlib.a" )

# Import target "vtkhdf5" for configuration "Release"
set_property(TARGET vtkhdf5 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkhdf5 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "m;vtkzlib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkhdf5.a"
  )

list(APPEND _cmake_import_check_targets vtkhdf5 )
list(APPEND _cmake_import_check_files_for_vtkhdf5 "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkhdf5.a" )

# Import target "vtkhdf5_hl" for configuration "Release"
set_property(TARGET vtkhdf5_hl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkhdf5_hl PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkhdf5"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkhdf5_hl.a"
  )

list(APPEND _cmake_import_check_targets vtkhdf5_hl )
list(APPEND _cmake_import_check_files_for_vtkhdf5_hl "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkhdf5_hl.a" )

# Import target "vtkjpeg" for configuration "Release"
set_property(TARGET vtkjpeg APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkjpeg PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkjpeg.a"
  )

list(APPEND _cmake_import_check_targets vtkjpeg )
list(APPEND _cmake_import_check_files_for_vtkjpeg "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkjpeg.a" )

# Import target "vtkpng" for configuration "Release"
set_property(TARGET vtkpng APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkpng PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkzlib;-lm"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkpng.a"
  )

list(APPEND _cmake_import_check_targets vtkpng )
list(APPEND _cmake_import_check_files_for_vtkpng "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkpng.a" )

# Import target "vtktiff" for configuration "Release"
set_property(TARGET vtktiff APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtktiff PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkzlib;vtkjpeg;-lm"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtktiff.a"
  )

list(APPEND _cmake_import_check_targets vtktiff )
list(APPEND _cmake_import_check_files_for_vtktiff "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtktiff.a" )

# Import target "vtkexpat" for configuration "Release"
set_property(TARGET vtkexpat APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkexpat PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkexpat.a"
  )

list(APPEND _cmake_import_check_targets vtkexpat )
list(APPEND _cmake_import_check_files_for_vtkexpat "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkexpat.a" )

# Import target "vtkfreetype" for configuration "Release"
set_property(TARGET vtkfreetype APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkfreetype PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkfreetype.a"
  )

list(APPEND _cmake_import_check_targets vtkfreetype )
list(APPEND _cmake_import_check_files_for_vtkfreetype "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkfreetype.a" )

# Import target "vtklibxml2" for configuration "Release"
set_property(TARGET vtklibxml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtklibxml2 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkzlib;dl;dl;m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtklibxml2.a"
  )

list(APPEND _cmake_import_check_targets vtklibxml2 )
list(APPEND _cmake_import_check_files_for_vtklibxml2 "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtklibxml2.a" )

# Import target "vtkDICOMParser" for configuration "Release"
set_property(TARGET vtkDICOMParser APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkDICOMParser PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkDICOMParser.a"
  )

list(APPEND _cmake_import_check_targets vtkDICOMParser )
list(APPEND _cmake_import_check_files_for_vtkDICOMParser "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkDICOMParser.a" )

# Import target "vtkproj4" for configuration "Release"
set_property(TARGET vtkproj4 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkproj4 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkproj4.a"
  )

list(APPEND _cmake_import_check_targets vtkproj4 )
list(APPEND _cmake_import_check_files_for_vtkproj4 "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkproj4.a" )

# Import target "mpistubs" for configuration "Release"
set_property(TARGET mpistubs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mpistubs PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libmpistubs.a"
  )

list(APPEND _cmake_import_check_targets mpistubs )
list(APPEND _cmake_import_check_files_for_mpistubs "${_IMPORT_PREFIX}/lib/vtk-5.10/libmpistubs.a" )

# Import target "MapReduceMPI" for configuration "Release"
set_property(TARGET MapReduceMPI APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(MapReduceMPI PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "mpistubs"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libMapReduceMPI.a"
  )

list(APPEND _cmake_import_check_targets MapReduceMPI )
list(APPEND _cmake_import_check_files_for_MapReduceMPI "${_IMPORT_PREFIX}/lib/vtk-5.10/libMapReduceMPI.a" )

# Import target "vtkverdict" for configuration "Release"
set_property(TARGET vtkverdict APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkverdict PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkverdict.a"
  )

list(APPEND _cmake_import_check_targets vtkverdict )
list(APPEND _cmake_import_check_files_for_vtkverdict "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkverdict.a" )

# Import target "vtkNetCDF" for configuration "Release"
set_property(TARGET vtkNetCDF APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkNetCDF PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkhdf5;vtkhdf5_hl"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkNetCDF.a"
  )

list(APPEND _cmake_import_check_targets vtkNetCDF )
list(APPEND _cmake_import_check_files_for_vtkNetCDF "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkNetCDF.a" )

# Import target "vtkNetCDF_cxx" for configuration "Release"
set_property(TARGET vtkNetCDF_cxx APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkNetCDF_cxx PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkNetCDF"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkNetCDF_cxx.a"
  )

list(APPEND _cmake_import_check_targets vtkNetCDF_cxx )
list(APPEND _cmake_import_check_files_for_vtkNetCDF_cxx "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkNetCDF_cxx.a" )

# Import target "vtkmetaio" for configuration "Release"
set_property(TARGET vtkmetaio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkmetaio PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkzlib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkmetaio.a"
  )

list(APPEND _cmake_import_check_targets vtkmetaio )
list(APPEND _cmake_import_check_files_for_vtkmetaio "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkmetaio.a" )

# Import target "vtksqlite" for configuration "Release"
set_property(TARGET vtksqlite APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtksqlite PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtksqlite.a"
  )

list(APPEND _cmake_import_check_targets vtksqlite )
list(APPEND _cmake_import_check_files_for_vtksqlite "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtksqlite.a" )

# Import target "vtkexoIIc" for configuration "Release"
set_property(TARGET vtkexoIIc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkexoIIc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkNetCDF"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkexoIIc.a"
  )

list(APPEND _cmake_import_check_targets vtkexoIIc )
list(APPEND _cmake_import_check_files_for_vtkexoIIc "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkexoIIc.a" )

# Import target "LSDyna" for configuration "Release"
set_property(TARGET LSDyna APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LSDyna PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libLSDyna.a"
  )

list(APPEND _cmake_import_check_targets LSDyna )
list(APPEND _cmake_import_check_files_for_LSDyna "${_IMPORT_PREFIX}/lib/vtk-5.10/libLSDyna.a" )

# Import target "vtkalglib" for configuration "Release"
set_property(TARGET vtkalglib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkalglib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkalglib.a"
  )

list(APPEND _cmake_import_check_targets vtkalglib )
list(APPEND _cmake_import_check_files_for_vtkalglib "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkalglib.a" )

# Import target "vtkEncodeString" for configuration "Release"
set_property(TARGET vtkEncodeString APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkEncodeString PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkEncodeString"
  )

list(APPEND _cmake_import_check_targets vtkEncodeString )
list(APPEND _cmake_import_check_files_for_vtkEncodeString "${_IMPORT_PREFIX}/bin/vtkEncodeString" )

# Import target "VPIC" for configuration "Release"
set_property(TARGET VPIC APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(VPIC PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libVPIC.a"
  )

list(APPEND _cmake_import_check_targets VPIC )
list(APPEND _cmake_import_check_files_for_VPIC "${_IMPORT_PREFIX}/lib/vtk-5.10/libVPIC.a" )

# Import target "Cosmo" for configuration "Release"
set_property(TARGET Cosmo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Cosmo PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtksys;vtkCommon"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libCosmo.a"
  )

list(APPEND _cmake_import_check_targets Cosmo )
list(APPEND _cmake_import_check_files_for_Cosmo "${_IMPORT_PREFIX}/lib/vtk-5.10/libCosmo.a" )

# Import target "vtkftgl" for configuration "Release"
set_property(TARGET vtkftgl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkftgl PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib/x86_64-linux-gnu/libGL.so;vtkfreetype"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkftgl.a"
  )

list(APPEND _cmake_import_check_targets vtkftgl )
list(APPEND _cmake_import_check_files_for_vtkftgl "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkftgl.a" )

# Import target "vtkCommon" for configuration "Release"
set_property(TARGET vtkCommon APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommon PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtksys;-lm;dl"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkCommon.a"
  )

list(APPEND _cmake_import_check_targets vtkCommon )
list(APPEND _cmake_import_check_files_for_vtkCommon "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkCommon.a" )

# Import target "vtkFiltering" for configuration "Release"
set_property(TARGET vtkFiltering APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltering PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkCommon"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkFiltering.a"
  )

list(APPEND _cmake_import_check_targets vtkFiltering )
list(APPEND _cmake_import_check_files_for_vtkFiltering "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkFiltering.a" )

# Import target "vtkImaging" for configuration "Release"
set_property(TARGET vtkImaging APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImaging PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkFiltering"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkImaging.a"
  )

list(APPEND _cmake_import_check_targets vtkImaging )
list(APPEND _cmake_import_check_files_for_vtkImaging "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkImaging.a" )

# Import target "vtkGraphics" for configuration "Release"
set_property(TARGET vtkGraphics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGraphics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkFiltering;vtkverdict"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkGraphics.a"
  )

list(APPEND _cmake_import_check_targets vtkGraphics )
list(APPEND _cmake_import_check_files_for_vtkGraphics "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkGraphics.a" )

# Import target "vtkGenericFiltering" for configuration "Release"
set_property(TARGET vtkGenericFiltering APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGenericFiltering PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkFiltering;vtkGraphics"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkGenericFiltering.a"
  )

list(APPEND _cmake_import_check_targets vtkGenericFiltering )
list(APPEND _cmake_import_check_files_for_vtkGenericFiltering "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkGenericFiltering.a" )

# Import target "vtkIO" for configuration "Release"
set_property(TARGET vtkIO APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIO PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkFiltering;vtkDICOMParser;vtkNetCDF;vtkNetCDF_cxx;LSDyna;vtkmetaio;vtksqlite;vtkpng;vtkzlib;vtkjpeg;vtktiff;vtkexpat;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkIO.a"
  )

list(APPEND _cmake_import_check_targets vtkIO )
list(APPEND _cmake_import_check_files_for_vtkIO "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkIO.a" )

# Import target "vtkRendering" for configuration "Release"
set_property(TARGET vtkRendering APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRendering PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkGraphics;vtkImaging;vtkIO;vtkftgl;vtkfreetype;/usr/lib/x86_64-linux-gnu/libGL.so;/usr/lib/x86_64-linux-gnu/libXt.so;/usr/lib/x86_64-linux-gnu/libSM.so;/usr/lib/x86_64-linux-gnu/libICE.so;/usr/lib/x86_64-linux-gnu/libX11.so;/usr/lib/x86_64-linux-gnu/libXext.so"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkRendering.a"
  )

list(APPEND _cmake_import_check_targets vtkRendering )
list(APPEND _cmake_import_check_files_for_vtkRendering "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkRendering.a" )

# Import target "vtkVolumeRendering" for configuration "Release"
set_property(TARGET vtkVolumeRendering APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkVolumeRendering PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkRendering;vtkIO;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkVolumeRendering.a"
  )

list(APPEND _cmake_import_check_targets vtkVolumeRendering )
list(APPEND _cmake_import_check_files_for_vtkVolumeRendering "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkVolumeRendering.a" )

# Import target "vtkHybrid" for configuration "Release"
set_property(TARGET vtkHybrid APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkHybrid PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkRendering;vtkIO;vtkParallel;vtkexoIIc;vtkftgl"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkHybrid.a"
  )

list(APPEND _cmake_import_check_targets vtkHybrid )
list(APPEND _cmake_import_check_files_for_vtkHybrid "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkHybrid.a" )

# Import target "vtkWidgets" for configuration "Release"
set_property(TARGET vtkWidgets APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkWidgets PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkRendering;vtkHybrid;vtkVolumeRendering;/usr/lib/x86_64-linux-gnu/libGL.so"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkWidgets.a"
  )

list(APPEND _cmake_import_check_targets vtkWidgets )
list(APPEND _cmake_import_check_files_for_vtkWidgets "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkWidgets.a" )

# Import target "vtkParallel" for configuration "Release"
set_property(TARGET vtkParallel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkParallel PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkRendering;vtkIO;VPIC;Cosmo;LSDyna;/usr/lib/x86_64-linux-gnu/libGL.so;vtkexoIIc"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkParallel.a"
  )

list(APPEND _cmake_import_check_targets vtkParallel )
list(APPEND _cmake_import_check_files_for_vtkParallel "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkParallel.a" )

# Import target "vtkInfovis" for configuration "Release"
set_property(TARGET vtkInfovis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkInfovis PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkWidgets;vtkParallel;vtklibxml2;vtkalglib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkInfovis.a"
  )

list(APPEND _cmake_import_check_targets vtkInfovis )
list(APPEND _cmake_import_check_files_for_vtkInfovis "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkInfovis.a" )

# Import target "vtkGeovis" for configuration "Release"
set_property(TARGET vtkGeovis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGeovis PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkWidgets;vtkViews;vtkproj4"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkGeovis.a"
  )

list(APPEND _cmake_import_check_targets vtkGeovis )
list(APPEND _cmake_import_check_files_for_vtkGeovis "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkGeovis.a" )

# Import target "vtkViews" for configuration "Release"
set_property(TARGET vtkViews APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkViews PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkInfovis"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkViews.a"
  )

list(APPEND _cmake_import_check_targets vtkViews )
list(APPEND _cmake_import_check_files_for_vtkViews "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkViews.a" )

# Import target "vtkCharts" for configuration "Release"
set_property(TARGET vtkCharts APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCharts PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkViews;vtkftgl"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkCharts.a"
  )

list(APPEND _cmake_import_check_targets vtkCharts )
list(APPEND _cmake_import_check_files_for_vtkCharts "${_IMPORT_PREFIX}/lib/vtk-5.10/libvtkCharts.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
