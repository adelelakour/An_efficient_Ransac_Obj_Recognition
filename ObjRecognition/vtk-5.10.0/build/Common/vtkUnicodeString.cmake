# Generate data for folding Unicode strings
SET(CASE_FOLD_DATA_FILE "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Common/vtkUnicodeCaseFoldData.h")
FILE(WRITE ${CASE_FOLD_DATA_FILE} "// Generated file, do not edit by hand!\n")
FILE(APPEND ${CASE_FOLD_DATA_FILE} "\n")
FILE(APPEND ${CASE_FOLD_DATA_FILE} "static vtkUnicodeString::value_type vtkUnicodeCaseFoldData[] = {\n")

# The following line relies on CMake 2.6, so for now we'll do it the old way
#FILE(STRINGS "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/CaseFolding.txt" FOLDING)
FILE(READ "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Common/CaseFolding.txt" FOLDING)
STRING(REGEX REPLACE ";" "\\\\;" FOLDING "${FOLDING}")
STRING(REGEX REPLACE "\n" ";" FOLDING "${FOLDING}")

FOREACH(FOLD ${FOLDING})
  IF(FOLD MATCHES "^([0-9A-F][0-9A-F][0-9A-F][0-9A-F][0-9A-F]?); (C|F); ([^;]*); # (.*)")
    # The following 3 lines work in CMake 2.6, but not in 2.4,
    # so again we do it the old way.
    #SET(CODE ${CMAKE_MATCH_1})
    #SET(MAPPING ${CMAKE_MATCH_3})
    #SET(COMMENT ${CMAKE_MATCH_4})
    STRING(REGEX REPLACE  "^([0-9A-F][0-9A-F][0-9A-F][0-9A-F][0-9A-F]?); (C|F); ([^;]*); # (.*)"
      "\\1" CODE "${FOLD}")
    STRING(REGEX REPLACE  "^([0-9A-F][0-9A-F][0-9A-F][0-9A-F][0-9A-F]?); (C|F); ([^;]*); # (.*)"
      "\\3" MAPPING "${FOLD}")
    STRING(REGEX REPLACE  "^([0-9A-F][0-9A-F][0-9A-F][0-9A-F][0-9A-F]?); (C|F); ([^;]*); # (.*)"
      "\\4" COMMENT "${FOLD}")

    SEPARATE_ARGUMENTS(MAPPING)

    FILE(APPEND ${CASE_FOLD_DATA_FILE} "  0x${CODE}, ")
    FOREACH(MAP ${MAPPING})
      FILE(APPEND ${CASE_FOLD_DATA_FILE} "0x${MAP}, ")
    ENDFOREACH(MAP ${MAPPING})
    FILE(APPEND ${CASE_FOLD_DATA_FILE} "0x0000, // ${COMMENT}\n")
  ENDIF(FOLD MATCHES "^([0-9A-F][0-9A-F][0-9A-F][0-9A-F][0-9A-F]?); (C|F); ([^;]*); # (.*)")
ENDFOREACH(FOLD ${FOLDING})

FILE(APPEND ${CASE_FOLD_DATA_FILE} "  0x0000 };\n\n")

