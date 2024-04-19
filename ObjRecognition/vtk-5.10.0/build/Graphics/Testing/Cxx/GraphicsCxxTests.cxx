#include <ctype.h>  /* NOLINT */
#include <stdio.h>  /* NOLINT */
#include <stdlib.h> /* NOLINT */
#include <string.h> /* NOLINT */
#include <time.h>   /* NOLINT */

#if defined(_MSC_VER)
#pragma warning(disable : 4996) /* deprecation */
#endif

#include "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/Rendering/vtkTestingObjectFactory.h"


/* Forward declare test functions. */
int TestCenterOfMass(int, char*[]);
int Mace(int, char*[]);
int expCos(int, char*[]);
int BoxClipTriangulate(int, char*[]);
int CellLocator(int, char*[]);
int PointLocator(int, char*[]);
int FrustumClip(int, char*[]);
int RGrid(int, char*[]);
int TestAppendSelection(int, char*[]);
int TestAssignAttribute(int, char*[]);
int TestBareScalarsToColors(int, char*[]);
int TestBSPTree(int, char*[]);
int TestCellDataToPointData(int, char*[]);
int TestDensifyPolyData(int, char*[]);
int TestClipHyperOctree(int, char*[]);
int TestConvertSelection(int, char*[]);
int TestDelaunay2D(int, char*[]);
int TestGlyph3D(int, char*[]);
int TestExtraction(int, char*[]);
int TestExtractSelection(int, char*[]);
int TestExtractSurfaceNonLinearSubdivision(int, char*[]);
int TestHyperOctreeContourFilter(int, char*[]);
int TestHyperOctreeCutter(int, char*[]);
int TestHyperOctreeDual(int, char*[]);
int TestHyperOctreeSurfaceFilter(int, char*[]);
int TestHyperOctreeToUniformGrid(int, char*[]);
int TestImageDataToPointSet(int, char*[]);
int TestLineSource(int, char*[]);
int TestMapVectorsAsRGBColors(int, char*[]);
int TestMapVectorsToColors(int, char*[]);
int TestNamedComponents(int, char*[]);
int TestMeanValueCoordinatesInterpolation1(int, char*[]);
int TestMeanValueCoordinatesInterpolation2(int, char*[]);
int TestPolyDataPointSampler(int, char*[]);
int TestPolyhedron0(int, char*[]);
int TestPolyhedron1(int, char*[]);
int TestQuadRotationalExtrusion(int, char*[]);
int TestRectilinearGridToPointSet(int, char*[]);
int TestReflectionFilter(int, char*[]);
int TestRotationalExtrusion(int, char*[]);
int TestSelectEnclosedPoints(int, char*[]);
int TestTessellatedBoxSource(int, char*[]);
int TestTessellator(int, char*[]);
int TestUncertaintyTubeFilter(int, char*[]);
int TestDecimatePolylineFilter(int, char*[]);
int TestImplicitPolyDataDistance(int, char*[]);
int BoxClipPolyData(int, char*[]);
int BoxClipTetrahedra(int, char*[]);
int BoxClipTriangulateAndInterpolate(int, char*[]);
int MeshQuality(int, char*[]);
int TestActor2DTextures(int, char*[]);
int TestArrayCalculator(int, char*[]);
int TestGradientAndVorticity(int, char*[]);
int TestHierarchicalBoxPipeline(int, char*[]);
int TestIconGlyphFilterGravity(int, char*[]);
int TestIncrementalOctreePointLocator(int, char*[]);
int TestMultiBlock(int, char*[]);
int TestQuadraturePoints(int, char*[]);
int TestQuadRotationalExtrusionMultiBlock(int, char*[]);
int TestUnstructuredGridGeometryFilter(int, char*[]);
int UnstructuredGridGradients(int, char*[]);
int UnstructuredGridCellGradients(int, char*[]);
int UnstructuredGridFastGradients(int, char*[]);
int TestBooleanOperationPolyDataFilter(int, char*[]);
int TestBooleanOperationPolyDataFilter2(int, char*[]);
int TestDistancePolyDataFilter(int, char*[]);
int TestIntersectionPolyDataFilter(int, char*[]);
int TestYoungsMaterialInterface(int, char*[]);
int TemporalStatistics(int, char*[]);


#ifdef __cplusplus
#  define CM_CAST(TYPE, EXPR) static_cast<TYPE>(EXPR)
#  if __cplusplus >= 201103L
#    define CM_NULL nullptr
#  else
#    define CM_NULL NULL
#  endif
#  define CM_NAMESPACE_BEGIN namespace {
#  define CM_NAMESPACE_END }
#  define CM_LOCAL
#else
#  define CM_CAST(TYPE, EXPR) (TYPE)(EXPR)
#  define CM_NULL NULL
#  define CM_NAMESPACE_BEGIN
#  define CM_NAMESPACE_END
#  define CM_LOCAL static
#endif

CM_NAMESPACE_BEGIN

/* Create map.  */

typedef int (*MainFuncPointer)(int, char* []); /* NOLINT */
typedef struct /* NOLINT */
{
  const char* name;
  MainFuncPointer func;
} functionMapEntry;

CM_LOCAL const functionMapEntry cmakeGeneratedFunctionMapEntries[] = {
    {
    "TestCenterOfMass",
    TestCenterOfMass
  },
  {
    "Mace",
    Mace
  },
  {
    "expCos",
    expCos
  },
  {
    "BoxClipTriangulate",
    BoxClipTriangulate
  },
  {
    "CellLocator",
    CellLocator
  },
  {
    "PointLocator",
    PointLocator
  },
  {
    "FrustumClip",
    FrustumClip
  },
  {
    "RGrid",
    RGrid
  },
  {
    "TestAppendSelection",
    TestAppendSelection
  },
  {
    "TestAssignAttribute",
    TestAssignAttribute
  },
  {
    "TestBareScalarsToColors",
    TestBareScalarsToColors
  },
  {
    "TestBSPTree",
    TestBSPTree
  },
  {
    "TestCellDataToPointData",
    TestCellDataToPointData
  },
  {
    "TestDensifyPolyData",
    TestDensifyPolyData
  },
  {
    "TestClipHyperOctree",
    TestClipHyperOctree
  },
  {
    "TestConvertSelection",
    TestConvertSelection
  },
  {
    "TestDelaunay2D",
    TestDelaunay2D
  },
  {
    "TestGlyph3D",
    TestGlyph3D
  },
  {
    "TestExtraction",
    TestExtraction
  },
  {
    "TestExtractSelection",
    TestExtractSelection
  },
  {
    "TestExtractSurfaceNonLinearSubdivision",
    TestExtractSurfaceNonLinearSubdivision
  },
  {
    "TestHyperOctreeContourFilter",
    TestHyperOctreeContourFilter
  },
  {
    "TestHyperOctreeCutter",
    TestHyperOctreeCutter
  },
  {
    "TestHyperOctreeDual",
    TestHyperOctreeDual
  },
  {
    "TestHyperOctreeSurfaceFilter",
    TestHyperOctreeSurfaceFilter
  },
  {
    "TestHyperOctreeToUniformGrid",
    TestHyperOctreeToUniformGrid
  },
  {
    "TestImageDataToPointSet",
    TestImageDataToPointSet
  },
  {
    "TestLineSource",
    TestLineSource
  },
  {
    "TestMapVectorsAsRGBColors",
    TestMapVectorsAsRGBColors
  },
  {
    "TestMapVectorsToColors",
    TestMapVectorsToColors
  },
  {
    "TestNamedComponents",
    TestNamedComponents
  },
  {
    "TestMeanValueCoordinatesInterpolation1",
    TestMeanValueCoordinatesInterpolation1
  },
  {
    "TestMeanValueCoordinatesInterpolation2",
    TestMeanValueCoordinatesInterpolation2
  },
  {
    "TestPolyDataPointSampler",
    TestPolyDataPointSampler
  },
  {
    "TestPolyhedron0",
    TestPolyhedron0
  },
  {
    "TestPolyhedron1",
    TestPolyhedron1
  },
  {
    "TestQuadRotationalExtrusion",
    TestQuadRotationalExtrusion
  },
  {
    "TestRectilinearGridToPointSet",
    TestRectilinearGridToPointSet
  },
  {
    "TestReflectionFilter",
    TestReflectionFilter
  },
  {
    "TestRotationalExtrusion",
    TestRotationalExtrusion
  },
  {
    "TestSelectEnclosedPoints",
    TestSelectEnclosedPoints
  },
  {
    "TestTessellatedBoxSource",
    TestTessellatedBoxSource
  },
  {
    "TestTessellator",
    TestTessellator
  },
  {
    "TestUncertaintyTubeFilter",
    TestUncertaintyTubeFilter
  },
  {
    "TestDecimatePolylineFilter",
    TestDecimatePolylineFilter
  },
  {
    "TestImplicitPolyDataDistance",
    TestImplicitPolyDataDistance
  },
  {
    "BoxClipPolyData",
    BoxClipPolyData
  },
  {
    "BoxClipTetrahedra",
    BoxClipTetrahedra
  },
  {
    "BoxClipTriangulateAndInterpolate",
    BoxClipTriangulateAndInterpolate
  },
  {
    "MeshQuality",
    MeshQuality
  },
  {
    "TestActor2DTextures",
    TestActor2DTextures
  },
  {
    "TestArrayCalculator",
    TestArrayCalculator
  },
  {
    "TestGradientAndVorticity",
    TestGradientAndVorticity
  },
  {
    "TestHierarchicalBoxPipeline",
    TestHierarchicalBoxPipeline
  },
  {
    "TestIconGlyphFilterGravity",
    TestIconGlyphFilterGravity
  },
  {
    "TestIncrementalOctreePointLocator",
    TestIncrementalOctreePointLocator
  },
  {
    "TestMultiBlock",
    TestMultiBlock
  },
  {
    "TestQuadraturePoints",
    TestQuadraturePoints
  },
  {
    "TestQuadRotationalExtrusionMultiBlock",
    TestQuadRotationalExtrusionMultiBlock
  },
  {
    "TestUnstructuredGridGeometryFilter",
    TestUnstructuredGridGeometryFilter
  },
  {
    "UnstructuredGridGradients",
    UnstructuredGridGradients
  },
  {
    "UnstructuredGridCellGradients",
    UnstructuredGridCellGradients
  },
  {
    "UnstructuredGridFastGradients",
    UnstructuredGridFastGradients
  },
  {
    "TestBooleanOperationPolyDataFilter",
    TestBooleanOperationPolyDataFilter
  },
  {
    "TestBooleanOperationPolyDataFilter2",
    TestBooleanOperationPolyDataFilter2
  },
  {
    "TestDistancePolyDataFilter",
    TestDistancePolyDataFilter
  },
  {
    "TestIntersectionPolyDataFilter",
    TestIntersectionPolyDataFilter
  },
  {
    "TestYoungsMaterialInterface",
    TestYoungsMaterialInterface
  },
  {
    "TemporalStatistics",
    TemporalStatistics
  },

  { CM_NULL, CM_NULL } /* NOLINT */
};

CM_LOCAL const int NumTests = CM_CAST(int,
  sizeof(cmakeGeneratedFunctionMapEntries) / sizeof(functionMapEntry)) - 1;

/* Allocate and create a lowercased copy of string
   (note that it has to be free'd manually) */
CM_LOCAL char* lowercase(const char* string)
{
  char *new_string;
  char *p;
  size_t stringSize;

  stringSize = CM_CAST(size_t, strlen(string) + 1);
  new_string = CM_CAST(char*, malloc(sizeof(char) * stringSize));

  if (new_string == CM_NULL) { /* NOLINT */
    return CM_NULL;            /* NOLINT */
  }
  strcpy(new_string, string);  /* NOLINT */
  for (p = new_string; *p != 0; ++p) {
    *p = CM_CAST(char, tolower(*p));
  }
  return new_string;
}

CM_LOCAL int isTestSkipped(const char *name, int n_skipped_tests, char *skipped_tests[]) {
  int i;
  for (i = 0; i < n_skipped_tests; i++) {
    if (strcmp(name, skipped_tests[i]) == 0) {
      return 1;
    }
  }

  return 0;
}

CM_NAMESPACE_END

int main(int ac, char* av[])
{
  int i;
  int testNum = 0;
  int partial_match;
  int run_all;
  char *arg;
  int testToRun = -1;

  

  /* If no test name was given */
  /* process command line with user function.  */
  if (ac < 2) {
    /* Ask for a test.  */
    printf("Available tests:\n");
    for (i = 0; i < NumTests; ++i) {
      printf("%3d. %s\n", i, cmakeGeneratedFunctionMapEntries[i].name);
    }
    printf("To run a test, enter the test number: ");
    fflush(stdout);
    if (scanf("%d", &testNum) != 1) {
      printf("Couldn't parse that input as a number\n");
      return -1;
    }
    if (testNum >= NumTests) {
      printf("%3d is an invalid test number.\n", testNum);
      return -1;
    }
    testToRun = testNum;
    ac--;
    av++;
  }
  partial_match = 0;
  run_all = 0;
  arg = CM_NULL; /* NOLINT */
  /* If partial match or running all tests are requested.  */
  if (testToRun == -1 && ac > 1) {
    partial_match = (strcmp(av[1], "-R") == 0) ? 1 : 0;
    run_all = (strcmp(av[1], "-A") == 0) ? 1 : 0;
  }
  if (partial_match != 0 && ac < 3) {
    printf("-R needs an additional parameter.\n");
    return -1;
  }
  if (run_all == 1) {
    clock_t t;
    int status = 0;
    const char* status_message = CM_NULL;
    printf("TAP version 13\n");
    printf("1..%d\n", NumTests);
    for (i = 0; i < NumTests; ++i) {
      const char *name = cmakeGeneratedFunctionMapEntries[i].name;
      if (ac > 2) {
        if (isTestSkipped(name, ac - 2, av + 2) == 1) {
          printf("ok %d %s # SKIP\n", i + 1, name);
          continue;
        }
      }
      t = clock();
      status = (*cmakeGeneratedFunctionMapEntries[i].func)(ac, av);
      t = clock() - t;
      status_message = (status == -1) ? "not ok" : "ok";
      {
        double time_taken = CM_CAST(double, t) / CLOCKS_PER_SEC;
        printf("%s %d %s # %f\n", status_message, i + 1, name, time_taken);
      }
    }
    printf("All tests finished.\n");

    return 0;
  }

  if (testToRun == -1) {
    arg = lowercase(av[1 + partial_match]);
  }
  for (i = 0; i < NumTests && testToRun == -1; ++i) {
    char *test_name = lowercase(cmakeGeneratedFunctionMapEntries[i].name);
    if (partial_match != 0 && strstr(test_name, arg) != CM_NULL) { /* NOLINT */
      testToRun = i;
      ac -= 2;
      av += 2;
    } else if (partial_match == 0 && strcmp(test_name, arg) == 0) {
      testToRun = i;
      ac--;
      av++;
    }
    free(test_name);
  }
  free(arg);
  if (testToRun != -1) {
    int result;

    // Set defaults
    vtkTestingInteractor::ValidBaseline =
      std::string("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/VTKData") +
      std::string("/Baseline/") +
      std::string("Graphics/") +
      std::string(cmakeGeneratedFunctionMapEntries[testToRun].name) +
      std::string(".png");
    vtkTestingInteractor::TempDirectory =
      std::string("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Testing/Temporary");
    vtkTestingInteractor::DataDirectory =
      std::string("/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/VTKData");

    int interactive = 0;
    for (int ii = 0; ii < ac; ++ii)
      {
      if ( strcmp(av[ii],"-I") == 0)
        {
        interactive = 1;
        continue;
        }
      if ( strcmp(av[ii],"-V") == 0 && ii < ac-1)
        {
        vtkTestingInteractor::ValidBaseline = std::string(av[ii+1]);
        ++ii;
        continue;
        }
      if ( strcmp(av[ii],"-T") == 0 && ii < ac-1)
        {
        vtkTestingInteractor::TempDirectory = std::string(av[ii+1]);
        ++ii;
        continue;
        }
      if ( strcmp(av[ii],"-D") == 0 && ii < ac-1)
        {
        vtkTestingInteractor::DataDirectory = std::string(av[ii+1]);
        ++ii;
        continue;
        }
      if ( strcmp(av[ii],"-E") == 0 && ii < ac-1)
        {
        vtkTestingInteractor::ErrorThreshold =
          static_cast<double>(atof(av[ii+1]));
        ++ii;
        continue;
        }
      }
    vtkSmartPointer<vtkTestingObjectFactory> factory = vtkSmartPointer<vtkTestingObjectFactory>::New();
    if (!interactive)
      {
      vtkObjectFactory::RegisterFactory(factory);
      }

    if (testToRun < 0 || testToRun >= NumTests) {
      printf("testToRun was modified by TestDriver code to an invalid value: "
             "%3d.\n",
             testNum);
      return -1;
    }
    result = (*cmakeGeneratedFunctionMapEntries[testToRun].func)(ac, av);
    
   if (!interactive)
     {
     if (vtkTestingInteractor::TestReturnStatus != -1)
        {
        if( vtkTestingInteractor::TestReturnStatus != vtkTesting::PASSED)
          {
          result = EXIT_FAILURE;
          }
        else
          {
          result = EXIT_SUCCESS;
          }
        }
      vtkObjectFactory::UnRegisterFactory(factory);
      }

    return result;
  }

  /* Nothing was run, display the test names.  */
  printf("Available tests:\n");
  for (i = 0; i < NumTests; ++i) {
    printf("%3d. %s\n", i, cmakeGeneratedFunctionMapEntries[i].name);
  }
  printf("Failed: %s is an invalid test name.\n", av[1]);

  return -1;
}
