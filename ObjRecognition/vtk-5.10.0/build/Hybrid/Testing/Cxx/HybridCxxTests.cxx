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
int TestBSplineWarp(int, char*[]);
int TestImageStencilData(int, char*[]);
int X3DTest(int, char*[]);
int TestCubeAxes2DMode(int, char*[]);
int TestCubeAxes3(int, char*[]);
int TestCubeAxesWithGridLines(int, char*[]);
int TestCubeAxesWithXLines(int, char*[]);
int TestCubeAxesWithYLines(int, char*[]);
int TestCubeAxesWithZLines(int, char*[]);
int TestCubeAxesWithXInnerGrids(int, char*[]);
int TestCubeAxesWithYInnerGrids(int, char*[]);
int TestCubeAxesWithZInnerGrids(int, char*[]);
int TestCubeAxesWithXInnerPolys(int, char*[]);
int TestCubeAxesWithYInnerPolys(int, char*[]);
int TestCubeAxesWithZInnerPolys(int, char*[]);
int TestBarChartActor(int, char*[]);
int TestExodusImplicitArrays(int, char*[]);
int TestLegendScaleActor(int, char*[]);
int TestPolarAxes(int, char*[]);
int TestPolyDataSilhouette(int, char*[]);
int TestPieChartActor(int, char*[]);
int TestSpiderPlotActor(int, char*[]);
int TestVRMLNormals(int, char*[]);
int TestTDxGeo(int, char*[]);
int TestLegendBoxActor(int, char*[]);
int TestImageDataLIC2D(int, char*[]);
int TestStructuredGridLIC2DXSlice(int, char*[]);
int TestStructuredGridLIC2DYSlice(int, char*[]);
int TestStructuredGridLIC2DZSlice(int, char*[]);


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
    "TestBSplineWarp",
    TestBSplineWarp
  },
  {
    "TestImageStencilData",
    TestImageStencilData
  },
  {
    "X3DTest",
    X3DTest
  },
  {
    "TestCubeAxes2DMode",
    TestCubeAxes2DMode
  },
  {
    "TestCubeAxes3",
    TestCubeAxes3
  },
  {
    "TestCubeAxesWithGridLines",
    TestCubeAxesWithGridLines
  },
  {
    "TestCubeAxesWithXLines",
    TestCubeAxesWithXLines
  },
  {
    "TestCubeAxesWithYLines",
    TestCubeAxesWithYLines
  },
  {
    "TestCubeAxesWithZLines",
    TestCubeAxesWithZLines
  },
  {
    "TestCubeAxesWithXInnerGrids",
    TestCubeAxesWithXInnerGrids
  },
  {
    "TestCubeAxesWithYInnerGrids",
    TestCubeAxesWithYInnerGrids
  },
  {
    "TestCubeAxesWithZInnerGrids",
    TestCubeAxesWithZInnerGrids
  },
  {
    "TestCubeAxesWithXInnerPolys",
    TestCubeAxesWithXInnerPolys
  },
  {
    "TestCubeAxesWithYInnerPolys",
    TestCubeAxesWithYInnerPolys
  },
  {
    "TestCubeAxesWithZInnerPolys",
    TestCubeAxesWithZInnerPolys
  },
  {
    "TestBarChartActor",
    TestBarChartActor
  },
  {
    "TestExodusImplicitArrays",
    TestExodusImplicitArrays
  },
  {
    "TestLegendScaleActor",
    TestLegendScaleActor
  },
  {
    "TestPolarAxes",
    TestPolarAxes
  },
  {
    "TestPolyDataSilhouette",
    TestPolyDataSilhouette
  },
  {
    "TestPieChartActor",
    TestPieChartActor
  },
  {
    "TestSpiderPlotActor",
    TestSpiderPlotActor
  },
  {
    "TestVRMLNormals",
    TestVRMLNormals
  },
  {
    "TestTDxGeo",
    TestTDxGeo
  },
  {
    "TestLegendBoxActor",
    TestLegendBoxActor
  },
  {
    "TestImageDataLIC2D",
    TestImageDataLIC2D
  },
  {
    "TestStructuredGridLIC2DXSlice",
    TestStructuredGridLIC2DXSlice
  },
  {
    "TestStructuredGridLIC2DYSlice",
    TestStructuredGridLIC2DYSlice
  },
  {
    "TestStructuredGridLIC2DZSlice",
    TestStructuredGridLIC2DZSlice
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
      std::string("Hybrid/") +
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
