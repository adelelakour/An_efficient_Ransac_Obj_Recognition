# Directory containing class headers.
SET(VTK_COMMON_HEADER_DIR "${VTK_INSTALL_PREFIX}/include/vtk-5.10")

# Classes in vtkCommon.
SET(VTK_COMMON_CLASSES
  "vtkAbstractArray"
  "vtkAbstractTransform"
  "vtkAmoebaMinimizer"
  "vtkAnimationCue"
  "vtkAnimationScene"
  "vtkArrayIterator"
  "vtkAssemblyNode"
  "vtkAssemblyPath"
  "vtkAssemblyPaths"
  "vtkBitArray"
  "vtkBitArrayIterator"
  "vtkBoundingBox"
  "vtkBox"
  "vtkBoxMuellerRandomSequence"
  "vtkBreakPoint"
  "vtkByteSwap"
  "vtkCallbackCommand"
  "vtkCharArray"
  "vtkClientSocket"
  "vtkCollection"
  "vtkCollectionIterator"
  "vtkColor"
  "vtkCommand"
  "vtkCommonInformationKeyManager"
  "vtkConditionVariable"
  "vtkContourValues"
  "vtkCriticalSection"
  "vtkCylindricalTransform"
  "vtkDataArray"
  "vtkDataArrayCollection"
  "vtkDataArrayCollectionIterator"
  "vtkDataArraySelection"
  "vtkDebugLeaks"
  "vtkDebugLeaksManager"
  "vtkDirectory"
  "vtkDoubleArray"
  "vtkDynamicLoader"
  "vtkEdgeTable"
  "vtkErrorCode"
  "vtkEventForwarderCommand"
  "vtkExtentSplitter"
  "vtkExtentTranslator"
  "vtkFastNumericConversion"
  "vtkFileOutputWindow"
  "vtkFloatArray"
  "vtkFloatingPointExceptions"
  "vtkFunctionParser"
  "vtkFunctionSet"
  "vtkGarbageCollector"
  "vtkGarbageCollectorManager"
  "vtkGaussianRandomSequence"
  "vtkGeneralTransform"
  "vtkHeap"
  "vtkHomogeneousTransform"
  "vtkIOStream"
  "vtkIdList"
  "vtkIdListCollection"
  "vtkIdTypeArray"
  "vtkIdentityTransform"
  "vtkImplicitFunction"
  "vtkImplicitFunctionCollection"
  "vtkIndent"
  "vtkInformation"
  "vtkInformationDataObjectKey"
  "vtkInformationDoubleKey"
  "vtkInformationDoubleVectorKey"
  "vtkInformationIdTypeKey"
  "vtkInformationInformationKey"
  "vtkInformationInformationVectorKey"
  "vtkInformationIntegerKey"
  "vtkInformationIntegerPointerKey"
  "vtkInformationIntegerVectorKey"
  "vtkInformationIterator"
  "vtkInformationKey"
  "vtkInformationKeyVectorKey"
  "vtkInformationObjectBaseKey"
  "vtkInformationObjectBaseVectorKey"
  "vtkInformationQuadratureSchemeDefinitionVectorKey"
  "vtkInformationRequestKey"
  "vtkInformationStringKey"
  "vtkInformationStringVectorKey"
  "vtkInformationUnsignedLongKey"
  "vtkInformationVector"
  "vtkInitialValueProblemSolver"
  "vtkInstantiator"
  "vtkIntArray"
  "vtkLargeInteger"
  "vtkLinearTransform"
  "vtkLogLookupTable"
  "vtkLongArray"
  "vtkLookupTable"
  "vtkLookupTableWithEnabling"
  "vtkMath"
  "vtkMathUtilities"
  "vtkMatrix3x3"
  "vtkMatrix4x4"
  "vtkMatrixToHomogeneousTransform"
  "vtkMatrixToLinearTransform"
  "vtkMinimalStandardRandomSequence"
  "vtkMultiThreader"
  "vtkMutexLock"
  "vtkOStrStreamWrapper"
  "vtkOStreamWrapper"
  "vtkObject"
  "vtkObjectBase"
  "vtkObjectFactory"
  "vtkObjectFactoryCollection"
  "vtkOldStyleCallbackCommand"
  "vtkOnePieceExtentTranslator"
  "vtkOutputWindow"
  "vtkOverrideInformation"
  "vtkOverrideInformationCollection"
  "vtkParametricBoy"
  "vtkParametricConicSpiral"
  "vtkParametricCrossCap"
  "vtkParametricDini"
  "vtkParametricEllipsoid"
  "vtkParametricEnneper"
  "vtkParametricFigure8Klein"
  "vtkParametricFunction"
  "vtkParametricKlein"
  "vtkParametricMobius"
  "vtkParametricRandomHills"
  "vtkParametricRoman"
  "vtkParametricSuperEllipsoid"
  "vtkParametricSuperToroid"
  "vtkParametricTorus"
  "vtkPerspectiveTransform"
  "vtkPlane"
  "vtkPlaneCollection"
  "vtkPlanes"
  "vtkPoints"
  "vtkPoints2D"
  "vtkPolynomialSolversUnivariate"
  "vtkPriorityQueue"
  "vtkProp"
  "vtkPropCollection"
  "vtkProperty2D"
  "vtkQuadratureSchemeDefinition"
  "vtkQuadric"
  "vtkRandomSequence"
  "vtkRect"
  "vtkReferenceCount"
  "vtkRungeKutta2"
  "vtkRungeKutta4"
  "vtkRungeKutta45"
  "vtkScalarsToColors"
  "vtkServerSocket"
  "vtkShortArray"
  "vtkSignedCharArray"
  "vtkSmartPointerBase"
  "vtkSocket"
  "vtkSocketCollection"
  "vtkSortDataArray"
  "vtkSphericalTransform"
  "vtkStdString"
  "vtkStringArray"
  "vtkStructuredData"
  "vtkStructuredExtent"
  "vtkStructuredVisibilityConstraint"
  "vtkTableExtentTranslator"
  "vtkTensor"
  "vtkThreadMessager"
  "vtkTimePointUtility"
  "vtkTimeStamp"
  "vtkTimerLog"
  "vtkTransform2D"
  "vtkTransform"
  "vtkTransformCollection"
  "vtkUnicodeString"
  "vtkUnicodeStringArray"
  "vtkUnsignedCharArray"
  "vtkUnsignedIntArray"
  "vtkUnsignedLongArray"
  "vtkUnsignedShortArray"
  "vtkVariant"
  "vtkVariantArray"
  "vtkVector"
  "vtkTuple"
  "vtkVersion"
  "vtkVoidArray"
  "vtkWarpTransform"
  "vtkWeakPointerBase"
  "vtkWindow"
  "vtkWindowLevelLookupTable"
  "vtkXMLDataElement"
  "vtkXMLFileOutputWindow"
  "vtkLongLongArray"
  "vtkUnsignedLongLongArray"
  "vtkTypeInt8Array"
  "vtkTypeInt16Array"
  "vtkTypeInt32Array"
  "vtkTypeInt64Array"
  "vtkTypeUInt8Array"
  "vtkTypeUInt16Array"
  "vtkTypeUInt32Array"
  "vtkTypeUInt64Array"
  "vtkTypeFloat32Array"
  "vtkTypeFloat64Array"
  "vtkArray"
  "vtkArrayCoordinates"
  "vtkArrayExtents"
  "vtkArrayExtentsList"
  "vtkArrayRange"
  "vtkArraySort"
  "vtkArrayWeights"
  "vtkDenseArray"
  "vtkSparseArray"
  "vtkTypedArray"
  "vtkTypeTemplate")

# Abstract classes in vtkCommon.
SET(VTK_COMMON_CLASSES_ABSTRACT
  "vtkAbstractArray"
  "vtkAbstractTransform"
  "vtkArrayIterator"
  "vtkCallbackCommand"
  "vtkCommand"
  "vtkCommonInformationKeyManager"
  "vtkDataArray"
  "vtkEventForwarderCommand"
  "vtkFloatingPointExceptions"
  "vtkFunctionSet"
  "vtkGaussianRandomSequence"
  "vtkHomogeneousTransform"
  "vtkImplicitFunction"
  "vtkInformationDataObjectKey"
  "vtkInformationDoubleKey"
  "vtkInformationDoubleVectorKey"
  "vtkInformationIdTypeKey"
  "vtkInformationInformationKey"
  "vtkInformationInformationVectorKey"
  "vtkInformationIntegerKey"
  "vtkInformationIntegerPointerKey"
  "vtkInformationIntegerVectorKey"
  "vtkInformationKey"
  "vtkInformationKeyVectorKey"
  "vtkInformationObjectBaseKey"
  "vtkInformationObjectBaseVectorKey"
  "vtkInformationQuadratureSchemeDefinitionVectorKey"
  "vtkInformationRequestKey"
  "vtkInformationStringKey"
  "vtkInformationStringVectorKey"
  "vtkInformationUnsignedLongKey"
  "vtkInitialValueProblemSolver"
  "vtkLinearTransform"
  "vtkObjectBase"
  "vtkObjectFactory"
  "vtkOldStyleCallbackCommand"
  "vtkOverrideInformation"
  "vtkOverrideInformationCollection"
  "vtkParametricFunction"
  "vtkProp"
  "vtkRandomSequence"
  "vtkScalarsToColors"
  "vtkSocket"
  "vtkStructuredData"
  "vtkWarpTransform"
  "vtkWindow"
  "vtkArray")

# Wrap-exclude classes in vtkCommon.
SET(VTK_COMMON_CLASSES_WRAP_EXCLUDE
  "vtkBoundingBox"
  "vtkBreakPoint"
  "vtkCallbackCommand"
  "vtkColor"
  "vtkCommonInformationKeyManager"
  "vtkDebugLeaksManager"
  "vtkErrorCode"
  "vtkEventForwarderCommand"
  "vtkFloatingPointExceptions"
  "vtkGarbageCollectorManager"
  "vtkIOStream"
  "vtkIndent"
  "vtkLargeInteger"
  "vtkMathUtilities"
  "vtkOStrStreamWrapper"
  "vtkOStreamWrapper"
  "vtkOldStyleCallbackCommand"
  "vtkRect"
  "vtkSmartPointerBase"
  "vtkStdString"
  "vtkTimeStamp"
  "vtkUnicodeString"
  "vtkVariant"
  "vtkVector"
  "vtkTuple"
  "vtkWeakPointerBase"
  "vtkArrayCoordinates"
  "vtkArrayExtents"
  "vtkArrayExtentsList"
  "vtkArrayRange"
  "vtkArraySort"
  "vtkArrayWeights"
  "vtkDenseArray"
  "vtkSparseArray"
  "vtkTypedArray"
  "vtkTypeTemplate")

# Wrap-special classes in vtkCommon.
SET(VTK_COMMON_CLASSES_WRAP_SPECIAL
  "vtkColor"
  "vtkRect"
  "vtkTimeStamp"
  "vtkVariant"
  "vtkVector"
  "vtkTuple"
  "vtkArrayCoordinates"
  "vtkArrayExtents"
  "vtkArrayExtentsList"
  "vtkArrayRange"
  "vtkArraySort"
  "vtkArrayWeights"
  "vtkDenseArray"
  "vtkSparseArray"
  "vtkTypedArray"
  "vtkTypeTemplate")

# Wrappable non-class headers for vtkCommon.
SET(VTK_COMMON_WRAP_HEADERS
  "vtkSystemIncludes.h"
  "vtkType.h"
  "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/Common/vtkMathConfigure.h"
  "/home/adelelakour/CLionProjects/An_effi_Ransac_RealSense-master/vtk-5.10.0/build/vtkConfigure.h")

# Set convenient variables to test each class.
FOREACH(class ${VTK_COMMON_CLASSES})
  SET(VTK_CLASS_EXISTS_${class} 1)
ENDFOREACH(class)
FOREACH(class ${VTK_COMMON_CLASSES_ABSTRACT})
  SET(VTK_CLASS_ABSTRACT_${class} 1)
ENDFOREACH(class)
FOREACH(class ${VTK_COMMON_CLASSES_WRAP_EXCLUDE})
  SET(VTK_CLASS_WRAP_EXCLUDE_${class} 1)
ENDFOREACH(class)
FOREACH(class ${VTK_COMMON_CLASSES_WRAP_SPECIAL})
  SET(VTK_CLASS_WRAP_SPECIAL_${class} 1)
ENDFOREACH(class)
