/*
 * Vtk3PlaneWidgets.cpp
 *
 *  Created on: Apr 1, 2010
 *      Author: papazov
 */

#include "Vtk3PlaneWidgets.h"
#include <vtkPointData.h>
#include <vtkLookupTable.h>

Vtk3PlaneWidgets::Vtk3PlaneWidgets()
{
	mImgPlaneX = vtkImagePlaneWidget::New();
	mImgPlaneY = vtkImagePlaneWidget::New();
	mImgPlaneZ = vtkImagePlaneWidget::New();
	mComputeRange = true;
}

Vtk3PlaneWidgets::~Vtk3PlaneWidgets()
{
	mImgPlaneX->Delete();
	mImgPlaneY->Delete();
	mImgPlaneZ->Delete();
}

//===========================================================================================================

void Vtk3PlaneWidgets::setInterpolationToCubic()
{
	mImgPlaneX->SetResliceInterpolateToCubic();
	mImgPlaneY->SetResliceInterpolateToCubic();
	mImgPlaneZ->SetResliceInterpolateToCubic();
}

//===========================================================================================================

void Vtk3PlaneWidgets::setInterpolationToLinear()
{
	mImgPlaneX->SetResliceInterpolateToLinear();
	mImgPlaneY->SetResliceInterpolateToLinear();
	mImgPlaneZ->SetResliceInterpolateToLinear();
}

//===========================================================================================================

void Vtk3PlaneWidgets::setInterpolationToNearest()
{
	mImgPlaneX->SetResliceInterpolateToNearestNeighbour();
	mImgPlaneY->SetResliceInterpolateToNearestNeighbour();
	mImgPlaneZ->SetResliceInterpolateToNearestNeighbour();
}

//===========================================================================================================

void Vtk3PlaneWidgets::switchOn(vtkImageData* imgData, vtkRenderWindowInteractor* interactor)
{
	double range[2];
	if ( mComputeRange )
	{
		// Compute the range of the values in imgData in order to use
		// the whole range of colors in vtkLookupTable
		imgData->GetPointData()->GetScalars()->GetRange(range);
		printf("'Vtk3ImagePlaneWidgets::%s()': range = [%f, %f]\n", __func__, range[0], range[1]);
	}
	else
	{
		// Use the range supplied by the user.
		range[0] = mValueRange[0];
		range[1] = mValueRange[1];
	}

	vtkLookupTable* lut = vtkLookupTable::New();
	  lut->SetAlphaRange(1.0, 1.0);
	  lut->SetValueRange(1.0, 1.0);
	  lut->SetSaturationRange(1.0, 1.0);
	  lut->SetHueRange(0.667, 0.0);
	  lut->SetNumberOfColors(256);
	  lut->SetRange(range);
	  lut->Build();

	mImgPlaneX->SetInput(imgData);
	mImgPlaneX->SetPlaneOrientationToXAxes();
	mImgPlaneX->SetInteractor(interactor);
	mImgPlaneX->SetLookupTable(lut);
	mImgPlaneX->On();

	mImgPlaneY->SetInput(imgData);
	mImgPlaneY->SetPlaneOrientationToYAxes();
	mImgPlaneY->SetInteractor(interactor);
	mImgPlaneY->SetLookupTable(lut);
	mImgPlaneY->On();

	mImgPlaneZ->SetInput(imgData);
	mImgPlaneZ->SetPlaneOrientationToZAxes();
	mImgPlaneZ->SetInteractor(interactor);
	mImgPlaneZ->SetLookupTable(lut);
	mImgPlaneZ->On();

	// Clean up
	lut->Delete();
}

//===========================================================================================================
