/*
 * VtkRMS.cpp
 *
 *  Created on: Apr 12, 2010
 *      Author: papazov
 */

#include "VtkRMS.h"
#include <cmath>


VtkRMS::VtkRMS()
{
	mMainPolyData = NULL;
	mPointLocator = NULL;
}

VtkRMS::VtkRMS(vtkPolyData* main_shape)
{
	mMainPolyData = NULL;
	mPointLocator = NULL;
	this->set_main(main_shape);
}

VtkRMS::~VtkRMS()
{
	if ( mPointLocator ) mPointLocator->Delete();
}

//=======================================================================================================================

void VtkRMS::set_main(vtkPolyData* main)
{
	mMainPolyData = main;

	if ( mPointLocator ) mPointLocator->Delete();
	mPointLocator = vtkPointLocator::New();
	mPointLocator->SetDataSet(mMainPolyData);
	mPointLocator->BuildLocator();
}

//=======================================================================================================================

#define RMS_SQR(x) ((x)*(x))

double VtkRMS::rms(vtkPoints* secondary)
{
	double sum = 0.0, p[3], x[3];
	int i, n = secondary->GetNumberOfPoints();

	for ( i = 0 ; i < n ; ++i )
	{
		secondary->GetPoint(i, x);
		mMainPolyData->GetPoint(mPointLocator->FindClosestPoint(x), p);
		sum += (RMS_SQR(x[0]-p[0]) + RMS_SQR(x[1]-p[1]) + RMS_SQR(x[2]-p[2]));
	}

	return sqrt(sum/(double)n);
}

//=======================================================================================================================
