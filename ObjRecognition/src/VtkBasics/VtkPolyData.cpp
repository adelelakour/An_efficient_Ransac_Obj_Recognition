#include "VtkPolyData.h"
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkColorTransferFunction.h>


VtkPolyData::VtkPolyData(vtkPolyData *polyData)
{
	if ( polyData )
	{
		mPolyData = polyData;
		mHasOwnPolyData = false;
	}
	else
	{
		mPolyData = vtkPolyData::New();
		mHasOwnPolyData = true;
	}

	mMapper = vtkPolyDataMapper::New();
	mMapper->SetInput(mPolyData);

	mActor = vtkActor::New();
	mActor->SetMapper(mMapper);

	this->setInterpolationToFlat();

	sprintf(mName, "VtkPolyData-object");
	mBounds = NULL;
}

VtkPolyData::~VtkPolyData()
{
	if ( mBounds )
		delete[] mBounds;
	if ( mHasOwnPolyData )
		mPolyData->Delete();
	mActor->Delete();
	mMapper->Delete();
}

//=====================================================================================================================

void VtkPolyData::update(vtkPolyData* poly_data)
{
	if ( mHasOwnPolyData )
	{
		mPolyData->Delete();
		mHasOwnPolyData = false;
	}

	mPolyData = poly_data;
	mMapper->SetInput(mPolyData);
	mMapper->Update();
}

//=====================================================================================================================

void VtkPolyData::normScalars()
{
	vtkDoubleArray* newScalars = vtkDoubleArray::New();
	  newScalars->SetNumberOfTuples(1);
	vtkDataArray* oldScalars = mPolyData->GetPointData()->GetScalars();

	double range[2]; oldScalars->GetRange(range);

	printf("range = [%lf, %lf]\n", range[0], range[1]); fflush(stdout);

	for ( int i = 0 ; i < oldScalars->GetNumberOfTuples() ; ++i )
		newScalars->InsertNextTuple1((oldScalars->GetTuple1(i) - range[0])/(range[1]-range[0]));

	mPolyData->GetPointData()->SetScalars(newScalars);
	// Clean up
	newScalars->Delete();
}

//=====================================================================================================================

void VtkPolyData::heatMap(double min, double max, bool print_range)
{
	vtkDataArray* scalars = mPolyData->GetPointData()->GetScalars();
	if ( !scalars )
		return;

	// Get the range of the scalars
	double range[2];
	if ( min > max )
		scalars->GetRange(range);
	else
	{
		range[0] = min;
		range[1] = max;
	}

	if ( print_range )
		printf("VtkPolyData::%s(): scalar range = [%lf, %lf]\n", __func__, range[0], range[1]);

	// Build the look-up table
	vtkLookupTable* lut = vtkLookupTable::New();
	  lut->SetHueRange(0.6, 0.0);
	  lut->SetRange(range[0], range[1]);
	  lut->Build();

	// Save it
	this->setLookupTable(lut);
	// Cleanup
	lut->Delete();
}

//=====================================================================================================================

void VtkPolyData::scalarsToHue(list<pair<double,double> >& scalar_hue_pairs)
{
	vtkColorTransferFunction* transfer_func = vtkColorTransferFunction::New();
	  transfer_func->SetColorSpaceToHSV();
	  transfer_func->ClampingOn();

	for ( list<pair<double,double> >::iterator it = scalar_hue_pairs.begin() ; it != scalar_hue_pairs.end() ; ++it )
		transfer_func->AddHSVPoint(it->first, it->second, 1.0, 1.0);

	this->setLookupTable(transfer_func);

	// Cleanup
	transfer_func->Delete();
}

//=====================================================================================================================
