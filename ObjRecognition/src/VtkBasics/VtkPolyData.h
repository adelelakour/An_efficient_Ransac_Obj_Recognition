#ifndef VTKPOLYDATA_H_
#define VTKPOLYDATA_H_

#include "VtkObject.h"
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkLookupTable.h>
#include <vtkScalarsToColors.h>
#include <cstring>
#include <list>

using namespace std;

class VtkPolyData: public VtkObject
{
public:
	VtkPolyData(vtkPolyData *polyData = NULL);
	virtual ~VtkPolyData();

	vtkPolyData* getPolyData(){ return mPolyData;}
	vtkPoints* getPoints(){ return mPolyData->GetPoints();}
	vtkMapper* getMapper(){ return mMapper;}

	void setInterpolationToFlat(){ mActor->GetProperty()->SetInterpolationToFlat();}
	void setInterpolationToPhong(){ mActor->GetProperty()->SetInterpolationToPhong();}

	void wireframe(){ mActor->GetProperty()->SetRepresentationToWireframe();}
	void update(vtkPolyData* poly_data);
	void update(){ mPolyData->Modified();}

	void setLineWidth(float value){ mActor->GetProperty()->SetLineWidth(value);}

	void heatMap(double min = 1.0, double max = -1.0, bool print_range = false);
	void scalarsToHue(list<pair<double,double> >& scalar_hue_pairs);
	inline void setLookupTable(vtkScalarsToColors* lut);

	void normScalars();

protected:
	vtkPolyDataMapper *mMapper;
	vtkPolyData *mPolyData;
	bool mHasOwnPolyData;
	double *mBounds;
};

//=== inline methods =======================================================================================

void VtkPolyData::setLookupTable(vtkScalarsToColors* lut)
{
	mMapper->SetLookupTable(lut);
	mMapper->SetColorModeToMapScalars();
	mMapper->SetScalarRange(lut->GetRange());
	this->setInterpolationToPhong();
	mMapper->Update();
}

//==========================================================================================================

#endif /*VTKPOLYDATA_H_*/
