/*
 * Vtk3PlaneWidgets.h
 *
 *  Created on: Apr 1, 2010
 *      Author: papazov
 */

#ifndef VTK3PLANEWIDGETS_H_
#define VTK3PLANEWIDGETS_H_

#include <vtkImageData.h>
#include <vtkImagePlaneWidget.h>
#include <vtkRenderWindowInteractor.h>

class Vtk3PlaneWidgets
{
public:
	Vtk3PlaneWidgets();
	virtual ~Vtk3PlaneWidgets();

	void switchOn(vtkImageData* imgData, vtkRenderWindowInteractor* interactor);

	void setValueRange(double range[2]){mValueRange[0] = range[0]; mValueRange[1] = range[1]; mComputeRange = false;}

	void setInterpolationToCubic();
	void setInterpolationToLinear();
	void setInterpolationToNearest();

protected:
	vtkImagePlaneWidget *mImgPlaneX, *mImgPlaneY, *mImgPlaneZ;
	double mValueRange[2];
	bool mComputeRange;
};

#endif /* VTK3PLANEWIDGETS_H_ */
