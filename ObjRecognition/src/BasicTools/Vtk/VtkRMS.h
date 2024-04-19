/*
 * VtkRMS.h
 *
 *  Created on: Apr 12, 2010
 *      Author: papazov
 */

#ifndef VTKRMS_H_
#define VTKRMS_H_

#include <vtkPolyData.h>
#include <vtkPointLocator.h>

class VtkRMS
{
public:
	VtkRMS();
	VtkRMS(vtkPolyData* main_shape);
	virtual ~VtkRMS();

	void set_main(vtkPolyData* main);
	/** RMS is computed from the 'secondary' to the main shape (set with this->set_main()). */
	double rms(vtkPoints* secondary);

protected:
	vtkPolyData* mMainPolyData;
	vtkPointLocator* mPointLocator;
};

#endif /* VTKRMS_H_ */
