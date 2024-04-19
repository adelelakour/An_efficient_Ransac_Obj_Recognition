/*
 * VtkPolyDataCleaner.h
 *
 *  Created on: Sep 18, 2012
 *      Author: papazov
 */

#ifndef VTKPOLYDATACLEANER_H_
#define VTKPOLYDATACLEANER_H_

#include <vtkPolyData.h>

class VtkPolyDataCleaner
{
public:
	VtkPolyDataCleaner();
	virtual ~VtkPolyDataCleaner();

	void copy_triangles(vtkPolyData* src, vtkPolyData* dst);
};

#endif /* VTKPOLYDATACLEANER_H_ */
