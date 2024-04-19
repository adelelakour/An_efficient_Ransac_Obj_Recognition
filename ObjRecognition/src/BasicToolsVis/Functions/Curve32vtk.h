/*
 * Curve32vtk.h
 *
 *  Created on: Aug 30, 2011
 *      Author: papazov
 */

#ifndef CURVE32VTK_H_
#define CURVE32VTK_H_

#include <BasicTools/Functions/Curve3.h>
#include <vtkPolyData.h>


class Curve32vtk
{
public:
	Curve32vtk();
	virtual ~Curve32vtk();

	void get(Curve3* curve, double t0, double t1, int numOfSegments, vtkPolyData* out);
};

#endif /* CURVE32VTK_H_ */
