/*
 * PolygonalCurve2vtk.h
 *
 *  Created on: Dec 18, 2011
 *      Author: papazov
 */

#ifndef POLYGONALCURVE2VTK_H_
#define POLYGONALCURVE2VTK_H_

#include <vtkPolyData.h>

class PolygonalCurve2vtk
{
public:
	PolygonalCurve2vtk();
	virtual ~PolygonalCurve2vtk();

	void get(const double* points, int num_points, bool closed, const int dim, vtkPolyData* out);
};

#endif /* POLYGONALCURVE2VTK_H_ */
