/*
 * PolygonalCurve2vtk.cpp
 *
 *  Created on: Dec 18, 2011
 *      Author: papazov
 */

#include "PolygonalCurve2vtk.h"
#include <vtkPoints.h>
#include <vtkCellArray.h>

PolygonalCurve2vtk::PolygonalCurve2vtk()
{
}

PolygonalCurve2vtk::~PolygonalCurve2vtk()
{
}

//===============================================================================================================

void PolygonalCurve2vtk::get(const double* points, int num_points, bool closed, const int dim, vtkPolyData* out)
{
	if ( dim != 2 && dim != 3 )
		return;

	vtkPoints *vtk_points = vtkPoints::New(VTK_DOUBLE);
	  vtk_points->SetNumberOfPoints(num_points);
	vtkCellArray *lines = vtkCellArray::New();
	vtkIdType ids[2] = {0, 1};
	const double *p1 = points, *p2 = points;
	double p[3]; p[2] = 0.0;
	int i = 0;

	for ( --num_points, p2 += dim ; i < num_points ; p1 += dim, p2 += dim, ++i )
	{
		// Set the point
		p[0] = p1[0];
		p[1] = p1[1];
		if ( dim == 3 ) p[2] = p1[2];
		vtk_points->SetPoint(i, p);

		// set the line connecting 'p1' and 'p2'
		lines->InsertNextCell(2, ids);
		++ids[0];
		++ids[1];
	}

	// Set the last point
	p[0] = p1[0];
	p[1] = p1[1];
	if ( dim == 3 ) p[2] = p1[2];
	vtk_points->SetPoint(i, p);

	// If the curve is closed, connect the first and last points
	if ( closed )
	{
		ids[0] = num_points;
		ids[1] = 0;
		lines->InsertNextCell(2, ids);
	}

	// Save the result
	out->SetPoints(vtk_points);
	out->SetLines(lines);

	// Cleanup
	vtk_points->Delete();
	lines->Delete();
}

//===============================================================================================================
