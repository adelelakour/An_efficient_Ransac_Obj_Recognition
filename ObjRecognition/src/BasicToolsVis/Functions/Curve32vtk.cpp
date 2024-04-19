/*
 * Curve32vtk.cpp
 *
 *  Created on: Aug 30, 2011
 *      Author: papazov
 */

#include "Curve32vtk.h"
#include <vtkPoints.h>
#include <vtkCellArray.h>

Curve32vtk::Curve32vtk()
{
}

Curve32vtk::~Curve32vtk()
{
}

//==============================================================================================================================

void Curve32vtk::get(Curve3* curve, double t0, double t1, int numOfSegments, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* lines = vtkCellArray::New();
	double p[3], t, dt = (t1 - t0)/(double)numOfSegments;
	vtkIdType ids[2];
	int i;

	for ( i = 0, t = t0 ; i < numOfSegments ; t += dt )
	{
		curve->g(t, p);
		points->InsertNextPoint(p);
		ids[0] = i;
		ids[1] = ++i;
		lines->InsertNextCell(2, ids);
	}
	curve->g(t, p);
	points->InsertNextPoint(p);

	// Save the result
	out->SetPoints(points);
	out->SetLines(lines);

	// Cleanup
	points->Delete();
	lines->Delete();
}

//==============================================================================================================================
