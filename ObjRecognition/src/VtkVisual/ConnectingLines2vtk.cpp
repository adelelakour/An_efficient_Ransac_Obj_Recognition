/*
 * ConnectingLines2vtk.cpp
 *
 *  Created on: Jun 14, 2011
 *      Author: papazov
 */

#include "ConnectingLines2vtk.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>

ConnectingLines2vtk::ConnectingLines2vtk()
{
}

ConnectingLines2vtk::~ConnectingLines2vtk()
{
}

//=====================================================================================================================================

void ConnectingLines2vtk::getVectorsAtPoints(const double *points, const double *vectors, int numberOfPoints, vtkPolyData *out)
{
	vtkPoints *vtk_points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray *vtk_lines = vtkCellArray::New();
	vtkIdType ids[2] = {0, 1};
	double p[3];

	for ( int i = 0 ; i < numberOfPoints ; ++i, points += 3, vectors += 3 )
	{
		p[0] = points[0] + vectors[0];
		p[1] = points[1] + vectors[1];
		p[2] = points[2] + vectors[2];
		vtk_points->InsertNextPoint(points);
		vtk_points->InsertNextPoint(p);
		vtk_lines->InsertNextCell(2, ids);
		ids[0] += 2;
		ids[1] += 2;
	}

	// Save the result
	out->SetPoints(vtk_points);
	out->SetLines(vtk_lines);

	// Cleanup
	vtk_points->Delete();
	vtk_lines->Delete();
}

//=====================================================================================================================================
