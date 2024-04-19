/*
 * VtkUnstructuredGrid2vtkPolyData.cpp
 *
 *  Created on: May 9, 2012
 *      Author: papazov
 */

#include "VtkUnstructuredGrid2vtkPolyData.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkCell.h>


VtkUnstructuredGrid2vtkPolyData::VtkUnstructuredGrid2vtkPolyData()
{
}

VtkUnstructuredGrid2vtkPolyData::~VtkUnstructuredGrid2vtkPolyData()
{
}

//===============================================================================================================================

void VtkUnstructuredGrid2vtkPolyData::getTetrahedraWireframe(vtkUnstructuredGrid *grid, vtkPolyData *out)
{
	int i, j;
	vtkCell *cell;
	double p[3];
	vtkIdType ids[2];

	vtkPoints *points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray *lines = vtkCellArray::New();

	for ( i = 0 ; i < grid->GetNumberOfCells() ; ++i )
	{
		cell = grid->GetCell(i);
		if ( cell->GetCellType() != VTK_TETRA )
			continue;

		if ( (int)cell->GetNumberOfPoints() != 4 )
		{
			fprintf(stderr, "WARNING in 'VtkUnstructuredGrid2vtkPolyData::%s()': tetrahedron %i should have 4 points and not %i\n",
					__func__, i, (int)cell->GetNumberOfPoints());
			continue;
		}

		// Insert the points
		for ( j = 0 ; j < 4 ; ++j )
		{
			cell->GetPoints()->GetPoint(j, p);
			points->InsertNextPoint(p);
		}
		// Insert the lines
		ids[0] = points->GetNumberOfPoints(); ids[1] = ids[0] + 1; // 0 -> 1
		lines->InsertNextCell(2, ids);
		++ids[1]; // 0 -> 2
		lines->InsertNextCell(2, ids);
		++ids[1]; // 0 -> 3
		lines->InsertNextCell(2, ids);
		++ids[0]; ids[1] = ids[0] + 1; // 1 -> 2
		lines->InsertNextCell(2, ids);
		++ids[1]; // 1 -> 3
		lines->InsertNextCell(2, ids);
		++ids[0]; // 2 -> 3
		lines->InsertNextCell(2, ids);
	}

	// Save the points and lines
	out->SetPoints(points);
	out->SetLines(lines);

	// Cleanup
	points->Delete();
	lines->Delete();
}

//===============================================================================================================================
