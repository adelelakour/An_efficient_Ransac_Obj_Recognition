/*
 * VoxelGrid2vtk.cpp
 *
 *  Created on: Aug 23, 2010
 *      Author: papazov
 */

#include "VoxelGrid2vtk.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>

VoxelGrid2vtk::VoxelGrid2vtk()
{
}

VoxelGrid2vtk::~VoxelGrid2vtk()
{
}

//=============================================================================================================================

void VoxelGrid2vtk::getAsWireframe(const double* bounds, const double spacing[3], const int numOfVoxels[3], vtkPolyData* out)
{
	vtkCellArray* lines = vtkCellArray::New();
	vtkPoints* points = vtkPoints::New();
	  points->SetDataTypeToDouble();

	vtkIdType ids[2]; ids[0] = 0;
	int i, j;
	double x, y, z;

	// Insert the lines parallel to the yz-plane
	x = bounds[0];
	for ( i = 0 ; i <= numOfVoxels[0] ; ++i, x += spacing[0] )
	{
		for ( j = 0, z = bounds[4] ; j <= numOfVoxels[2] ; ++j, z += spacing[2], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(x, bounds[2], z);
			points->InsertNextPoint(x, bounds[3], z);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}
	// Insert the lines parallel to the xz-plane
	y = bounds[2];
	for ( i = 0 ; i <= numOfVoxels[1] ; ++i, y += spacing[1] )
	{
		for ( j = 0, z = bounds[4] ; j <= numOfVoxels[2] ; ++j, z += spacing[2], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(bounds[0], y, z);
			points->InsertNextPoint(bounds[1], y, z);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}
	// Insert the planes parallel to the xy-plane
	x = bounds[0];
	for ( i = 0 ; i <= numOfVoxels[0] ; ++i, x += spacing[0] )
	{
		for ( j = 0, y = bounds[2] ; j <= numOfVoxels[1] ; ++j, y += spacing[1], ids[0] += 2 )
		{
			// Insert two points
			points->InsertNextPoint(x, y, bounds[4]);
			points->InsertNextPoint(x, y, bounds[5]);
			// setup the ids
			ids[1] = ids[0]+1;
			// and connect the points with a line
			lines->InsertNextCell(2, ids);
		}
	}

	// Associate the points and lines with the vtk poly object
	out->SetPoints(points);
	out->SetLines(lines);

	// Clean up
	points->Delete();
	lines->Delete();
}

//=============================================================================================================================
