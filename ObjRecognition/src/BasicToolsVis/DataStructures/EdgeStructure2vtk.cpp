/*
 * EdgeStructure2vtk.cpp
 *
 *  Created on: Apr 14, 2011
 *      Author: papazov
 */

#include "EdgeStructure2vtk.h"
#include <vtkCellArray.h>
#include <vtkPoints.h>

EdgeStructure2vtk::EdgeStructure2vtk()
{
}

EdgeStructure2vtk::~EdgeStructure2vtk()
{
}

//=======================================================================================================

void EdgeStructure2vtk::getEdges(const EdgeStructure* edgeStruct, vtkPolyData* out)
{
	vtkPoints* vtk_points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* vtk_lines = vtkCellArray::New();
	vtkIdType ids[2];
	const double *p = edgeStruct->getPoints_const();
	list<int>::const_iterator neigh_id;
	int i;

	// Set the geometry
	for ( i = 0 ; i < edgeStruct->getNumberOfPoints() ; ++i, p += 3 )
		vtk_points->InsertNextPoint(p);

	unsigned int max_num_of_neighs = 0;

	// Set the topology
	for ( i = 0 ; i < edgeStruct->getNumberOfPoints() ; ++i )
	{
		const list<int>& neigh_ids = edgeStruct->getNeighborhood(i);
		ids[0] = i;

		if ( neigh_ids.size() > max_num_of_neighs )
			max_num_of_neighs = neigh_ids.size();

		// Set the second id
		for ( neigh_id = neigh_ids.begin() ; neigh_id != neigh_ids.end() ; ++neigh_id )
		{
			ids[1] = *neigh_id;
			vtk_lines->InsertNextCell(2, ids);
		}
	}
	// Save the result
	out->SetPoints(vtk_points);
	out->SetLines(vtk_lines);
	// Cleanup
	vtk_points->Delete();
	vtk_lines->Delete();

	printf("EdgeStructure2vtk::%s(): max number of neighbors = %i\n", __func__, max_num_of_neighs);
}

//=======================================================================================================
