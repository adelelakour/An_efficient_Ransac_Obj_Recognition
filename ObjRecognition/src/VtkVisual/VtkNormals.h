/*
 * VtkNormals.h
 *
 *  Created on: Aug 27, 2012
 *      Author: papazov
 */

#ifndef VTKNORMALS_H_
#define VTKNORMALS_H_

#include "VtkParts.h"
#include <btl1/List.h>
#include <btl1/Vector.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <list>
#include <set>

using namespace std;

class VtkNormals
{
public:
	VtkNormals(){}
	virtual ~VtkNormals(){}

	static inline void compute_triangle_normals(vtkPolyData* mesh, vtkDoubleArray* normals);
	static inline void compute_vertex_normals(int kring, vtkPolyData* mesh);
};

//=== inline methods ===========================================================================================

inline void VtkNormals::compute_triangle_normals(vtkPolyData* mesh, vtkDoubleArray* normals)
{
	vtkCellArray *polys = mesh->GetPolys();
	vtkIdList *cell_point_ids = vtkIdList::New();
	vtkIdType i;
	double x[3], y[3], z[3];

	// Prepare the normals array
	normals->SetNumberOfComponents(3);
	normals->SetNumberOfTuples(polys->GetNumberOfCells());

	for ( i = 0, polys->InitTraversal() ; polys->GetNextCell(cell_point_ids) ; cell_point_ids->Reset(), ++i )
	{
		if ( cell_point_ids->GetNumberOfIds() != 3 )
			continue;

		// Get the triangle points
		mesh->GetPoint(cell_point_ids->GetId(0), x);
		mesh->GetPoint(cell_point_ids->GetId(1), y);
		mesh->GetPoint(cell_point_ids->GetId(2), z);

		// Compute the vectors z->x and z->y
		btl1::vec_sub3(x, z);
		btl1::vec_sub3(y, z);
		// Compute the normal
		btl1::vec_cross3(x, y, z);
		btl1::vec_normalize3(z);
		// Save it
		normals->SetTuple(i, z);
	}

	// Clean up
	cell_point_ids->Delete();
}

//==============================================================================================================

inline void VtkNormals::compute_vertex_normals(int k_ring, vtkPolyData* mesh)
{
	// First, compute the triangle normals
	vtkDoubleArray *tria_normals = vtkDoubleArray::New(), *vertex_normals = vtkDoubleArray::New();
	VtkNormals::compute_triangle_normals(mesh, tria_normals);

	// These will be the triangles whose normals will be used to compute the current vertex normal
	set<vtkIdType> adj_trias;
	double vertex_normal[3], tria_normal[3];
	vtkIdType num_points = mesh->GetNumberOfPoints();

	// Initialize the vertex normals array
	vertex_normals->SetNumberOfComponents(3);
	vertex_normals->SetNumberOfTuples(num_points);

	// Now, compute and save the vertex normals
	for ( vtkIdType i = 0 ; i < num_points ; ++i )
	{
		// Get the triangles adjacent to 'v'
		adj_trias.clear();
		VtkParts::get_adjacent_triangles(mesh, i, k_ring, adj_trias);

		// Accumulate the triangle normals
		btl1::vec_set3(vertex_normal, 0.0);
		for ( set<vtkIdType>::iterator at = adj_trias.begin() ; at != adj_trias.end() ; ++at )
		{
			tria_normals->GetTuple(*at, tria_normal);
			btl1::vec_add3(vertex_normal, tria_normal);
		}
		// normalize and save
		btl1::vec_normalize3(vertex_normal);
		vertex_normals->SetTuple(i, vertex_normal);
	}

	// Save the normals
	mesh->GetPointData()->SetNormals(vertex_normals);

	// Clean up
	tria_normals->Delete();
	vertex_normals->Delete();
}

//===========================================================================================================================================

#endif /* VTKNORMALS_H_ */
