/*
 * VtkParts.h
 *
 *  Created on: Aug 27, 2012
 *      Author: papazov
 */

#ifndef VTKPARTS_H_
#define VTKPARTS_H_

#include <btl1/List.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <set>

using namespace std;

class VtkParts
{
public:
	VtkParts(){}
	virtual ~VtkParts(){}

	static inline void get_adjacent_triangles(vtkPolyData* mesh, vtkIdType vertex_id, int k_ring, set<vtkIdType>& adj_trias);
	/** \brief Saves the ids of the triangles adjacent to 'triangle_id' in 'adj_trias'. Make sure that mesh->BuildLinks() has
	  * been called! */
	static inline void get_adjacent_triangles(vtkPolyData* mesh, vtkIdType triangle_id, set<vtkIdType>& adj_trias);
	/** \brief Make sure that mesh->BuildLinks() has been called! */
	static inline void get_neighbors(vtkPolyData* mesh, vtkIdType vertex_id, set<vtkIdType>& neighs);
	static inline void get_triangle_edges(vtkPolyData* in, set<vtkIdType>& tria_ids, vtkPolyData* out);
};

//=== inline methods ===================================================================================================================

inline void VtkParts::get_neighbors(vtkPolyData* mesh, vtkIdType vertex_id, set<vtkIdType>& neighs)
{
	// Get the cells adjacent to 'vertex_id'
	vtkIdList *cell_ids = vtkIdList::New(), *point_ids = vtkIdList::New();
	mesh->GetPointCells(vertex_id, cell_ids);

	// Some counters
	vtkIdType i, k;

	// Get the points of all adjacent cells
	for ( i = 0 ; i < cell_ids->GetNumberOfIds() ; ++i )
	{
		point_ids->Reset();
		mesh->GetCellPoints(cell_ids->GetId(i), point_ids);

		// Try to insert the points of the current adjacent cell
		for ( k = 0 ; k < point_ids->GetNumberOfIds() ; ++k )
			if ( point_ids->GetId(k) != vertex_id )
				neighs.insert(point_ids->GetId(k));
	}

	// Clean up
	cell_ids->Delete();
	point_ids->Delete();
}

//======================================================================================================================================

inline void VtkParts::get_triangle_edges(vtkPolyData* in, set<vtkIdType>& tria_ids, vtkPolyData* out)
{
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray* lines = vtkCellArray::New();
	vtkIdList *cell_point_ids = vtkIdList::New();
	vtkIdType base_id = 0, ids[2];
	double p[3];

	for ( set<vtkIdType>::iterator id = tria_ids.begin() ; id != tria_ids.end() ; ++id )
	{
		// Get the triangle
		vtkCell *tria = in->GetCell(*id);
		// Make sure that it's a triangle
		if ( tria->GetNumberOfPoints() != 3 )
			continue;

		// Insert the triangle points
		tria->GetPoints()->GetPoint(0, p); points->InsertNextPoint(p);
		tria->GetPoints()->GetPoint(1, p); points->InsertNextPoint(p);
		tria->GetPoints()->GetPoint(2, p); points->InsertNextPoint(p);

		// Insert the lines
		ids[0] = base_id;     ids[1] = base_id + 1; lines->InsertNextCell(2, ids);
		ids[0] = base_id + 1; ids[1] = base_id + 2; lines->InsertNextCell(2, ids);
		ids[0] = base_id + 2; ids[1] = base_id    ; lines->InsertNextCell(2, ids);

		base_id += 3;
	}

	// Save the stuff
	out->SetPoints(points);
	out->SetLines(lines);

	// Clean up
	points->Delete();
	lines->Delete();
	cell_point_ids->Delete();
}

//==============================================================================================================

inline void VtkParts::get_adjacent_triangles(vtkPolyData* mesh, vtkIdType vertex_id, int k_ring, set<vtkIdType>& adj_trias)
{
	if ( k_ring < 1 )
		return;

	btl1::List<vtkIdType> tria_list;
	set<vtkIdType> local_adj_trias;
	vtkIdList *adj_ids = vtkIdList::New();
	vtkIdType k, to_expand_id;
	std::pair<set<vtkIdType>::iterator, bool> res;
	int i, size;

	// Important to perform!
	mesh->BuildLinks();

	// Start with the triangles which are "directly" adjacent to 'vertex_id'
	mesh->GetPointCells(vertex_id, adj_ids);

	// Copy the ids of the triangle cells only to the output set
	for ( k = 0 ; k < adj_ids->GetNumberOfIds() ; ++k )
	{
		if ( mesh->GetCell(adj_ids->GetId(k))->GetCellType() == VTK_TRIANGLE )
		{
			adj_trias.insert(adj_ids->GetId(k));
			tria_list.push_back(adj_ids->GetId(k));
		}
	}

	// Cleanup
	adj_ids->Delete();

	// Now get the triangles in the other rings around 'vertex_id'
	for ( i = 1 ; i < k_ring ; ++i )
	{
		// Important to get the size now, since the list can change its size in the next loop
		size = tria_list.size();

		// Pop back the current triangles and push front the new ones
		for ( k = 0 ; k < size ; ++k )
		{
			// Pop the last triangle from the working list
			tria_list.pop_last(to_expand_id);

			// Get its adjacent triangles
			local_adj_trias.clear();
			VtkParts::get_adjacent_triangles(mesh, to_expand_id, local_adj_trias);

			// Try to add the triangles adjacent to 'to_expand_id' to the output triangle set
			for ( set<vtkIdType>::iterator at = local_adj_trias.begin() ; at != local_adj_trias.end() ; ++at )
			{
				// Try to insert
				res = adj_trias.insert(*at);
				// Is 'at' new to us?
				if ( res.second )
					tria_list.push_front(*at);
			}
		}
	}
}

//======================================================================================================================================

inline void VtkParts::get_adjacent_triangles(vtkPolyData* mesh, vtkIdType triangle_id, set<vtkIdType>& adj_trias)
{
	// Get the triangle vertices
	vtkIdList *tria_vertices = vtkIdList::New(), *tria_ids = vtkIdList::New();
	mesh->GetCellPoints(triangle_id, tria_vertices);

	// Some counters
	vtkIdType i, k;

	// For each triangle vertex
	for ( i = 0 ; i < tria_vertices->GetNumberOfIds() ; ++i )
	{
		tria_ids->Reset();
		// Get the cells adjacent to the current triangle vertex
		mesh->GetPointCells(tria_vertices->GetId(i), tria_ids);

		// Insert them into the set
		for ( k = 0 ; k < tria_ids->GetNumberOfIds() ; ++k )
			adj_trias.insert(tria_ids->GetId(k));
	}

	// Clean up
	tria_vertices->Delete();
	tria_ids->Delete();
}

//======================================================================================================================================

#endif /* VTKPARTS_H_ */
