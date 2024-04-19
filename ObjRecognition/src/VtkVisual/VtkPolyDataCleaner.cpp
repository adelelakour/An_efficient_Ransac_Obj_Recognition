/*
 * VtkPolyDataCleaner.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: papazov
 */

#include "VtkPolyDataCleaner.h"
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <list>
#include <map>

using namespace std;

VtkPolyDataCleaner::VtkPolyDataCleaner()
{
}

VtkPolyDataCleaner::~VtkPolyDataCleaner()
{
}

//==================================================================================================================

void VtkPolyDataCleaner::copy_triangles(vtkPolyData* src, vtkPolyData* dst)
{
	map<vtkIdType,vtkIdType> src_to_dst_ids;
	map<vtkIdType,vtkIdType>::iterator res;
	vtkIdType dst_point_id = 0;
	vtkPoints *dst_points = vtkPoints::New();
	list<vtkIdType> src_tria_ids;
	double u[3];

	vtkCellArray* src_polys = src->GetPolys();
	vtkIdList* poly_point_ids = vtkIdList::New();

	// Collect the triangles, insert their points and save their ids
	for ( src_polys->InitTraversal() ; src_polys->GetNextCell(poly_point_ids) ; poly_point_ids->Reset() )
	{
		if ( poly_point_ids->GetNumberOfIds() != 3 )
			continue;

		// Check the first triangle point
		res = src_to_dst_ids.find(poly_point_ids->GetId(0));
		if ( res == src_to_dst_ids.end() )
		{
			// We do not have that point yet -> insert it
			src->GetPoint(poly_point_ids->GetId(0), u);
			dst_points->InsertNextPoint(u);
			src_to_dst_ids[poly_point_ids->GetId(0)] = dst_point_id++;
		}

		// Check the second triangle point
		res = src_to_dst_ids.find(poly_point_ids->GetId(1));
		if ( res == src_to_dst_ids.end() )
		{
			// We do not have that point yet -> insert it
			src->GetPoint(poly_point_ids->GetId(1), u);
			dst_points->InsertNextPoint(u);
			src_to_dst_ids[poly_point_ids->GetId(1)] = dst_point_id++;
		}

		// Check the third triangle point
		res = src_to_dst_ids.find(poly_point_ids->GetId(2));
		if ( res == src_to_dst_ids.end() )
		{
			// We do not have that point yet -> insert it
			src->GetPoint(poly_point_ids->GetId(2), u);
			dst_points->InsertNextPoint(u);
			src_to_dst_ids[poly_point_ids->GetId(2)] = dst_point_id++;
		}

		// Save the point ids of the current triangle
		src_tria_ids.push_back(src_to_dst_ids[poly_point_ids->GetId(0)]);
		src_tria_ids.push_back(src_to_dst_ids[poly_point_ids->GetId(1)]);
		src_tria_ids.push_back(src_to_dst_ids[poly_point_ids->GetId(2)]);
	}

	vtkCellArray* dst_trias = vtkCellArray::New();
	vtkIdType tria_ids[3];

	// Create the source triangles
	for ( list<vtkIdType>::iterator id = src_tria_ids.begin() ; id != src_tria_ids.end() ; )
	{
		// Get the right ids using the map
		tria_ids[0] = *id; ++id;
		tria_ids[1] = *id; ++id;
		tria_ids[2] = *id; ++id;
		// Insert the triangle
		dst_trias->InsertNextCell(3, tria_ids);
	}

	// Copy the normals if there are any
	vtkDataArray* src_normals = src->GetPointData()->GetNormals();

	if ( src_normals )
	{
		vtkDoubleArray* dst_normals = vtkDoubleArray::New();
		  dst_normals->SetNumberOfComponents(3);
		  dst_normals->SetNumberOfTuples(dst_points->GetNumberOfPoints());

		// Get the normals
		for ( map<vtkIdType,vtkIdType>::iterator id = src_to_dst_ids.begin() ; id != src_to_dst_ids.end() ; ++id )
		{
			src_normals->GetTuple(id->first, u);
			dst_normals->SetTuple(id->second, u);
		}

		// Save the normals
		dst->GetPointData()->SetNormals(dst_normals);
		// Clean up
		dst_normals->Delete();
	}

	// Save the result
	dst->SetPoints(dst_points);
	dst->SetPolys(dst_trias);

	// Clean up
	poly_point_ids->Delete();
	dst_points->Delete();
	dst_trias->Delete();
}

//==================================================================================================================
