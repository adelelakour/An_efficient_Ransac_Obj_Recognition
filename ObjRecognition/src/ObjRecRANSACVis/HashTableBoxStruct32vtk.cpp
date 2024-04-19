/*
 * HashTableBoxStruct32vtk.cpp
 *
 *  Created on: Mar 12, 2011
 *      Author: papazov
 */

#include "HashTableBoxStruct32vtk.h"
#include <btl1/Vector.h>
#include <ObjRecRANSAC/DataStructures/DatabaseModelEntry.h>
#include <vtkDataArray.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>

using namespace btl1;

HashTableBoxStruct32vtk::HashTableBoxStruct32vtk()
{
}

HashTableBoxStruct32vtk::~HashTableBoxStruct32vtk()
{
}

//===================================================================================================================
#if 0
void HashTableBoxStruct32vtk::getKeysAsPoints(const HashTableBoxStruct3* hash, vtkPoints* out)
{
	const set<HashTableCell3*>& fullcells = hash->getFullCells();
	map<DatabaseModelEntry*, HashTableKeys3*>::const_iterator entry;

	// For all full cells in the hash table
	for ( set<HashTableCell3*>::iterator cell = fullcells.begin() ; cell != fullcells.end() ; ++cell )
	{
		// For all models in the current cell
		for ( entry = (*cell)->getCellEntries().begin() ; entry != (*cell)->getCellEntries().end() ; ++entry )
		{
			const list<HashTableKeys3::Key3*>& keys = entry->second->getKeys();
			// For all keys for the current model
			for ( list<HashTableKeys3::Key3*>::const_iterator key = keys.begin() ; key != keys.end() ; ++key )
				out->InsertNextPoint((*key)->mKey);
		}
	}
}

//===================================================================================================================

void HashTableBoxStruct32vtk::getTriples(const HashTableCell3* cell, vtkPolyData* out, vtkPolyData* normals, double normal_scale)
{
	const map<DatabaseModelEntry*, HashTableKeys3*>& entries = cell->getCellEntries();
	map<DatabaseModelEntry*, HashTableKeys3*>::const_iterator entry;
	list<HashTableKeys3::Key3*>::const_iterator key3;
	vtkPolyData* model;
	double p[3], model_normal[3];
	vtkPoints *points = vtkPoints::New(VTK_DOUBLE), *normal_points = vtkPoints::New(VTK_DOUBLE);
	vtkCellArray *lines = vtkCellArray::New(), *normal_lines = vtkCellArray::New();
	vtkIdType base = 0, ids[2], normal_ids[2];

	// For all models
	for ( entry = entries.begin() ; entry != entries.end() ; ++entry )
	{
		model = entry->first->getOwnModel();
		vtkDataArray* model_normals = model->GetPointData()->GetNormals();

		for ( key3 = entry->second->getKeys().begin() ; key3 != entry->second->getKeys().end() ; ++key3, base += 3 )
		{
			// Insert the points
			model->GetPoint((*key3)->mId1, p);
			points->InsertNextPoint(p);
			if ( normals )
			{
				normal_points->InsertNextPoint(p);
				model_normals->GetTuple((*key3)->mId1, model_normal);
				vec_mult3(model_normal, normal_scale);
				vec_add3(model_normal, p);
				normal_points->InsertNextPoint(model_normal);
			}
			model->GetPoint((*key3)->mId2, p);
			points->InsertNextPoint(p);
			if ( normals )
			{
				normal_points->InsertNextPoint(p);
				model_normals->GetTuple((*key3)->mId2, model_normal);
				vec_mult3(model_normal, normal_scale);
				vec_add3(model_normal, p);
				normal_points->InsertNextPoint(model_normal);
			}
			model->GetPoint((*key3)->mId3, p);
			points->InsertNextPoint(p);
			if ( normals )
			{
				normal_points->InsertNextPoint(p);
				model_normals->GetTuple((*key3)->mId3, model_normal);
				vec_mult3(model_normal, normal_scale);
				vec_add3(model_normal, p);
				normal_points->InsertNextPoint(model_normal);
			}

			// Insert the lines
			ids[0] = base; ids[1] = base + 1;
			lines->InsertNextCell(2, ids);
			++ids[0]; ++ids[1];
			lines->InsertNextCell(2, ids);
			++ids[0]; ids[1] = base;
			lines->InsertNextCell(2, ids);

			// Insert the lines for the normals
			if ( normals )
			{
				normal_ids[0] = 2*base; normal_ids[1] = 2*base + 1;
				normal_lines->InsertNextCell(2, normal_ids);
				normal_ids[0] += 2; normal_ids[1] += 2;
				normal_lines->InsertNextCell(2, normal_ids);
				normal_ids[0] += 2; normal_ids[1] += 2;
				normal_lines->InsertNextCell(2, normal_ids);
			}
		}
	}

	// Save the result
	out->SetPoints(points);
	out->SetLines(lines);
	if ( normals )
	{
		normals->SetPoints(normal_points);
		normals->SetLines(normal_lines);
	}
	// Cleanup
	points->Delete();
	lines->Delete();
	normal_points->Delete();
	normal_lines->Delete();
}

//===================================================================================================================
#endif
