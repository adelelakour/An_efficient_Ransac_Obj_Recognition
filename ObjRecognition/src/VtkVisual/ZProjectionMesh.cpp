/*
 * ZProjectionMesh.cpp
 *
 *  Created on: Nov 1, 2010
 *      Author: papazov
 */

#include "ZProjectionMesh.h"
#include <BasicTools/DataStructures/PtrArray2d.h>
#include <BasicTools/LinearAlgebra/Vector.h>
#include <vtkCellArray.h>

using namespace tum;

ZProjectionMesh::ZProjectionMesh()
{
	mBorderFrac = 0.001;
}

ZProjectionMesh::~ZProjectionMesh()
{
}

//=====================================================================================================================

void ZProjectionMesh::create_mesh(vtkPoints* input, double voxelsize, vtkPolyData* out, double max_z_diff)
{
	double p[3], b[4]; this->comp_xy_bounds(input, b);
	double len[2] = {b[1]-b[0], b[3]-b[2]};
	int i, j, k, width, height;
	bool insert_element;

	// Set some variables
	width  = 1 + (int)(len[0]/voxelsize);
	height = 1 + (int)(len[1]/voxelsize);

	// Create the pixel array
	PtrArray2d<ZPMPixel> img(width, height);
	ZPMPixel*** pixels = img.mData;

	// Determine the bounds of the structure
	double offset[2] = {0.5*(voxelsize*(double)width  - len[0]), 0.5*(voxelsize*(double)height - len[1])};
	b[0] -= offset[0]; b[1] += offset[0];
	b[2] -= offset[1]; b[3] += offset[1];

	// Create the image
	for ( k = 0 ; k < input->GetNumberOfPoints() ; ++k )
	{
		input->GetPoint(k, p);
		i = (int)((p[0] - b[0])/voxelsize);
		j = (int)((p[1] - b[2])/voxelsize);

		// Create a new voxel if necessary
		if ( !pixels[i][j] )
		{
			pixels[i][j] = new ZPMPixel();
			Vector::copy3(p, pixels[i][j]->p);
			pixels[i][j]->numOfPts = 1;
		}
		else
		{
			Vector::add3(pixels[i][j]->p, p);
			++(pixels[i][j]->numOfPts);
		}
	}

	// Points, edges and triangles
	vtkCellArray *edges = vtkCellArray::New(), *trias = vtkCellArray::New();
	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	vtkIdType edge_ids[2], tria_ids[3];

	// Insert the points
	for ( j = 0, k = 0 ; j < height ; ++j )
	{
		for ( i = 0 ; i < width ; ++i )
		{
			if ( !pixels[i][j] )
				continue;

			// Construct the point
			Vector::mult3(pixels[i][j]->p, 1.0/(double)(pixels[i][j]->numOfPts));
			pixels[i][j]->numOfPts = 1;

			// Save it
			points->InsertNextPoint(pixels[i][j]->p);
			pixels[i][j]->id = k++;
		}
	}

	// Now create the edges and triangles
	for ( j = 0 ; j < height-1 ; ++j )
	{
		for ( i = 0 ; i < width-1 ; ++i )
		{
			if ( !pixels[i][j] )
				continue;

			// First triangle and first edges
			if ( pixels[i][j+1] && pixels[i+1][j+1] )
			{
				insert_element = true;

				if ( max_z_diff > 0.0 )
					// Check the triangle for too large z-differences
					insert_element = this->check_z_diff(pixels[i][j], pixels[i][j+1], pixels[i+1][j+1], max_z_diff);

				if ( insert_element )
				{
					tria_ids[0] = pixels[i][j]->id;
					tria_ids[1] = pixels[i][j+1]->id;
					tria_ids[2] = pixels[i+1][j+1]->id;
					trias->InsertNextCell(3, tria_ids);
				}
			}
			else if ( pixels[i][j+1] )
			{
				insert_element = true;

				if ( max_z_diff > 0.0 )
					// Check the triangle for too large z-differences
					insert_element = this->check_z_diff(pixels[i][j], pixels[i][j+1], max_z_diff);

				if ( insert_element )
				{
					edge_ids[0] = pixels[i][j]->id;
					edge_ids[1] = pixels[i][j+1]->id;
					edges->InsertNextCell(2, edge_ids);
				}
			}
			else if ( pixels[i+1][j+1] )
			{
				insert_element = true;

				if ( max_z_diff > 0.0 )
					// Check the triangle for too large z-differences
					insert_element = this->check_z_diff(pixels[i][j], pixels[i+1][j+1], max_z_diff);

				if ( insert_element )
				{
					edge_ids[0] = pixels[i][j]->id;
					edge_ids[1] = pixels[i+1][j+1]->id;
					edges->InsertNextCell(2, edge_ids);
				}
			}

			// Second triangle and second group of edges
			if ( pixels[i+1][j] && pixels[i+1][j+1] )
			{
				insert_element = true;

				if ( max_z_diff > 0.0 )
					// Check the triangle for too large z-differences
					insert_element = this->check_z_diff(pixels[i][j], pixels[i+1][j], pixels[i+1][j+1], max_z_diff);

				if ( insert_element )
				{
					tria_ids[0] = pixels[i][j]->id;
					tria_ids[1] = pixels[i+1][j]->id;
					tria_ids[2] = pixels[i+1][j+1]->id;
					trias->InsertNextCell(3, tria_ids);
				}
			}
			else if ( pixels[i+1][j] )
			{
				insert_element = true;

				if ( max_z_diff > 0.0 )
					// Check the triangle for too large z-differences
					insert_element = this->check_z_diff(pixels[i][j], pixels[i+1][j], max_z_diff);

				if ( insert_element )
				{
					edge_ids[0] = pixels[i][j]->id;
					edge_ids[1] = pixels[i+1][j]->id;
					edges->InsertNextCell(2, edge_ids);
				}
			}
		}
	}

	// Save the points, edges and triangles
	out->SetPoints(points);
	out->SetLines(edges);
	out->SetPolys(trias);

	// Cleanup
	points->Delete();
	edges->Delete();
	trias->Delete();
}

//=====================================================================================================================
