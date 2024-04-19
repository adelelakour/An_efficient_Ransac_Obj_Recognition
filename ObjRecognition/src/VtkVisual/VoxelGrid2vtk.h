/*
 * VoxelGrid2vtk.h
 *
 *  Created on: Aug 23, 2010
 *      Author: papazov
 */

#ifndef VOXELGRID2VTK_H_
#define VOXELGRID2VTK_H_

#include <vtkPolyData.h>

class VoxelGrid2vtk
{
public:
	VoxelGrid2vtk();
	virtual ~VoxelGrid2vtk();

	void getAsWireframe(const double* bounds, const double spacing[3], const int numOfVoxels[3], vtkPolyData* out);
};

#endif /* VOXELGRID2VTK_H_ */
