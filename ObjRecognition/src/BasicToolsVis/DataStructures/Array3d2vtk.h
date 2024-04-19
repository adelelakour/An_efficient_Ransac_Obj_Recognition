/*
 * Array3d2vtk.h
 *
 *  Created on: Jun 11, 2010
 *      Author: papazov
 */

#ifndef ARRAY3D2VTK_H_
#define ARRAY3D2VTK_H_

#include <BasicTools/DataStructures/Array3d.h>
#include <vtkPolyData.h>

class Array3d2vtk
{
public:
	Array3d2vtk();
	virtual ~Array3d2vtk();

	void getNonZerosAsPolygons(Array3d<char>* array, double begin[3], double step, vtkPolyData* out);
};

#endif /* ARRAY3D2VTK_H_ */
