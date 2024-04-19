/*
 * BoxStructure3d2vtk.h
 *
 *  Created on: Mar 31, 2010
 *      Author: papazov
 */

#ifndef BOXSTRUCTURE3D2VTK_H_
#define BOXSTRUCTURE3D2VTK_H_

#include <BasicTools/DataStructures/BoxStructure3d.h>
#include <vtkPolyData.h>

class BoxStructure3d2vtk
{
public:
	BoxStructure3d2vtk();
	virtual ~BoxStructure3d2vtk();

	void getAsWireframe(const BoxStructure3d<class T>* boxStruct, vtkPolyData* out);
};

#endif /* BOXSTRUCTURE3D2VTK_H_ */
