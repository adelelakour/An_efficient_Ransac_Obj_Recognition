/*
 * EdgeStructure2vtk.h
 *
 *  Created on: Apr 14, 2011
 *      Author: papazov
 */

#ifndef EDGESTRUCTURE2VTK_H_
#define EDGESTRUCTURE2VTK_H_

#include <BasicTools/DataStructures/EdgeStructure.h>
#include <vtkPolyData.h>

class EdgeStructure2vtk
{
public:
	EdgeStructure2vtk();
	virtual ~EdgeStructure2vtk();

	void getEdges(const EdgeStructure* edgeStruct, vtkPolyData* out);
};

#endif /* EDGESTRUCTURE2VTK_H_ */
