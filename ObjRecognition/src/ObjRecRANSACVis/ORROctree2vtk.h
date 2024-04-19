	/*
 * ORROctree2vtk.h
 *
 *  Created on: Mar 12, 2011
 *      Author: papazov
 */

#ifndef ORROCTREE2VTK_H_
#define ORROCTREE2VTK_H_

#include <ObjRecRANSAC/DataStructures/ORROctree/ORROctree.h>
#include <vtkPolyData.h>

class ORROctree2vtk
{
public:
	ORROctree2vtk();
	virtual ~ORROctree2vtk();
#if 0
	void getFullLeavesWithVisIdAsPoly(ORROctree* octree, int visId, vtkPolyData* out,
			list<const double*>* nodePoints = NULL, vtkPolyData* normals = NULL, double normal_scale = 1.0);
#endif
};

#endif /* ORROCTREE2VTK_H_ */
