/*
 * ORRRangeImage2vtk.h
 *
 *  Created on: Jun 9, 2010
 *      Author: papazov
 */

#ifndef _ORR_RANGE_IMAGE_2_VTK_H_
#define _ORR_RANGE_IMAGE_2_VTK_H_

#include <ObjRecRANSAC/DataStructures/RangeImage/ORRRangeImage2.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>

class ORRRangeImage2vtk
{
public:
	ORRRangeImage2vtk();
	virtual ~ORRRangeImage2vtk();

	void getAsPolygons(const ORRRangeImage2* image, vtkPolyData* out);
	void getPoints(const ORRRangeImage2* image, vtkPoints* out);
};

#endif /* _ORR_RANGE_IMAGE_2_VTK_H_ */
