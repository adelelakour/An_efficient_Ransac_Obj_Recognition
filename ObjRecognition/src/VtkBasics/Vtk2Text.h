/*
 * Vtk2Text.h
 *
 *  Created on: Nov 11, 2011
 *      Author: papazov
 */

#ifndef VTK2TEXT_H_
#define VTK2TEXT_H_

#include <vtkPoints.h>

class Vtk2Text
{
public:
	Vtk2Text();
	virtual ~Vtk2Text();

	void putIdAsTextAtEachPoint(vtkPoints *points, double textSize);
};

#endif /* VTK2TEXT_H_ */
