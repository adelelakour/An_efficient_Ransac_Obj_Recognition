/*
 * ConnectingLines2vtk.h
 *
 *  Created on: Jun 14, 2011
 *      Author: papazov
 */

#ifndef CONNECTINGLINES2VTK_H_
#define CONNECTINGLINES2VTK_H_

#include <vtkPolyData.h>

class ConnectingLines2vtk
{
public:
	ConnectingLines2vtk();
	virtual ~ConnectingLines2vtk();

	void getVectorsAtPoints(const double *points, const double *vectors, int numberOfPoints, vtkPolyData *out);
};

#endif /* CONNECTINGLINES2VTK_H_ */
