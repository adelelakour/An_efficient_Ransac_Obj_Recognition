/*
 * SphericalBox2vtk.h
 *
 *  Created on: Apr 9, 2010
 *      Author: papazov
 */

#ifndef SPHERICALBOX2VTK_H_
#define SPHERICALBOX2VTK_H_

#include <vtkPolyData.h>

class SphericalBox2vtk
{
public:
	SphericalBox2vtk();
	virtual ~SphericalBox2vtk();

	/** A spherical box is a set of the form:
	  * {(x, y, z) in R^3 : (x, y, z) = (Rcos(phi)sin(psi),  Rsin(phi)sin(psi), Rcos(psi))}, where
	  * phi in [range[0][0], range[0][1]],
	  * psi in [range[1][0], range[1][1]],
	  *   R in [range[2][0], range[2][1]]. */
	void getAsWireframe(const double** range, int resolution_phi, int resolution_psi, vtkPolyData* out);
};

#endif /* SPHERICALBOX2VTK_H_ */
