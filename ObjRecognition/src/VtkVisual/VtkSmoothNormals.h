/*
 * VtkSmoothNormals.h
 *
 *  Created on: Aug 29, 2012
 *      Author: papazov
 */

#ifndef VTKSMOOTHNORMALS_H_
#define VTKSMOOTHNORMALS_H_

#include "VtkParts.h"
#include <btl1/Vector.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <set>


class VtkSmoothNormals
{
public:
	VtkSmoothNormals(){}
	virtual ~VtkSmoothNormals(){}

	static inline void smooth_normals(vtkPolyData* mesh, int iterations);
};

//=== inline methods ===============================================================================================================

inline void VtkSmoothNormals::smooth_normals(vtkPolyData* mesh, int iterations)
{
	vtkDataArray *n1, *n2, *tmp, *input_normals = mesh->GetPointData()->GetNormals();
	vtkDoubleArray *tmp_normals = vtkDoubleArray::New();
	  tmp_normals->SetNumberOfComponents(3);
	  tmp_normals->SetNumberOfTuples(mesh->GetNumberOfPoints());

	if ( !input_normals )
	{
		fprintf(stderr, "ERROR in 'VtkSmoothNormals::%s()': the input mesh has no normals.\n", __func__);
		tmp_normals->Delete();
		return;
	}

	// Some variables
	vtkIdType i, k, num_points = mesh->GetNumberOfPoints();
	std::set<vtkIdType> neighs;
	double n[3], vertex_normal[3];

	// Initialize the pointers
	n1 = input_normals;
	n2 = tmp_normals;

	// Make the necessary number of iterations
	for ( k = 0 ; k < iterations ; ++k )
	{
		// For each point
		for ( i = 0 ; i < num_points ; ++i )
		{
			// Reset
			neighs.clear();
			n1->GetTuple(i, vertex_normal);

			// Get the neighbors of the i-th point
			VtkParts::get_neighbors(mesh, i, neighs);

			// Sum the normals of the neighboring points
			for ( set<vtkIdType>::iterator it = neighs.begin() ; it != neighs.end() ; ++it )
			{
				n1->GetTuple(*it, n);
				btl1::vec_add3(vertex_normal, n);
			}

			// Normalize the vector sum
			btl1::vec_normalize3(vertex_normal);
			// Save the normal in the right array
			n2->SetTuple(i, vertex_normal);
		}

		// Swap pointers
		tmp = n1;
		n1 = n2;
		n2 = tmp;
	}

	// If the number of iterations is odd, we have to copy the normals from the temporary array to the input array
	if ( iterations & 0x1 )
		input_normals->DeepCopy(tmp_normals);

	// Clean up
	tmp_normals->Delete();
}

//==================================================================================================================================

#endif /* VTKSMOOTHNORMALS_H_ */
