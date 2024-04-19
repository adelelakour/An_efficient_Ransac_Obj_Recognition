/*
 * VtkCopy.h
 *
 *  Created on: Jul 8, 2010
 *      Author: papazov
 */

#ifndef VTKCOPY_H_
#define VTKCOPY_H_

#include <vtkPoints.h>
#include <map>

using namespace std;

class VtkCopy
{
public:
	VtkCopy();
	virtual ~VtkCopy();

	inline void copyPoints(vtkPoints* in, vtkPoints* out);
	inline void getNHighestPoints(vtkPoints* in, int N, int coordinate_axis, vtkPoints* out);
	inline void getNLowestPoints(vtkPoints* in, int N, int coordinate_axis, vtkPoints* out);
};

//=== inline methods ==============================================================================================

inline void VtkCopy::copyPoints(vtkPoints* in, vtkPoints* out)
{
	out->SetNumberOfPoints(in->GetNumberOfPoints());
	double p[3];

	for ( vtkIdType i = 0 ; i < in->GetNumberOfPoints() ; ++i )
	{
		in->GetPoint(i, p);
		out->SetPoint(i, p);
	}
}

//=================================================================================================================

inline void VtkCopy::getNLowestPoints(vtkPoints* in, int N, int coordinate_axis, vtkPoints* out)
{
	// Create a multi-map between the 'axis'-coordinates of the points and the corresponding vertices
	multimap<double, int> coord2id;
	double p[3];
	int i;

	// Insert to the multimap (it is a sorted insertion)
	for ( i = 0 ; i < in->GetNumberOfPoints() ; ++i )
	{
		in->GetPoint(i, p);
		coord2id.insert(pair<double,int>(p[coordinate_axis], i));
	}

	// Just to exclude crashes
	if ( N > in->GetNumberOfPoints() )
		N = in->GetNumberOfPoints();

	multimap<double,int>::iterator it;

	// Get the N lowest points
	for ( i = 0, it = coord2id.begin() ; i < N ; ++i, ++it )
	{
		// Fix the point
		in->GetPoint(it->second, p);
		// Insert the point for visualization
		out->InsertNextPoint(p);
	}
}

//=================================================================================================================

inline void VtkCopy::getNHighestPoints(vtkPoints* in, int N, int coordinate_axis, vtkPoints* out)
{
	// Create a multi-map between the 'axis'-coordinates of the points and the corresponding vertices
	multimap<double, int> coord2id;
	double p[3];
	int i;

	// Insert to the multimap (it is a sorted insertion)
	for ( i = 0 ; i < in->GetNumberOfPoints() ; ++i )
	{
		in->GetPoint(i, p);
		coord2id.insert(pair<double,int>(p[coordinate_axis], i));
	}

	// Just to exclude crashes
	if ( N > in->GetNumberOfPoints() )
		N = in->GetNumberOfPoints();

	multimap<double,int>::reverse_iterator it;

	// Get the N highest points
	for ( i = 0, it = coord2id.rbegin() ; i < N ; ++i, ++it )
	{
		// Fix the point
		in->GetPoint(it->second, p);
		// Insert the point for visualization
		out->InsertNextPoint(p);
	}
}

//=================================================================================================================

#endif /* VTKCOPY_H_ */
