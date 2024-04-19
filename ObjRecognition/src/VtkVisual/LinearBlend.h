/*
 * LinearBlend.h
 *
 *  Created on: Feb 20, 2012
 *      Author: papazov
 */

#ifndef LINEARBLEND_H_
#define LINEARBLEND_H_

#include <vtkPolyData.h>
#include <btl1/Vector.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>

using namespace btl1;

class LinearBlend
{
public:
	LinearBlend();
	virtual ~LinearBlend();

	void init(vtkPolyData* source, vtkPolyData* target);

	/** The method computes a shape between 'source' and 'target' at time 't' (which should be in [0, 1]) by
	  * linear point position interpolation. If 'source' and 'target' have scalars, they will be interpolated as well.  */
	inline void interpolate(double t, vtkPolyData *out);

protected:
	vtkPolyData *mSrc, *mTar;
	bool mInterpolateScalars;
};

//=== inline methods ===========================================================================================================

inline void LinearBlend::interpolate(double t, vtkPolyData* out)
{
	if ( t < 0.0 || t > 1.0 )
		return;

	int i, num_points = (int)mSrc->GetNumberOfPoints();
	vtkPoints* out_points = out->GetPoints();
	double p[3], src_p[3], tar_p[3], tar_t = t, src_t = 1.0 - t;

	// First, interpolate the point positions
	for ( i = 0 ; i < num_points ; ++i )
	{
		// Get the points
		mSrc->GetPoint(i, src_p);
		mTar->GetPoint(i, tar_p);

		// Interpolate
		vec_mult3(src_p, src_t);
		vec_mult3(tar_p, tar_t);
		vec_sum3(src_p, tar_p, p);

		// Write back the point
		out_points->SetPoint(i, p);
	}

	if ( mInterpolateScalars && out->GetPointData()->GetScalars() )
	{
		vtkDataArray *inter_scalars = out->GetPointData()->GetScalars(),
				*src_scalars = mSrc->GetPointData()->GetScalars(),
				*tar_scalars = mTar->GetPointData()->GetScalars();

		for ( i = 0 ; i < num_points ; ++i )
			inter_scalars->SetTuple1(i, tar_t*tar_scalars->GetTuple1(i) + src_t*src_scalars->GetTuple1(i));
	}
}

//==============================================================================================================================

#endif /* LINEARBLEND_H_ */
