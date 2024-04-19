/*
 * LinearBlend.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: papazov
 */

#include "LinearBlend.h"
#include <vtkPointData.h>

LinearBlend::LinearBlend()
{
}

LinearBlend::~LinearBlend()
{
}

//======================================================================================================================

void LinearBlend::init(vtkPolyData* source, vtkPolyData* target)
{
	mSrc = source;
	mTar = target;

	if ( mSrc->GetPointData()->GetScalars() && mTar->GetPointData()->GetScalars() )
		mInterpolateScalars = true;
	else
		mInterpolateScalars = false;
}

//======================================================================================================================
