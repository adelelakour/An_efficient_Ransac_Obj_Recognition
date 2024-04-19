/*
 * RangeImage2AcceptanceFunction.h
 *
 *  Created on: Jun 8, 2010
 *      Author: papazov
 */

#ifndef _RANGE_IMAGE_2_ACCEPTANCE_FUNCTION_H_
#define _RANGE_IMAGE_2_ACCEPTANCE_FUNCTION_H_

#include "../PixelVector.h"
#include "../DataStructures/RangeImage/ORRRangeImage2.h"
#include "../DataStructures/DatabaseModelEntry.h"
#include "GeometryProcessor.h"
#include <vtkPoints.h>
#include <pthread.h>

extern const ORRRangeImage2* gRangeImage;
extern int** gSceneGrid;
extern PixelVector* gPixelVector;

class ThreadStuff
{
public:
	ThreadStuff(){ this->reset();}
	virtual ~ThreadStuff(){}
	void reset(){ mMatch = mPenalty = 0;}

	const double* mPoints;
	int mNumOfPoints, mMatch, mPenalty;
	pthread_t mThreadId;
};

class RangeImage2AcceptanceFunction
{
public:
	RangeImage2AcceptanceFunction();
	virtual ~RangeImage2AcceptanceFunction();

	inline void init(ORRRangeImage2* img, int** sceneGrid, PixelVector* pixelVector,
			double matchThresh, double penaltyThresh, int numOfThreads, bool icpPreProcessing);

	int accept(double* rigidTransform, const LayeredPointSet* layeredModel, int hypoId);
	void icp(const LayeredPointSet* layeredModel, const ORRRangeImage2* image, double* rigid_transform);

protected:
	double mMatchThresh, mPenaltyThresh;
	int mNumOfThreads;
	ThreadStuff *mThreadStuff;
	GeometryProcessor mgp;
	bool mICPPreProcessing;
};

//=== inline method ===================================================================================================

void RangeImage2AcceptanceFunction::init(ORRRangeImage2* img, int** sceneGrid, PixelVector* pixelVector,
		double matchThresh, double penaltyThresh, int numOfThreads, bool icpPreProcessing)
{
	// Init the global variables
	gRangeImage = img;
	gSceneGrid = sceneGrid;
	gPixelVector = pixelVector;

	mMatchThresh = matchThresh;
	mPenaltyThresh = penaltyThresh;
	mNumOfThreads = numOfThreads;

	if ( mThreadStuff ) delete[] mThreadStuff;
	mThreadStuff = new ThreadStuff[numOfThreads];

	mICPPreProcessing = icpPreProcessing;
}

//=====================================================================================================================

#endif /* _RANGE_IMAGE_2_ACCEPTANCE_FUNCTION_H_ */
