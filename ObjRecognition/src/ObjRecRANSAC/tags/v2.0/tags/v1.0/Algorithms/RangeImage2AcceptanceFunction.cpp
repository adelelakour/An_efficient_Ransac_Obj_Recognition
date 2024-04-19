/*
 * RangeImage2AcceptanceFunction.cpp
 *
 *  Created on: Jun 8, 2010
 *      Author: papazov
 */

#include "../ORRDefines.h"
#include "RangeImage2AcceptanceFunction.h"
#include <BasicToolsL1/Matrix.h>
#include <BasicToolsL1/Vector.h>
#include <BasicTools/DataStructures/LayeredPointSet.h>


// Global mutex variable
pthread_mutex_t gMutex = PTHREAD_MUTEX_INITIALIZER;
// Global variables initialized in the 'RangeImage2AcceptanceFunction::init()' function
const ORRRangeImage2* gRangeImage;
int** gSceneGrid;
PixelVector* gPixelVector;
// Global variables initialized in the 'RangeImage2AcceptanceFunction::accept()' function
const double* gRigidTransform;
int gHypothesisId;


void *thread_accept(void *_data_)
{
	ThreadStuff* data = (ThreadStuff*)_data_;
	double p[3];
	int x, y;
	const double_2* pixel;

	// Reset the match and penalty counters
	data->reset();

	for ( int k = 0, id = 0 ; k < data->mNumOfPoints ; ++k, id += 3 )
	{
		// Transform the current model point
		Matrix::mult3ByRigid(gRigidTransform, data->mPoints + id, p);
		// Get the pixel corresponding to the transformed point
		pixel = gRangeImage->getSafePixel(p, x, y);
		// Check if we have a valid pixel
		if ( pixel == NULL )
			continue;

		if ( p[2] < pixel->x ) // The transformed model point overshadows a pixel -> penalize it.
			++data->mPenalty;
		else if ( p[2] <= pixel->y ) // The point is OK.
		{
			++data->mMatch;
			// Protect this area from other threads
			pthread_mutex_lock(&gMutex);
			if ( gSceneGrid[x][y] != gHypothesisId )
			{
				gSceneGrid[x][y] = gHypothesisId;
				gPixelVector->pushBackPixelCoords(x, y, gRangeImage->getLinearId(x, y));
			}
			pthread_mutex_unlock(&gMutex);
		}
	}

	return NULL;
}

RangeImage2AcceptanceFunction::RangeImage2AcceptanceFunction()
{
	mThreadStuff = NULL;
	mICPPreProcessing = false;
}

RangeImage2AcceptanceFunction::~RangeImage2AcceptanceFunction()
{
	if ( mThreadStuff ) delete[] mThreadStuff;
}

//=================================================================================================================

int RangeImage2AcceptanceFunction::accept(double* rigid_transform, const LayeredPointSet* layeredModel, int hypoId)
{
#if 0
	int i, numOfPoints = layeredModel->getInputPoints()->GetNumberOfPoints();
	vtkPoints* input_points = layeredModel->getInputPoints();
	double p[3], out[3];
	const double_2* pixel;
	int x, y;

	for ( i = 0 ; i < numOfPoints ; ++i )
	{
		input_points->GetPoint(i, p);
		mat_mult3_by_rigid(rigid_transform, p, out);

		pixel = gRangeImage->getSafePixel(out, x, y);
		// Check if we have a valid pixel
		if ( pixel == NULL )
			continue;
	}
	return 0;
#endif

	if ( mICPPreProcessing )
		this->icp(layeredModel, gRangeImage, rigid_transform);// Perform one ICP loop

	// Initialize the global variables
	gRigidTransform = rigid_transform;
	gHypothesisId = hypoId;

	// Local variables
	int k, pos, numOfPointsPerThread, numOfPointsInLayer;
	int score = 0, numOfIllegalPoints = 0, numOfMatchPoints = 0, numOfMatchPointsThresh, numOfIllegalPointsThresh;

	gPixelVector->resetNumberOfOccupiedPixels();

	for ( int l = 0 ; l < layeredModel->getNumberOfLayers() ; ++l )
	{
		// Some initializations for the current layer
		numOfPointsInLayer = layeredModel->getNumberOfPointsInLayer(l);
		numOfMatchPointsThresh = (int)((double)numOfPointsInLayer*mMatchThresh + 0.5);
		numOfIllegalPointsThresh = (int)((double)numOfPointsInLayer*mPenaltyThresh + 0.5);
		numOfMatchPoints = numOfIllegalPoints = 0;
		// Some more initializations
		numOfPointsPerThread = numOfPointsInLayer/mNumOfThreads;

		// Create the threads and let them compute the match and the penalty
		for ( k = 0, pos = 0 ; k < mNumOfThreads-1 ; ++k )
		{
			mThreadStuff[k].mNumOfPoints = numOfPointsPerThread;
			mThreadStuff[k].mPoints = layeredModel->getLayer(l) + pos;
			pthread_create(&mThreadStuff[k].mThreadId, NULL, thread_accept, (void*)&mThreadStuff[k]);
			// Save the current position for the next iteration
			pos += 3*(mThreadStuff[k].mNumOfPoints);
		}
		// The number of points for the last thread could be different (because of remainder)
		mThreadStuff[k].mNumOfPoints = numOfPointsPerThread + numOfPointsInLayer%mNumOfThreads;
		mThreadStuff[k].mPoints = layeredModel->getLayer(l) + pos;
		pthread_create(&mThreadStuff[k].mThreadId, NULL, thread_accept, (void*)&mThreadStuff[k]);

		// Wait for all threads to finish their job
		for ( k = 0 ; k < mNumOfThreads ; ++k )
			pthread_join(mThreadStuff[k].mThreadId, NULL);

		// Collect the results of all threads
		for ( k = 0 ; k < mNumOfThreads ; ++k )
		{
			numOfMatchPoints += mThreadStuff[k].mMatch;
			numOfIllegalPoints += mThreadStuff[k].mPenalty;
		}

		if ( numOfMatchPoints < numOfMatchPointsThresh || numOfIllegalPoints > numOfIllegalPointsThresh )
			return 0;

		score += numOfMatchPoints;
	}

	return score;
}

//=====================================================================================================================

void RangeImage2AcceptanceFunction::icp(const LayeredPointSet* layeredModel, const ORRRangeImage2* image, double* rigid_transform)
{
	const double *s_p, *m_p = layeredModel->getLayer(0);
	int i, x, y, numOfValidPoints, numOfPoints = layeredModel->getNumberOfPointsInLayer(0);
	double m_transf[3], m_0[3] = {0.0, 0.0, 0.0}, s_0[3] = {0.0, 0.0, 0.0};
	double /*R[3][3],*/ C[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}}, Ncc[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
	const double_2* pixel;

	for ( i = 0, numOfValidPoints = 0 ; i < numOfPoints ; ++i, m_p += 3 )
	{
		// Transform the current model point
		mat_mult3_by_rigid(rigid_transform, m_p, m_transf);
		// Get the pixel corresponding to the transformed model point
		pixel = image->getSafePixel(m_transf, x, y);
		// Check if we have a valid pixel
		if ( pixel == NULL )
			continue;

		// Check if the z-coordinate of 'm_transformed' is within the acceptable range [pixel->x, pixel->y]
		if ( pixel->x <= m_transf[2] && m_transf[2] <= pixel->y )
		{
			// Get the scene point
			s_p = image->getGridSet(x, y)->p();

			// Contribute to the center of mass
			vec_add3(m_0, m_p); // We use 'm_transf' only to establish the correspondence
			vec_add3(s_0, s_p);

			// Contribute to the covariance matrix
			mat_add_tensor_product_to_mat3x3(s_p, m_p, C);

			++numOfValidPoints;
		}
	}

	// We need at least three corresponding points
	if ( numOfValidPoints < 3 )
		return;

	// Compute the center of mass for the model
	vec_mult3(m_0, 1.0/(double)numOfValidPoints);
	// Compute 'Ncc'
	mat_tensor_product3x3(s_0, m_0, Ncc);
	// Compute the covariance matrix
	mat_sub3x3(C, Ncc);
	// Compute the optimal rotation
//	mat_svd_based_polar_decomposition3x3(C, R);
//	mgp.vtkPolarDecomp(C, R);
//	mat_polar_decomposition3x3(C, R);
//	mat_copy3x3(R, rigid_transform);
	mgp.cvPolarDecomp(C, rigid_transform);

	// Compute the center of mass for the scene
	vec_mult3(s_0, 1.0/(double)numOfValidPoints);

	// Compute the optimal translation
	rigid_transform[9]  = s_0[0] - (rigid_transform[0]*m_0[0] + rigid_transform[1]*m_0[1] + rigid_transform[2]*m_0[2]);
	rigid_transform[10] = s_0[1] - (rigid_transform[3]*m_0[0] + rigid_transform[4]*m_0[1] + rigid_transform[5]*m_0[2]);
	rigid_transform[11] = s_0[2] - (rigid_transform[6]*m_0[0] + rigid_transform[7]*m_0[1] + rigid_transform[8]*m_0[2]);
}

//=====================================================================================================================
