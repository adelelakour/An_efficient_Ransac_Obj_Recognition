#ifndef OBJRECRANSAC_H_
#define OBJRECRANSAC_H_

#include "ORRDefines.h"
#include <BasicToolsL1/Vector.h>
#include <BasicTools/DataStructures/LayeredPointSet.h>
#include <BasicTools/ComputationalGeometry/Algorithms/PCA.h>
#include <BasicTools/ComputationalGeometry/Algorithms/OptimalTransformation.h>
#include <BasicTools/LinearAlgebra/Matrix.h>
#include <BasicTools/LinearAlgebra/Vector.h>
#include <BasicTools/Graph/UndirectedGraph.h>
#include "PixelVector.h"
#include "DataStructures/RangeImage/ORRRangeImage2.h"
#include "Algorithms/RangeImage2AcceptanceFunction.h"
#include "Algorithms/HashTableKeyGenerator.h"
#include "Shapes/PointSetShape.h"
#include "Shapes/ORRPointSetShape.h"
#include "ModelDatabase.h"
#include "UserData.h"
#include "DataStructures/HashTableBoxStruct/HashTableBoxStruct.h"
#include "DataStructures/ORROctree/ORROctreeNodeData.h"
#include "DataStructures/ORROctree/ORROctree.h"
#include "DataStructures/ORRUndirectedGraph.h"
#include "DataStructures/OctreeNodeStamp.h"
#include <vtkPolyData.h>
#include <cstdio>
#include <vector>
#include <cmath>
#include <list>

using namespace std;
using namespace tum;

class ObjRecRANSAC
{
public:
	class OrientedPair
	{
	public:
		OrientedPair(const double *p1, const double *n1, const double *p2, const double *n2){
			vec_copy3(p1, this->p1); vec_copy3(n1, this->n1); vec_copy3(p2, this->p2); vec_copy3(n2, this->n2);
		}
		virtual ~OrientedPair(){}
		double p1[3], n1[3], p2[3], n2[3];
	};

public:
	ObjRecRANSAC(double pairwidth, double voxelsize, double relNumOfPairsInHashTable = 0.8);
	virtual ~ObjRecRANSAC();

	bool addModel(vtkPolyData* model, UserData* userData);
	/** Do NOT forget to delete the shapes saved in 'detectedShapes' after using them! */
	void doRecognition(vtkPoints* scene, double successProbability, list<PointSetShape*>& detectedShapes);

	/** Usually you do not need to call this method. It will be called within 'this->doRecognition()'. */
	bool buildSceneOctree(vtkPoints* scene, double voxelsize);

	void setVisibility(double value){ mVisibility = value;}
	void setRelativeObjectSize(double value){ mRelativeObjSize = value;}
	void setRelativeNumberOfIllegalPoints(double value){ mRelativeNumOfIllegalPts = value;}
	void setAbsZDistThresh(double value){ mAbsZDistThresh = value*mVoxelSize;}
	void setNormalEstimationRadius(int value){ mNormalEstimationNeighRadius = value;}
	void setIntersectionFraction(double value){ mIntersectionFraction = value;}
	void setNumberOfThreads(int numOfThreads){ mNumOfThreads = numOfThreads;}

	vtkPoints* getInputScene(){ return mInputScene;}
	ORROctree* getSceneOctree(){ return mSceneOctree;}
	ORRRangeImage2* getSceneRangeImage(){ return &mSceneRangeImage;}
	ModelDatabase* getModelDatabase(){ return &mModelDatabase;}
	DatabaseModelEntry* getDatabaseModelEntry(vtkPolyData* polydata){ return mModelDatabase.getModelEntry(polydata);}

	double getLastOverallRecognitionTimeSec(){ return mLastOverallRecognitionTimeSec;}
	double getLastPureRecognitionTimeSec(){ return mLastPureRecognitionTimeSec;}
	int getLastNumberOfCheckedHypotheses(){ return mNumOfCheckedHypotheses;}
	int getNumberOfThreads(){ return mNumOfThreads;}
	list<OrientedPair>& getSampledPairs(){ return mSampledPairs;}

	void useAbsoluteObjSize(int numOfModelPoints){ mEstOfModelPointsInTheScene = numOfModelPoints; mUseAbsoluteObjSize = true;}
	void useRelativeObjSize(){ mUseAbsoluteObjSize = false;}

	void setICPPreProcessing(bool value){ mICPPreProcessing = value;}
	void setICPPostProcessing(bool value){ mICPPostProcessing = value;}

	/** This method clears everything (frees memory, etc...). It is called automatically by the destruction
	 * of this object. */
	void clear();

	void printParameters(FILE* fp);

protected:
	void sample(OctreeNode** leaves1, int numOfLeaves);
	void clear_rec();
	void init_rec(vtkPoints* scene);
	bool fillModelHashTable(vtkPolyData* model);
	void checkObjectHypotheses(const double* p1, const double* n1, const double* p2, const double* n2,
			HashTableCell** cells, int numOfCells);
	inline void collectCenteredPoints(list<OctreeNode*>& nodes, double** out);
	inline double* estimateNodeNormal(double** ptsToUse, OctreeNode* node, ORROctreeNodeData* node_data);
	void generateAndTest(const double* scenePoint1, const double* sceneNormal1,
			const double* scenePoint2, const double* sceneNormal2);
	void gridBasedFiltering(vector<ORRPointSetShape*>& shapes, list<ORRPointSetShape*>& out);
	inline int computeNumberOfIterations(double successProbability, int numOfScenePoints);

	/** Computes the intersection between %shape1 and %shape2. Returns true if the intersection is significant,
	  * meaning that the cardinality of the intersection set is a large fraction of the id sets of the shapes. */
	inline bool significantIntersection(ORRPointSetShape* shape1, ORRPointSetShape* shape2);

	inline void getIdPair(const ORRPointSetShape* shape1, const ORRPointSetShape* shape2, std::pair<int,int>& id_pair);

public:
	double mMeanConfidence;

protected:
	ModelDatabase mModelDatabase;
	ORROctree *mSceneOctree;
	OptimalTransformation mOptTransform;
	PCA mPCA;
	HashTableKeyGenerator mKeyGen;
	double mLastOverallRecognitionTimeSec, mLastPureRecognitionTimeSec;
	int mNumOfThreads, mNumOfCheckedHypotheses;
	PixelVector* mPixelVector;
	vtkPoints* mInputScene;

	ORRRangeImage2 mSceneRangeImage;
	RangeImage2AcceptanceFunction mRangeImageAccept;
	list<list<int>* > mOccupiedPixelsByShapes;
	vector<ORRPointSetShape*> mShapes;
	// Only for 3D hash tables: this is the max number of neighbors
	// a 3D hash table cell can have!
	HashTableCell *mMaxCellNeighbors[27];

	list<OrientedPair> mSampledPairs;
	int *mMaxModelSupport;
	ORRPointSetShape **mBestShape;
	PixelVector **mPixelVectorPerModel;

	bool mUseAbsoluteObjSize;

	// Parameters
	int mNormalEstimationNeighRadius, mNumOfPointsPerLayer, mEstOfModelPointsInTheScene;
	double mRelNumOfPairsInTheHashTable;
	double mVoxelSize, mVisibility, mRelativeNumOfIllegalPts;
	double mRelativeObjSize, mPairWidth, mAbsZDistThresh;
	double mRelNumOfPairsToKill, mIntersectionFraction;
	bool mICPPreProcessing, mICPPostProcessing;
};

//=== inline methods ==============================================================================================

int ObjRecRANSAC::computeNumberOfIterations(double successProbability, int numOfScenePoints)
{
	// 'p_obj' is the probability that given that the first sample point belongs to an object,
	// the second sample point will belong to the same object
	double p_obj = 0.25;
	double P;

	if ( mUseAbsoluteObjSize )
		P = p_obj*((double)mEstOfModelPointsInTheScene/numOfScenePoints)*mRelNumOfPairsInTheHashTable;
	else
		P = p_obj*mRelativeObjSize*mRelNumOfPairsInTheHashTable;

	if ( 1.0-P <= 0.0 )
		return 1;

	return (int)(log(1.0-successProbability)/log(1.0-P) + 1.0);
}

//=================================================================================================================

inline void ObjRecRANSAC::collectCenteredPoints(list<OctreeNode*>& nodes, double** out)
{
	const double *p;
	double com[3] = {0.0, 0.0, 0.0};
	list<OctreeNode*>::iterator it;

	for ( it = nodes.begin() ; it != nodes.end() ; ++it )
	{
		p = ((ORROctreeNodeData*)(*it)->getData())->getPoint();
		com[0] += p[0];
		com[1] += p[1];
		com[2] += p[2];
	}
	com[0] /= (double)nodes.size();
	com[1] /= (double)nodes.size();
	com[2] /= (double)nodes.size();

	int i;
	for ( i = 0, it = nodes.begin() ; it != nodes.end() ; ++it, ++i )
	{
		p = ((ORROctreeNodeData*)(*it)->getData())->getPoint();
		out[0][i] = p[0] - com[0];
		out[1][i] = p[1] - com[1];
		out[2][i] = p[2] - com[2];
	}
}

//================================================================================================================================

inline double* ObjRecRANSAC::estimateNodeNormal(double** ptsToUse, OctreeNode* node, ORROctreeNodeData* node_data)
{
	if ( !node_data->getNormal() )
	{
		list<OctreeNode*> pca_nodes;
		double eigenvals[3], eigenvecs[3][3];
	
		mSceneOctree->getFullNeighbours(node, mNormalEstimationNeighRadius, pca_nodes);
		// Check if we have enough nodes for a PCA
		if ( pca_nodes.size() < 4 )
			return NULL;

		// Collect the middle points of the nodes and center them
		this->collectCenteredPoints(pca_nodes, ptsToUse);
		// Perform the real PCA
		mPCA.eigenComputations(ptsToUse, pca_nodes.size(), eigenvecs, eigenvals);
		// Switch the normal if necessary
		if ( eigenvecs[2][2] > 0.0 )
		{
			eigenvecs[0][2] = -eigenvecs[0][2];
			eigenvecs[1][2] = -eigenvecs[1][2];
			eigenvecs[2][2] = -eigenvecs[2][2];
		}
		// Save the normal
		node_data->allocNormal();
		node_data->setNormal(eigenvecs[0][2], eigenvecs[1][2], eigenvecs[2][2]);
	}

	return node_data->getNormal();
}

//================================================================================================================================

inline bool ObjRecRANSAC::significantIntersection(ORRPointSetShape* shape1, ORRPointSetShape* shape2)
{
	// Some variables for the intersection between both shapes
	vector<int> intersection(shape1->getLinearPixelIds().size() + shape2->getLinearPixelIds().size());
	// Compute the intersection set
	vector<int>::iterator intersection_it = set_intersection(
			shape1->getLinearPixelIds().begin(),
			shape1->getLinearPixelIds().end(),
			shape2->getLinearPixelIds().begin(),
			shape2->getLinearPixelIds().end(), intersection.begin());

	double frac_1 = (double)(intersection_it - intersection.begin())/(double)shape1->getNumberOfOccupiedScenePixels();
	double frac_2 = (double)(intersection_it - intersection.begin())/(double)shape2->getNumberOfOccupiedScenePixels();

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("\tintersection fractions = %lf (%s), %lf (%s) ", frac_1, shape1->getLabel(), frac_2, shape2->getLabel());
#endif

	if ( frac_1 > mIntersectionFraction || frac_2 > mIntersectionFraction )
		return true;

	return false;
}

//================================================================================================================================

inline void ObjRecRANSAC::getIdPair(const ORRPointSetShape* shape1, const ORRPointSetShape* shape2, std::pair<int,int>& id_pair)
{
	if ( shape1->getShapeId() <= shape2->getShapeId() )
	{
		id_pair.first = shape1->getShapeId();
		id_pair.second = shape2->getShapeId();
	}
	else
	{
		id_pair.first = shape2->getShapeId();
		id_pair.second = shape1->getShapeId();
	}
}

//================================================================================================================================

#endif /*OBJRECRANSAC_H_*/
