#include "ObjRecRANSAC.h"
#include <BasicTools/Aux/Stopwatch.h>
#include <vtkPointData.h>
#include <vtkIdList.h>
#include <cstdlib>
#include <pthread.h>

ObjRecRANSAC::ObjRecRANSAC(double pairwidth, double voxelsize, double relNumOfPairsInHashTable)
: mModelDatabase(pairwidth, voxelsize)
{
	mSceneOctree = NULL;

	mNormalEstimationNeighRadius = 4;
	mVisibility = 0.06;
	mRelativeNumOfIllegalPts = 0.02;
	mNumOfPointsPerLayer = 1000;
	mRelNumOfPairsToKill = 1.0 - relNumOfPairsInHashTable;
	// At least so many voxels of the scene have to belong to an object
	mRelativeObjSize = 0.05;
	mVoxelSize = voxelsize;
	mAbsZDistThresh = 1.5*voxelsize;
	mIntersectionFraction = 0.03;

	mNumOfThreads = 1;
	mNumOfCheckedHypotheses = 0;
	mUseAbsoluteObjSize = false;

	mPairWidth = pairwidth;
	mRelNumOfPairsInTheHashTable = 1.0;

	mMaxModelSupport = NULL;
	mBestShape = NULL;
	mPixelVectorPerModel = NULL;

	mMeanConfidence = 0.0;
	mICPPreProcessing = false;
	mICPPostProcessing = false;

	// Build an empty hash table
	mModelDatabase.setRelativeNumberOfPairsToKill(mRelNumOfPairsToKill);
}

ObjRecRANSAC::~ObjRecRANSAC()
{
	this->clear();
}

//=============================================================================================================================

void ObjRecRANSAC::clear()
{
	this->clear_rec();
	mModelDatabase.clear();
}

//=============================================================================================================================

void ObjRecRANSAC::clear_rec()
{
	if ( mSceneOctree )
	{
		delete mSceneOctree;
		mSceneOctree = NULL;
	}

	if ( mMaxModelSupport ){ delete[] mMaxModelSupport; mMaxModelSupport = NULL;}
	if ( mBestShape ){ delete[] mBestShape; mBestShape = NULL;}
	if ( mPixelVectorPerModel )
	{
		for ( int i = 0 ; i < mModelDatabase.getNumberOfModels() ; ++i )
			delete mPixelVectorPerModel[i];
		delete[] mPixelVectorPerModel;
		mPixelVectorPerModel = NULL;
	}

	for ( unsigned int i = 0 ; i < mShapes.size() ; ++i )
		delete mShapes[i];
	mShapes.clear();

	mModelDatabase.resetInstanceCounters();
	mSampledPairs.clear();
}

//=============================================================================================================================

void ObjRecRANSAC::printParameters(FILE* fp)
{
	fprintf(fp,
			"pair width = %lf\n"
			"voxel size = %lf\n"
			"object visibility = %lf\n"
			"relative object size = %lf\n"
			"relative number of illegal points = %lf\n"
			"abs. z dist = %lf\n"
			"intersection fraction = %lf\n"
			"normal estimation radius = %i\n",
			mPairWidth, mVoxelSize, mVisibility, mRelativeObjSize,
			mRelativeNumOfIllegalPts, mAbsZDistThresh,
			mIntersectionFraction, mNormalEstimationNeighRadius);
	fflush(fp);
}

//=============================================================================================================================

bool ObjRecRANSAC::addModel(vtkPolyData* model, UserData* userData)
{
#if defined OBJ_REC_RANSAC_VERBOSE || defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
	if ( userData )
	{
		printf("ObjRecRANSAC::%s(): Adding '%s' to the database ...\n", __func__, userData->getLabel());
		fflush(stdout);
	}
#endif

	double relNumOfPairs = 1.0;
	// Create a new entry in the data base
	bool result = mModelDatabase.addModel(model, userData, mNumOfPointsPerLayer, relNumOfPairs);

	if ( relNumOfPairs < mRelNumOfPairsInTheHashTable )
		mRelNumOfPairsInTheHashTable = relNumOfPairs;

	if ( !result )
	{
		fprintf(stderr, "ERROR in 'ObjRecRANSAC::%s()': the model will not be used for recognition.\n", __func__);
		fflush(stderr);
	}

	return result;
}

//=============================================================================================================================

bool ObjRecRANSAC::buildSceneOctree(vtkPoints* scene, double voxelsize)
{
	if ( mSceneOctree ) delete mSceneOctree;
	mSceneOctree = new ORROctree();

	// Compute a new one
	mSceneOctree->buildOctree(scene, voxelsize);

	return true;
}

//=============================================================================================================================

void ObjRecRANSAC::init_rec(vtkPoints* scene)
{
	// Clear some stuff
	this->clear_rec();

	// Initialize
	this->buildSceneOctree(scene, mVoxelSize);
	mSceneRangeImage.buildFromOctree(mSceneOctree, mAbsZDistThresh, mAbsZDistThresh);

	mMaxModelSupport = new int[mModelDatabase.getNumberOfModels()];
	mBestShape = new ORRPointSetShape*[mModelDatabase.getNumberOfModels()];
	mPixelVectorPerModel = new PixelVector*[mModelDatabase.getNumberOfModels()];
	for ( int i = 0 ; i < mModelDatabase.getNumberOfModels() ; ++i )
		mPixelVectorPerModel[i] = new PixelVector(mModelDatabase.getMaxNumberOfModelPoints());

	mMeanConfidence = 0.0;
	mNumOfCheckedHypotheses = 0;
}

//=============================================================================================================================

void ObjRecRANSAC::doRecognition(vtkPoints* scene, double successProbability,
		list<PointSetShape*>& out)
{
	Stopwatch overallStopwatch(false);
	overallStopwatch.start();

	// Do some initial cleanup and setup
	mInputScene = scene;
	this->init_rec(scene);

	// Create a scene grid for hypotheses checking
	int i;
	int** sceneGrid = new int*[mSceneRangeImage.width()];
	for ( i = 0 ; i < mSceneRangeImage.width() ; ++i )
		sceneGrid[i] = new int[mSceneRangeImage.height()];

	// Create a pixel vector for hypotheses checking
	mPixelVector = new PixelVector(mModelDatabase.getMaxNumberOfModelPoints());

	mRangeImageAccept.init(&mSceneRangeImage, sceneGrid, mPixelVector, mVisibility, mRelativeNumOfIllegalPts, mNumOfThreads, mICPPreProcessing);
	mOccupiedPixelsByShapes.clear();

	int numOfIterations = this->computeNumberOfIterations(successProbability, (int)scene->GetNumberOfPoints());

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("ObjRecRANSAC::%s(): recognizing objects [%i iteration(s)]\n",
			__func__, numOfIterations); fflush(stdout);
#endif

	OctreeNode** leaves = new OctreeNode*[numOfIterations];
	RandomGenerator randgen;
	vector<OctreeNode*>& fullLeaves = mSceneOctree->getFullLeafs();

	// Init the vector with the ids
	vector<int> ids; ids.reserve(fullLeaves.size());
	for ( i = 0 ; i < (int)fullLeaves.size() ; ++i ) ids.push_back(i);

	// Choose the leaves at random
	for ( i = 0 ; i < numOfIterations ; ++i )
	{
		// Choose a random position within the array of ids
		int rand_pos = randgen.getRandomInteger(0, ids.size() - 1);
		// Get the id at that random position
		leaves[i] = fullLeaves[ids[rand_pos]];
		// Delete the selected id
		ids.erase(ids.begin() + rand_pos);
	}

	// Start a stopwatch
//	Stopwatch pureStopwatch; pureStopwatch.start();

	// Do the real work
	this->sample(leaves, numOfIterations);

	// Filter the weak hypotheses
	list<ORRPointSetShape*> detectedShapes;
	this->gridBasedFiltering(mShapes, detectedShapes);

	printf("\nchecked hypotheses = %i\n", mNumOfCheckedHypotheses);

	// Save the shapes in 'out'
	for ( list<ORRPointSetShape*>::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
	{
		PointSetShape* shape = new PointSetShape((*it)->getUserData(), (*it)->getPolyData(), (*it)->getHomogeneousRigidTransform());
		  shape->setConfidence((*it)->getConfidence());
		  // Save the new created shape
		  out.push_back(shape);

		// Get the nodes of the current shape
		list<OctreeNode*>& nodes = (*it)->getOctreeSceneNodes();
		// For all nodes of the current shape
		for ( list<OctreeNode*>::iterator node = nodes.begin() ; node != nodes.end() ; ++node )
		{
			ORROctreeNodeData* data = (ORROctreeNodeData*)(*node)->getData();
			// Get the ids from the current node
			for ( list<int>::iterator id = data->getPointIds().begin() ; id != data->getPointIds().end() ; ++id )
				shape->getScenePointIds().push_back(*id);
		}
	}

	// Make some time measurements
//	mLastPureRecognitionTimeSec = pureStopwatch.stop();

	// Cleanup
	for ( i = 0 ; i < mSceneRangeImage.width() ; ++i )
		delete[] sceneGrid[i];
	delete[] sceneGrid;
	delete mPixelVector;
	delete[] leaves;

	mLastOverallRecognitionTimeSec = overallStopwatch.stop();
	mMeanConfidence /= (double)mNumOfCheckedHypotheses;
}

//=============================================================================================================================

void ObjRecRANSAC::sample(OctreeNode** leaves1, int numOfLeaves)
{
	OctreeNode *leaf2;
	ORROctreeNodeData *data1, *data2;
	const double *point1, *point2, *normal1, *normal2;

	// Compute the maximal number of neighbor leaves:
	int tmp = 2*mNormalEstimationNeighRadius + 1;
	int i, maxNumOfNeighbours = tmp*tmp*tmp;
	// Reserve memory for the max number of neighbors
	Matrix* pca_pts = new Matrix(3, maxNumOfNeighbours);

	for ( i = 0 ; i < numOfLeaves ; ++i )
	{
		// Get the first leaf (from the randomly sampled array of leaves)
		data1 = (ORROctreeNodeData*)leaves1[i]->getData();

		// Estimate the node normal if necessary
		normal1 = this->estimateNodeNormal(pca_pts->m, leaves1[i], data1);

		if ( !normal1 )
			continue;
		// Get the node point
		point1 = data1->getPoint();

		// Randomly select a leaf at the right distance
		leaf2 = mSceneOctree->getRandomFullLeafOnSphere(point1, mPairWidth);
		if ( !leaf2 )
			continue;
		data2 = (ORROctreeNodeData*)leaf2->getData();

		// Estimate the node normal if necessary
		normal2 = this->estimateNodeNormal(pca_pts->m, leaf2, data2);

		if ( !normal2 )
			continue;
		// Get the node point
		point2 = data2->getPoint();

		// Save the sampled point pair
		mSampledPairs.push_back(OrientedPair(point1, normal1, point2, normal2));

		// Try to detect an object
		this->generateAndTest(point1, normal1, point2, normal2);

		printf("\r%.1lf%% ", (double)(100*i)/(double)numOfLeaves); fflush(stdout);
	}
	printf("\r%.1lf%%\n", (double)(100*i)/(double)numOfLeaves); fflush(stdout);

	// Cleanup
	delete pca_pts;
}

//=============================================================================================================================

void ObjRecRANSAC::generateAndTest(const double* scenePoint1, const double* sceneNormal1,
		const double* scenePoint2, const double* sceneNormal2)
{
	double hashTableKey[3];

	// Use normals and points to compute a hash table key
	mKeyGen.computeHashTableKey3(scenePoint1, sceneNormal1, scenePoint2, sceneNormal2, hashTableKey);
	// Get the cell and its neighbors based on 'key'
	int numOfFullNeighCells = mModelDatabase.getHashTable()->getNeighbors(hashTableKey, mMaxCellNeighbors);
	// If the cell with 'key' has any full neighbors -> check them!
	if ( numOfFullNeighCells )
		this->checkObjectHypotheses(scenePoint1, sceneNormal1, scenePoint2, sceneNormal2,
				mMaxCellNeighbors, numOfFullNeighCells);

	// There is no other point/normal combination because of the way the hash table is filled:
	// both combinations are saved!
}

//=============================================================================================================================

void ObjRecRANSAC::checkObjectHypotheses(const double* scenePoint1, const double* sceneNormal1,
		const double* scenePoint2, const double* sceneNormal2, HashTableCell** cells, int numOfCells)
{
	double modelPoint1[3], modelPoint2[3], modelNormal1[3], modelNormal2[3];
	vtkPolyData* model;
	vtkDataArray* modelNormals;
	DatabaseModelEntry* dbModelEntry;
	map<DatabaseModelEntry*, HashTableCellEntry*>::const_iterator cell_entry;
	double rigid_transform[12];
	int i, support, model_id;

	for ( i = 0 ; i < mModelDatabase.getNumberOfModels() ; ++i )
	{
		mBestShape[i] = NULL;
		mMaxModelSupport[i] = 0;
	}

	// The matrix describing the rigid transform
	Matrix hommat(4, 4);

	for ( i = 0 ; i < numOfCells ; ++i )
	{
		const map<DatabaseModelEntry*, HashTableCellEntry*>& cellEntries = cells[i]->getCellEntries();
		// Check for all models in the current cell
		for ( cell_entry = cellEntries.begin() ; cell_entry != cellEntries.end() ; ++cell_entry )
		{
			dbModelEntry = (*cell_entry).first;
			model = dbModelEntry->getOwnModel();
			modelNormals = model->GetPointData()->GetNormals();
			model_id = dbModelEntry->getId();
			const list<Key*>& keys = (*cell_entry).second->getKeys();

			// Check for all pairs which belong to the current model
			for ( list<Key*>::const_iterator key = keys.begin() ; key != keys.end() ; ++key )
			{
				// Get the points and the normals from the model
				model->GetPoint((*key)->getPointId1(), modelPoint1);
				modelNormals->GetTuple((*key)->getPointId1(), modelNormal1);
				model->GetPoint((*key)->getPointId2(), modelPoint2);
				modelNormals->GetTuple((*key)->getPointId2(), modelNormal2);

				// Get the optimal rigid transform from model to scene
				mOptTransform.getRigidTransform(modelPoint1, modelNormal1, modelPoint2, modelNormal2,
						scenePoint1, sceneNormal1, scenePoint2, sceneNormal2, rigid_transform);

				// This counter is very important, since it is also used as a uniquie id for the
				// current object hypothesis
				++mNumOfCheckedHypotheses;

				// Check the hypothesis
				support = mRangeImageAccept.accept(rigid_transform, dbModelEntry->getLayeredModel(), mNumOfCheckedHypotheses);
				if ( support > mMaxModelSupport[model_id] )
				{
					mMaxModelSupport[model_id] = support;
					Matrix::copyRigidTransformTo4x4(rigid_transform, hommat.m);

					if ( mBestShape[model_id] == NULL )
					{
						mBestShape[model_id] = new ORRPointSetShape(dbModelEntry->getUserData(), dbModelEntry->getOwnModel(),
								dbModelEntry->getLayeredModel(),
								(const double**)hommat.m, dbModelEntry->getInstanceCounter());
						dbModelEntry->incrementInstanceCounter();
						mBestShape[model_id]->setSceneStateOn();
						mBestShape[model_id]->setRigidTransform12(rigid_transform);
					}
					else
						mBestShape[model_id]->setHomogeneousRigidTransform(hommat.getm());
					// Set some other stuff ('mPixelVector' contains the pixels "explained" by the current shape hypothesis)
					mBestShape[model_id]->setOccupiedScenePixels(&mSceneRangeImage, mPixelVector);
					mPixelVectorPerModel[model_id]->copyfrom(*mPixelVector);
					mBestShape[model_id]->setPoint1(modelPoint1);
					mBestShape[model_id]->setNormal1(modelNormal1);
					mBestShape[model_id]->setPoint2(modelPoint2);
					mBestShape[model_id]->setNormal2(modelNormal2);
					mBestShape[model_id]->setConfidence((double)support/(double)model->GetNumberOfPoints());
				}

				mMeanConfidence += (double)support/(double)model->GetNumberOfPoints();
			}
		}
	}

	for ( i = 0 ; i < mModelDatabase.getNumberOfModels() ; ++i )
	{
		if ( mBestShape[i] )
		{
			if ( mICPPostProcessing )
			{
				mRangeImageAccept.icp(mBestShape[i]->getLayeredPointSet(), &mSceneRangeImage, mBestShape[i]->getRigidTransform12());
				mBestShape[i]->copyRigidTransformFrom12To4x4();
			}

//			printf("."); fflush(stdout);
			// Create a new shape and save it in the vector of shapes
			int currentShapeId = (int)mShapes.size();
			// Save the best shape
			mShapes.push_back(mBestShape[i]);
			mBestShape[i]->sortLinearPixelIds();
			mBestShape[i]->setShapeId(currentShapeId);

			int j, x, y;
			for ( j = 0 ; j < mPixelVectorPerModel[i]->getNumberOfOccupiedPixels() ; ++j )
			{
				// The range image pixel [x, y] is occupied by 'mBestShape[i]'
				x = mPixelVectorPerModel[i]->getPixelCoords()[j].x;
				y = mPixelVectorPerModel[i]->getPixelCoords()[j].y;

				// Check whether the current scene pixel is occupied
				if ( mSceneRangeImage.getShapesGrid()[x][y] == NULL )
				{
					// Create a new occupancy pixel and save it
					mSceneRangeImage.getShapesGrid()[x][y] = new list<int>;
					mOccupiedPixelsByShapes.push_back(mSceneRangeImage.getShapesGrid()[x][y]);
				}
				// Mark the current range image pixel as occupied by the current hypothesis
				mSceneRangeImage.getShapesGrid()[x][y]->push_back(currentShapeId);
			}
		}
	}
}

//=============================================================================================================================

bool cmp_int_pairs(std::pair<int,int> p1, std::pair<int,int> p2)
{
	if ( p1.first == p2.first )
		return p1.second < p2.second;

	return p1.first < p2.first;
}

//=============================================================================================================================

void ObjRecRANSAC::gridBasedFiltering(vector<ORRPointSetShape*>& shapes, list<ORRPointSetShape*>& out)
{
	list<list<int>* >::iterator pixel;
	list<ORRPointSetShape*> pixel_shapes;
	list<int>::iterator id;
	set<std::pair<int,int>, bool(*)(std::pair<int,int>, std::pair<int,int>)> both_on(cmp_int_pairs);
	std::pair<int,int> on_pair;

#ifdef OBJ_REC_RANSAC_VERBOSE
	printf("ObjRecRANSAC::%s(): We have %i shapes to filter.\n", __func__, (int)mShapes.size());
#endif

	for ( pixel = mOccupiedPixelsByShapes.begin() ; pixel != mOccupiedPixelsByShapes.end() ; ++pixel )
	{
		pixel_shapes.clear();

		// Save all ON shapes in 'pixel_shapes'
		for ( id = (*pixel)->begin() ; id != (*pixel)->end() ; ++id )
			if ( shapes[*id]->isSceneStateOn() )
				pixel_shapes.push_back(shapes[*id]);

		// All against all
		for ( list<ORRPointSetShape*>::iterator it1 = pixel_shapes.begin() ; it1 != pixel_shapes.end() ; ++it1 )
		{
			if ( (*it1)->isSceneStateOff() )
				continue;

			list<ORRPointSetShape*>::iterator it2 = it1;
			for ( ++it2 ; it2 != pixel_shapes.end() && (*it1)->isSceneStateOn() ; ++it2 )
			{
				// Both 'it1' and 'it2' are ON -> check if they are not in the list of checked shapes
				this->getIdPair(*it1, *it2, on_pair);
				if ( both_on.find(on_pair) != both_on.end() )
					continue; // These shapes are already checked -> go to the next pair

				if ( this->significantIntersection(*it1, *it2) )
				{
					if ( (*it1)->getNumberOfOccupiedScenePixels() <= (*it2)->getNumberOfOccupiedScenePixels() )
					{
#ifdef OBJ_REC_RANSAC_VERBOSE
						printf("[turn off %s]\n", (*it1)->getLabel());
#endif
						(*it1)->setSceneStateOff();
					}
					else
					{
#ifdef OBJ_REC_RANSAC_VERBOSE
						printf("[turn off %s]\n", (*it2)->getLabel());
#endif
						(*it2)->setSceneStateOff();
					}
				}
				else
				{
#ifdef OBJ_REC_RANSAC_VERBOSE
					printf("[both on]\n");
#endif
					both_on.insert(on_pair);
				}
			}
		}
	}

	// The shapes that are still ON are the ones we want
	for ( vector<ORRPointSetShape*>::iterator shape = shapes.begin() ; shape != shapes.end() ; ++shape )
		if ( (*shape)->isSceneStateOn() )
			out.push_back(*shape);
}

//====================================================================================================================
