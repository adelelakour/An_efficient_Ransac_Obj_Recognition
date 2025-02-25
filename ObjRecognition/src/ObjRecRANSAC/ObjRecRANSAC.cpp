#include <opencv2/opencv.hpp>
#include "ObjRecRANSAC.h"
#include "Algorithms/ObjRecICP.h"
#include "DataStructures/ThreadInfo.h"
#include <BasicTools/Aux/Stopwatch.h>
#include <vtkPointData.h>
#include <vtkIdList.h>
#include <cstdlib>
//#include <ippcore.h>
#include <pthread.h>
#include <random>
#include<bits/stdc++.h>
#include<list>

ObjRecRANSAC::ObjRecRANSAC(double pairwidth, double voxelsize, double relNumOfPairsInHashTable)
        : mModelDatabase(pairwidth, voxelsize) {
    mSceneOctree = NULL;

    mNormalEstimationNeighRadius = 4;
    mVisibility = 0.06;
    mRelativeNumOfIllegalPts = 0.02;
    mNumOfPointsPerLayer = 1000;
    mRelNumOfPairsToKill = 1.0 - relNumOfPairsInHashTable;
    // At least so many voxels of the scene have to belong to an object
    mRelativeObjSize = 0.05;
    mVoxelSize = voxelsize;
    mAbsZDistThresh = 1.5 * voxelsize;
    mIntersectionFraction = 0.05;

    mNumOfThreads = 1;
    mNumOfHypotheses = 0;
    mUseAbsoluteObjSize = false;

    mPairWidth = pairwidth;
    mRelNumOfPairsInTheHashTable = 1.0;

    mRigidTransforms = NULL;
    mPointSetPointers = NULL;
    mPairIds = NULL;
    mModelEntryPointers = NULL;
    mICPRefinement = true;

    // Build an empty hash table
    mModelDatabase.setRelativeNumberOfPairsToKill(mRelNumOfPairsToKill);
}

ObjRecRANSAC::~ObjRecRANSAC() {
    this->clear();
}

//=============================================================================================================================

void ObjRecRANSAC::clear() {
    this->clear_rec();
    mModelDatabase.clear();
}

//=============================================================================================================================

void ObjRecRANSAC::clear_rec() {
    if (mSceneOctree) {
        delete mSceneOctree;
        mSceneOctree = NULL;
    }

    if (mRigidTransforms) {
        delete[] mRigidTransforms;
        mRigidTransforms = NULL;
    }
    if (mPointSetPointers) {
        delete[] mPointSetPointers;
        mPointSetPointers = NULL;
    }
    if (mPairIds) {
        delete[] mPairIds;
        mPairIds = NULL;
    }
    if (mModelEntryPointers) {
        delete[] mModelEntryPointers;
        mModelEntryPointers = NULL;
    }

    for (unsigned int i = 0; i < mShapes.size(); ++i)
        delete mShapes[i];
    mShapes.clear();

    for (list<Hypothesis *>::iterator it = mHypotheses.begin(); it != mHypotheses.end(); ++it)
        delete *it;
    mHypotheses.clear();

    mModelDatabase.resetInstanceCounters();
    mSampledPairs.clear();
}

//=============================================================================================================================

void ObjRecRANSAC::printParameters(FILE *fp) const {
    fprintf(fp,
            "pair width = %lf\n"
            "relative number of pairs to remove from hash table = %lf\n"
            "voxel size = %lf\n"
            "object visibility = %lf\n"
            "relative object size = %lf\n"
            "relative number of illegal points = %lf\n"
            "abs. z dist = %lf\n"
            "intersection fraction = %lf\n"
            "normal estimation radius = %i\n",
            mPairWidth, mRelNumOfPairsToKill, mVoxelSize, mVisibility, mRelativeObjSize,
            mRelativeNumOfIllegalPts, mAbsZDistThresh,
            mIntersectionFraction, mNormalEstimationNeighRadius);
    fflush(fp);
}

//=============================================================================================================================

bool ObjRecRANSAC::addModel(vtkPolyData *model, UserData *userData) {
#if defined OBJ_REC_RANSAC_VERBOSE || defined OBJ_REC_RANSAC_VERBOSE_1 || defined OBJ_REC_RANSAC_VERBOSE_2
    if (userData) {
        printf("ObjRecRANSAC::%s(): Adding '%s' to the database ...\n", __func__, userData->getLabel());
        fflush(stdout);
    }
#endif

    double relNumOfPairs = 1.0;
    // Create a new entry in the data base
    bool result = mModelDatabase.addModel(model, userData, mNumOfPointsPerLayer, relNumOfPairs);

    if (relNumOfPairs < mRelNumOfPairsInTheHashTable)
        mRelNumOfPairsInTheHashTable = relNumOfPairs;

    if (!result) {
        fprintf(stderr, "ERROR in 'ObjRecRANSAC::%s()': the model will not be used for recognition.\n", __func__);
        fflush(stderr);
    }

    return result;
}

//=============================================================================================================================

bool ObjRecRANSAC::buildSceneOctree(vtkPoints *scene, double voxelsize) {
    delete mSceneOctree;
    mSceneOctree = new ORROctree();

    // Compute a new one
    mSceneOctree->buildOctree(scene, voxelsize);

    cout << "this is the size of constructed OCTREE " << mSceneOctree->getNumberOfFullLeafs() << endl;
    return true;
}

//=============================================================================================================================

void ObjRecRANSAC::init_rec(vtkPoints *scene) {
    // Clear some stuff
    this->clear_rec();

    // Initialize
    this->buildSceneOctree(scene, mVoxelSize);
    mSceneRangeImage.buildFromOctree(mSceneOctree, mAbsZDistThresh, mAbsZDistThresh);

    mNumOfHypotheses = 0;
}

//############################################

bool compareOctreeNodesByCenter(const OctreeNode* a, const OctreeNode* b) {
    // Retrieve center coordinates of the nodes
    double centerA[3], centerB[3];
    centerA[0] = a->getCenter()[0];
    centerA[1] = a->getCenter()[1];
    centerA[2] = a->getCenter()[2];

    centerB[0] = b->getCenter()[0];
    centerB[1] = b->getCenter()[1];
    centerB[2] = b->getCenter()[2];

    // Compare center coordinates lexicographically
    for (int i = 0; i < 3; ++i) {
        if (centerA[i] != centerB[i]) {
            return centerA[i] < centerB[i];
        }
    }
    return false; // Nodes have the same center
}


//*******************
//*********
double distance(const double p1[3], const double p2[3]) {
    return std::sqrt(std::pow(p1[0] - p2[0], 2) +
                     std::pow(p1[1] - p2[1], 2) +
                     std::pow(p1[2] - p2[2], 2));
}
//=============================================================================================================================
void ObjRecRANSAC::doRecognition(vtkPoints *scene, double successProbability, list<PointSetShape *> &out) {


    auto startTime = std::chrono::steady_clock::now();

    if (scene->GetNumberOfPoints() <= 0)
        return;

    Stopwatch overallStopwatch(false);
    overallStopwatch.start();

    // Do some initial cleanup and setup
    mInputScene = scene;
    this->init_rec(scene);
    mOccupiedPixelsByShapes.clear();

    int i, numOfIterations = this->computeNumberOfIterations(successProbability, (int) scene->GetNumberOfPoints());
    vector<OctreeNode *> &fullLeaves = mSceneOctree->getFullLeafs();
    cout << "fullLeaves.size " << fullLeaves.size() << endl;      // fixed, confirmed

    // numOfIterations is fixed, confirmed

  /*  I sorted fullLeaves according to the centroid of each voxel
    */
    std::sort(fullLeaves.begin(), fullLeaves.end(), compareOctreeNodesByCenter);

    if ((int) fullLeaves.size() < numOfIterations)
        numOfIterations = (int) fullLeaves.size();

#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("ObjRecRANSAC::%s(): recognizing objects [%i iteration(s), %i thread(s)]\n",
           __func__, numOfIterations, mNumOfThreads);
    fflush(stdout);
#endif

    OctreeNode **leaves = new OctreeNode *[numOfIterations];  // leaves here means sampled N leaves

    RandomGenerator randgen;

    // he created a vector of indecies
    vector<int> ids;
    ids.reserve(fullLeaves.size());
    for (i = 0; i < (int) fullLeaves.size(); ++i) ids.push_back(i);   // we filled the vector with all possible ids


    std::mt19937 rng(1234); // You can use any integer value here

    // Define your distribution
    int min = 0;
    int max = fullLeaves.size() - 1;
    std::uniform_int_distribution<int> dist;


    // Sample the leaves at random (seeded)
    for (i = 0; i < numOfIterations; ++i) {
        // Choose a random position within the array of ids
        int rand_pos = dist(rng, decltype(dist)::param_type{min, max});   // consistent return (confirmed)
        //cout << "rand_pose is ... " << rand_pos << endl;
        //cout << "rand_pose : " << rand_pos << endl;
        // Get the id at that random position
        leaves[i] = fullLeaves[ids[rand_pos]];
        //cout << "leaves[i] : " << *leaves[i]->getCenter() <<endl;
        // Delete the selected id
        //ids.erase(ids.begin() + rand_pos);
    }


    // Sample the oriented point pairs
    this->sampleOrientedPointPairs(leaves, numOfIterations, mSampledPairs);




    // Generate the object hypotheses
    this->generateHypotheses(mSampledPairs);  //# generated Hypotheses are fixed

    cout << "number of sampled pairs : " << mSampledPairs.size() << endl; // added by me

    //✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓
    // So far, I get the same (rand_pos), the same (leaves[i]) , sampled Pairs, generateHypotheses  at every run ✓
    //✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓

    list<AcceptedHypothesis> accepted_hypotheses;

    // Accept hypotheses
    this->acceptHypotheses(accepted_hypotheses);

    // written by me
/*     while(accepted_hypotheses.size() >1)
     {
         accepted_hypotheses.pop_front();
     }*/

  /* for (auto hypo : accepted_hypotheses)
    {
        auto iter = accepted_hypotheses.begin();
        iter ++;
        accepted_hypotheses.erase(iter, accepted_hypotheses.end());
    }*/

    // Convert the accepted hypotheses to shapes
    this->hypotheses2Shapes(accepted_hypotheses, mShapes);

    // Filter the weak hypotheses
    list<ORRPointSetShape *> detectedShapes;
    this->gridBasedFiltering(mShapes, detectedShapes);

    //write by me
    //AcceptedHypothesis best_hypothesis = accepted_hypotheses.begin()->rigid_transform;

    for (auto shape : detectedShapes)
    {
        cout << shape->getLabel() << endl;
    }
    // Save the shapes in 'out'
    for (list<ORRPointSetShape *>::iterator it = detectedShapes.begin(); it != detectedShapes.end(); ++it) {
        PointSetShape *shape = new PointSetShape(*it);
        // Save the new created shape
        out.push_back(shape);

        // Get the nodes of the current shape
        list<OctreeNode *> &nodes = (*it)->getOctreeSceneNodes();
        // For all nodes of the current shape
        for (list<OctreeNode *>::iterator node = nodes.begin(); node != nodes.end(); ++node) {
            ORROctreeNodeData *data = (ORROctreeNodeData *) (*node)->getData();
            // Get the ids from the current node
            for (list<int>::iterator id = data->getPointIds().begin(); id != data->getPointIds().end(); ++id)
                shape->getScenePointIds().push_back(*id);
        }
    }

    // Cleanup
    delete[] leaves;
    accepted_hypotheses.clear();

    if (mICPRefinement) {
        ObjRecICP objrec_icp;
        objrec_icp.setEpsilon(1.5);
        objrec_icp.doICP(*this, out);
    }

    mLastOverallRecognitionTimeSec = overallStopwatch.stop();

    auto endTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> recognitionDuration = endTime - startTime;
    cout << "It took " << recognitionDuration.count() << " Second to recognize the objects " << endl;

}


//=============================================================================================================================

void ObjRecRANSAC::sampleOrientedPointPairs(OctreeNode **leaves1, int numOfLeaves, list<OrientedPair> &pairs) {

    srand(123);
    OctreeNode *leaf2;
    ORROctreeNodeData *data1, *data2;
    const double *point1, *point2, *normal1, *normal2;

    // Compute the maximal number of neighbor leaves:
    int tmp = 2 * mNormalEstimationNeighRadius + 1;
    int i, maxNumOfNeighbours = tmp * tmp * tmp;
    // Reserve memory for the max number of neighbors
    double **pca_pts = mat_alloc(3, maxNumOfNeighbours);

#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("ObjRecRANSAC::%s(): sample pairs ... ", __func__);
    fflush(stdout);
#endif


    for (i = 0; i < numOfLeaves; ++i) {
        // Get the first leaf (from the randomly sampled array of leaves)
        data1 = (ORROctreeNodeData *) leaves1[i]->getData();

        // Estimate the node normal if necessary
        normal1 = this->estimateNodeNormal(pca_pts, leaves1[i], data1);

        if (!normal1)
            continue;
        // Get the node point
        point1 = data1->getPoint();

        // Randomly select a leaf at the right distance

        leaf2 = mSceneOctree->getRandomFullLeafOnSphere(point1, mPairWidth);
        if (!leaf2)
            continue;
        data2 = (ORROctreeNodeData *) leaf2->getData();

        // Estimate the node normal if necessary
        normal2 = this->estimateNodeNormal(pca_pts, leaf2, data2);

        if (!normal2)
            continue;
        // Get the node point
        point2 = data2->getPoint();

        //cout << "point 1 is : " << *point1 << " & point 2 is : " << *point2 << endl;
        // Save the sampled point pair
        pairs.push_back(OrientedPair(point1, normal1, point2, normal2));

        pairs.sort([](const OrientedPair &a, const OrientedPair &b) {
            return distance(a.p1, a.p2) < distance(b.p1, b.p2);
        });

        // I sample the same pairs at every run (confirmed)

    }

#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("done.\n");
    fflush(stdout);
#endif

    // Cleanup
    mat_dealloc(pca_pts, 3);
}

//=============================================================================================================================

void ObjRecRANSAC::generateHypotheses(const list<OrientedPair> &pairs) {
    // Only for 3D hash tables: this is the max number of neighbors a 3D hash table cell can have!
    HashTableCell *neigh_cells[27];
    double hashTableKey[3];
    HashTableKeyGenerator key_gen;
    list<OrientedPair>::const_iterator pair;
    int i = 0;

#ifdef OBJ_REC_RANSAC_VERBOSE
    double factor = 100.0 / (double) pairs.size();
    printf("ObjRecRANSAC::%s(): generate hypotheses ...\n", __func__);
    fflush(stdout);
#endif

    for (i = 0, pair = pairs.begin(); pair != pairs.end(); ++i, ++pair) {
        // Use normals and points to compute a hash table key
        key_gen.computeHashTableKey3((*pair).p1, (*pair).n1, (*pair).p2, (*pair).n2, hashTableKey);

        //### p1, n1, p2, n2  are fixed at every trial

        // Get the cell and its neighbors based on 'key'
        int numOfFullNeighCells = mModelDatabase.getHashTable()->getNeighbors(hashTableKey, neigh_cells);
        // If the cell with 'key' has any full neighbors -> check them!
        if (numOfFullNeighCells)
            this->generateHypothesesForPair((*pair).p1, (*pair).n1, (*pair).p2, (*pair).n2,
                                            neigh_cells, numOfFullNeighCells, i);
        // (generateHypothesesForPair) returns fixed rigid transformations

#ifdef OBJ_REC_RANSAC_VERBOSE
        printf("\r%.1lf%% ", ((double) (i + 1)) * factor);
        fflush(stdout);
#endif
    }
#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("\r%.1lf%% done [%i hypotheses]\n", ((double) i) * factor, mNumOfHypotheses);
    fflush(stdout);
#endif

    mRigidTransforms = new double[12 * mNumOfHypotheses];
    mPointSetPointers = new const double *[mNumOfHypotheses];
    mPairIds = new int[mNumOfHypotheses];
    cout << "number of Hypotheses is : " << mNumOfHypotheses << endl;
    mModelEntryPointers = new DatabaseModelEntry *[mNumOfHypotheses];

    double *rigid_transform = mRigidTransforms;
    list<Hypothesis *>::iterator hypo;

    for (i = 0, hypo = mHypotheses.begin(); hypo != mHypotheses.end(); ++i, ++hypo, rigid_transform += 12) {
        vec_copy12<double>((*hypo)->rigid_transform, rigid_transform);
        mPointSetPointers[i] = (*hypo)->model_entry->getOwnPointSet()->getPoints_const();
        mPairIds[i] = (*hypo)->pair_id;
        mModelEntryPointers[i] = (*hypo)->model_entry;
    }
}

//=============================================================================================================================

void ObjRecRANSAC::generateHypothesesForPair(const double *scenePoint1, const double *sceneNormal1,
                                             const double *scenePoint2, const double *sceneNormal2,
                                             HashTableCell **cells,
                                             int numOfCells, int pair_id) {
    double modelPoint1[3], modelPoint2[3], modelNormal1[3], modelNormal2[3];
    vtkPolyData *model;
    vtkDataArray *modelNormals;
    DatabaseModelEntry *dbModelEntry;
    map<DatabaseModelEntry *, HashTableCellEntry *>::const_iterator cell_entry;
    int i, model_id;

    for (i = 0; i < numOfCells; ++i) {
        const map<DatabaseModelEntry *, HashTableCellEntry *> &cellEntries = cells[i]->getCellEntries();
        // Check for all models in the current cell
        for (cell_entry = cellEntries.begin(); cell_entry != cellEntries.end(); ++cell_entry) {
            dbModelEntry = (*cell_entry).first;
            model = dbModelEntry->getOwnModel();
            modelNormals = model->GetPointData()->GetNormals();
            model_id = dbModelEntry->getId();
            const list<Key *> &keys = (*cell_entry).second->getKeys();

            // Check for all pairs which belong to the current model
            for (list<Key *>::const_iterator key = keys.begin(); key != keys.end(); ++key) {
                // Get the points and the normals from the model
                model->GetPoint((*key)->getPointId1(), modelPoint1);
                modelNormals->GetTuple((*key)->getPointId1(), modelNormal1);
                model->GetPoint((*key)->getPointId2(), modelPoint2);
                modelNormals->GetTuple((*key)->getPointId2(), modelNormal2);

                // Get the optimal rigid transform from model to scene
                double *rigid_transform = new double[12];
                mOptTransform.getRigidTransform(modelPoint1, modelNormal1, modelPoint2, modelNormal2,
                                                scenePoint1, sceneNormal1, scenePoint2, sceneNormal2, rigid_transform);
                // returned (getRigidTransform) is fixed

                ++mNumOfHypotheses;
                // Save the current object hypothesis
                mHypotheses.push_back(new Hypothesis(rigid_transform, pair_id, dbModelEntry));
            }
        }
    }
}

//=============================================================================================================================

int gMatchThresh, gPenaltyThresh;
const ORRRangeImage2 *gImage;

void *accept(void *data) {
    GeometryProcessor geom_processor;
    ThreadInfo *info = (ThreadInfo *) data;
    double out[3], m_0[3], s_0[3], C[9], Ncc[9];
    double *transform = info->transforms;
    const double *mp, **model_points = info->model_points;
    int i, k, x, y, match, penalty, num_transforms = info->num_transforms;
    const int *pair_id = info->pair_ids;
    const double_2 *pixel;

    // For all hypotheses
    for (i = 0; i < num_transforms; ++i, transform += 12) {
        one_icp_iteration(model_points[i], ORR_NUM_OF_OWN_MODEL_POINTS, &geom_processor,
                          gImage, transform, C, Ncc, m_0, s_0);

        // Some initializations for the second loop (the match/penalty loop)
        mp = model_points[i];
        match = penalty = 0;

        // The match/penalty loop
        for (k = 0; k < ORR_NUM_OF_OWN_MODEL_POINTS; ++k, mp += 3) {
            // Transform the model point with the current rigid transform
            mat_mult3_by_rigid<double>(transform, mp, out);

            // Get the pixel the point 'out' lies in
            pixel = gImage->getSafePixel(out[0], out[1], x, y);
            // Check if we have a valid pixel
            if (pixel == NULL)
                continue;

            if (out[2] < pixel->x) // The transformed model point overshadows a pixel -> penalize it.
                ++penalty;
            else if (out[2] <= pixel->y) // The point is OK.
                ++match;
        }

        // Check if we should accept this hypothesis
        if (match >= gMatchThresh && penalty <= gPenaltyThresh) {
            // Is it better than the one we already have for 'pair_id[i]'
            if (match > info->pair_result[pair_id[i]].match) {
                info->pair_result[pair_id[i]].match = match;
                info->pair_result[pair_id[i]].model_entry = info->model_entries[i];
                info->pair_result[pair_id[i]].transform = transform;
            }
        }
    }

    return NULL;
}

//=============================================================================================================================

void ObjRecRANSAC::acceptHypotheses(list<AcceptedHypothesis> &acceptedHypotheses) {
#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("ObjRecRANSAC::%s(): checking the hypotheses ... ", __func__);
    fflush(stdout);
#endif

    double *rigid_transform = mRigidTransforms;
    const double **model_points = mPointSetPointers;
    DatabaseModelEntry **model_entries = mModelEntryPointers;
    const int *pair_ids = mPairIds;
    cout << "*pair_ids is .. " << *pair_ids << endl;
    int i, numOfTransforms = mNumOfHypotheses / mNumOfThreads, num_of_pairs = (int) mSampledPairs.size();


    // Stuff for the threads
    ThreadInfo *thread_info = new ThreadInfo[mNumOfThreads];
    pthread_t *threads = new pthread_t[mNumOfThreads];

    // Initialize some global variables
    gImage = &mSceneRangeImage;
    gMatchThresh = (int) ((double) ORR_NUM_OF_OWN_MODEL_POINTS * mVisibility + 0.5);
    gPenaltyThresh = (int) ((double) ORR_NUM_OF_OWN_MODEL_POINTS * mRelativeNumOfIllegalPts + 0.5);

    for (i = 0; i < mNumOfThreads - 1; ++i) {
        thread_info[i].num_transforms = numOfTransforms;
        thread_info[i].transforms = rigid_transform;
        thread_info[i].model_points = model_points;
        thread_info[i].model_entries = model_entries;
        thread_info[i].pair_ids = pair_ids;
        thread_info[i].pair_result = new ThreadResult[num_of_pairs];
        // Create the thread and let him do its job
        pthread_create(&threads[i], NULL, accept, (void *) &thread_info[i]);

        // Increment some pointers
        rigid_transform += 12 * numOfTransforms;
        model_points += numOfTransforms;
        model_entries += numOfTransforms;
        pair_ids += numOfTransforms;
    }

    // For the last thread
    thread_info[i].num_transforms = numOfTransforms + mNumOfHypotheses % mNumOfThreads;
    thread_info[i].transforms = rigid_transform;
    thread_info[i].model_points = model_points;
    thread_info[i].model_entries = model_entries;
    thread_info[i].pair_ids = pair_ids;
    thread_info[i].pair_result = new ThreadResult[num_of_pairs];
    // Create the last thread and let it work
    pthread_create(&threads[i], NULL, accept, (void *) &thread_info[i]);

    // Wait for all threads
    for (i = 0; i < mNumOfThreads; ++i)
        pthread_join(threads[i], NULL);

    // Some variables needed for the hypothesis acceptance
    AcceptedHypothesis accepted;
    int k;

    // For all pairs

    for (i = 0; i < num_of_pairs; ++i) {
        accepted.match = 0;

        // Get the best match for the i-th pair from all threads
        for (k = 0; k < mNumOfThreads; ++k) {
            // Check the result of this thread for the i-th pair
            if (thread_info[k].pair_result[i].match > accepted.match) {
                accepted.model_entry = thread_info[k].pair_result[i].model_entry;
                accepted.match = thread_info[k].pair_result[i].match;
                accepted.rigid_transform = thread_info[k].pair_result[i].transform;
            }
        }
        if (accepted.match > 0)
            acceptedHypotheses.push_back(accepted);

    }

#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("%i accepted.\n", (int) acceptedHypotheses.size());
    fflush(stdout);
#endif

    // Cleanup
    delete[] thread_info;
    delete[] threads;

    cout << " acceptedHypotheses SIZE : " << acceptedHypotheses.size() << endl;

    for (auto hypo : acceptedHypotheses)
    {
        cout << "Hypo : \n" << hypo.rigid_transform[0] << " , " << hypo.rigid_transform[1] << " , "
                << hypo.rigid_transform[2] << " , " << hypo.rigid_transform[3] << " , "
                << hypo.rigid_transform[4] << " , " << hypo.rigid_transform[5] << " , "
                << hypo.rigid_transform[6] << " , " << hypo.rigid_transform[7] << " , "
                << hypo.rigid_transform[8] << " , " << hypo.rigid_transform[9] << " , "
                << hypo.rigid_transform[10] << " , " << hypo.rigid_transform[11] << endl;
        cout << "Hypo match score : " << hypo.match << endl;
    }
    cout << "____________________________________ " << endl;

}

//=============================================================================================================================

void ObjRecRANSAC::hypotheses2Shapes(list<AcceptedHypothesis> &hypotheses, vector<ORRPointSetShape *> &shapes) {
    GeometryProcessor geom_processor;
    const double *mp;
    double p[3], *rigid_transform;
    int i, x, y, numOfPoints, support, shape_id, hypo_id = 1;
    const double_2 *pixel;
    ORRPointSetShape *shape;
    DatabaseModelEntry *model_entry;

    // For the shapes
    int **scene_grid = new int *[mSceneRangeImage.width()];
    for (i = 0; i < mSceneRangeImage.width(); ++i) {
        scene_grid[i] = new int[mSceneRangeImage.height()];
        memset(scene_grid[i], 0, mSceneRangeImage.height() * sizeof(int));
    }

    // Convert each hypothesis in 'hypotheses' to a shape and save it in 'shapes'
    for (list<AcceptedHypothesis>::iterator hypo = hypotheses.begin(); hypo != hypotheses.end(); ++hypo, ++hypo_id) {
        // Some initializations
        mp = (*hypo).model_entry->getOwnPointSet()->getPoints_const();
        numOfPoints = (*hypo).model_entry->getOwnPointSet()->getNumberOfPoints();
        rigid_transform = (*hypo).rigid_transform;
        model_entry = (*hypo).model_entry;
        shape = NULL;
        support = 0;
        // Get the current shape id
        shape_id = (int) shapes.size();

        for (i = 0; i < numOfPoints; ++i, mp += 3) {
            // Transform the current model point
            mat_mult3_by_rigid<double>(rigid_transform, mp, p);
            // Get the pixel corresponding to the transformed point
            pixel = mSceneRangeImage.getSafePixel(p[0], p[1], x, y);
            // Check if we have a valid pixel
            if (pixel == NULL)
                continue;

            // Check if the pixel is OK
            if (pixel->x <= p[2] && p[2] <= pixel->y) {
                ++support;

                if (shape == NULL) {
                    // Create a new shape
                    shape = new ORRPointSetShape(model_entry->getUserData(), model_entry->getOwnModel(),
                                                 rigid_transform, model_entry->getHighResModel(),
                                                 model_entry->getInstanceCounter());
                    shape->setShapeId(shape_id);
                    shape->setSceneStateOn();

                    // Update some things
                    model_entry->incrementInstanceCounter();
                }

                // Check if the pixel [x, y] is already occupied by the current shape
                if (scene_grid[x][y] != hypo_id) {
                    scene_grid[x][y] = hypo_id;
                    // Add the pixel [x, y] to the current shape
                    shape->addPixel(&mSceneRangeImage, x, y);

                    // Check whether the current scene pixel is occupied
                    if (mSceneRangeImage.getShapesGrid()[x][y] == NULL) {
                        // Create a new occupancy pixel and save it
                        mSceneRangeImage.getShapesGrid()[x][y] = new list<int>;
                        mOccupiedPixelsByShapes.push_back(mSceneRangeImage.getShapesGrid()[x][y]);
                    }
                    // Mark the current range image pixel as occupied by the current hypothesis
                    mSceneRangeImage.getShapesGrid()[x][y]->push_back(shape_id);
                }
            }
        }

        // Save the shape in the 'shapes' list
        if (shape) {
            shape->sortLinearPixelIds();
            shape->setConfidence((double) support / (double) model_entry->getOwnPointSet()->getNumberOfPoints());
            shapes.push_back(shape);
        }
    }

    // Cleanup
    for (i = 0; i < mSceneRangeImage.width(); ++i)
        delete[] scene_grid[i];
    delete[] scene_grid;
}

//=============================================================================================================================

bool cmp_int_pairs(std::pair<int, int> p1, std::pair<int, int> p2) {
    if (p1.first == p2.first)
        return p1.second < p2.second;

    return p1.first < p2.first;
}

//=============================================================================================================================

void ObjRecRANSAC::gridBasedFiltering(vector<ORRPointSetShape *> &shapes, list<ORRPointSetShape *> &out) {
    list<list<int> *>::iterator pixel;
    list<ORRPointSetShape *> pixel_shapes;
    list<int>::iterator id;
    set<std::pair<int, int>, bool (*)(std::pair<int, int>, std::pair<int, int>)> both_on(cmp_int_pairs);
    std::pair<int, int> on_pair;

#ifdef OBJ_REC_RANSAC_VERBOSE
    printf("ObjRecRANSAC::%s(): We have %i shapes to filter.\n", __func__, (int) shapes.size());
#endif

    for (pixel = mOccupiedPixelsByShapes.begin(); pixel != mOccupiedPixelsByShapes.end(); ++pixel) {
        pixel_shapes.clear();

        // Save all ON shapes in 'pixel_shapes'
        for (id = (*pixel)->begin(); id != (*pixel)->end(); ++id)
            if (shapes[*id]->isSceneStateOn())
                pixel_shapes.push_back(shapes[*id]);

        // All against all
        for (list<ORRPointSetShape *>::iterator it1 = pixel_shapes.begin(); it1 != pixel_shapes.end(); ++it1) {
            if ((*it1)->isSceneStateOff())
                continue;

            list<ORRPointSetShape *>::iterator it2 = it1;
            for (++it2; it2 != pixel_shapes.end() && (*it1)->isSceneStateOn(); ++it2) {
                // Both 'it1' and 'it2' are ON -> check if they are not in the list of already checked shapes
                this->getIdPair(*it1, *it2, on_pair);
                if (both_on.find(on_pair) != both_on.end())
                    continue; // These shapes are already checked -> go to the next pair

                if (this->significantIntersection(*it1, *it2)) {
                    if ((*it1)->getNumberOfOccupiedScenePixels() <= (*it2)->getNumberOfOccupiedScenePixels()) {
#ifdef OBJ_REC_RANSAC_VERBOSE_1
                        printf("[turn off %s]\n", (*it1)->getLabel());
#endif
                        (*it1)->setSceneStateOff();
                    } else {
#ifdef OBJ_REC_RANSAC_VERBOSE_1
                        printf("[turn off %s]\n", (*it2)->getLabel());
#endif
                        (*it2)->setSceneStateOff();
                    }
                } else {
#ifdef OBJ_REC_RANSAC_VERBOSE_1
                    printf("[both on]\n");
#endif
                    both_on.insert(on_pair);
                }
            }
        }
    }

    // The shapes that are still ON are the ones we want
    for (vector<ORRPointSetShape *>::iterator shape = shapes.begin(); shape != shapes.end(); ++shape)
        if ((*shape)->isSceneStateOn())
            out.push_back(*shape);
}

//====================================================================================================================
