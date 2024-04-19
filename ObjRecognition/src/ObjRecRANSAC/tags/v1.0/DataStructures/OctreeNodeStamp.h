/*
 * OctreeNodeStamp.h
 *
 *  Created on: Mar 5, 2010
 *      Author: papazov
 */

#ifndef OCTREENODESTAMP_H_
#define OCTREENODESTAMP_H_

#include <BasicTools/ComputationalGeometry/DataStructures/Octree/OctreeNode.h>

using namespace tum;

class OctreeNodeStamp
{
public:
	OctreeNodeStamp();
	virtual ~OctreeNodeStamp();

public:
	int mId1, mId2;
	OctreeNode* mNode;
};

#endif /* OCTREENODESTAMP_H_ */
