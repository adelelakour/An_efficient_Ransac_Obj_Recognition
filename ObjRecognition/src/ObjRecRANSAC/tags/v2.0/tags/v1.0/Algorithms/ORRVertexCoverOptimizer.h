/*
 * ORRVertexCoverOptimizer.h
 *
 *  Created on: Jan 27, 2010
 *      Author: papazov
 */

#ifndef ORRVERTEXCOVEROPTIMIZER_H_
#define ORRVERTEXCOVEROPTIMIZER_H_

#include "../DataStructures/ORRUndirectedGraph.h"

using namespace tum;

class ORRVertexCoverOptimizer
{
public:
	ORRVertexCoverOptimizer();
	virtual ~ORRVertexCoverOptimizer();

#if 0
	void optimizeGreedy_NodeDegree(ORRUndirectedGraph& graph, list<ORRPointSetShape*>& out);
	void optimizeGreedy_NumOfVoxels(ORRUndirectedGraph& graph, list<ORRPointSetShape*>& out);
#endif
};

#endif /* ORRVERTEXCOVEROPTIMIZER_H_ */
