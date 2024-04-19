/*
 * ORRVertexCoverOptimizer.cpp
 *
 *  Created on: Jan 27, 2010
 *      Author: papazov
 */

#include "ORRVertexCoverOptimizer.h"
#include "../DataStructures/ORRGraphNode.h"
#include <algorithm>
#include <vector>

using namespace std;

#if 0
bool compareORRGraphNodes_NodeDegree(ORRGraphNode* node1, ORRGraphNode* node2)
{
	return (bool)(node1->getNumberOfNeighbours() < node2->getNumberOfNeighbours());
}

bool compareORRGraphNodes_NumOfVoxels(ORRGraphNode* node1, ORRGraphNode* node2)
{
	return (bool)(node1->getKey()->getNumberOfVoxelIds() > node2->getKey()->getNumberOfVoxelIds());
}
#endif

ORRVertexCoverOptimizer::ORRVertexCoverOptimizer()
{
}

ORRVertexCoverOptimizer::~ORRVertexCoverOptimizer()
{
}

//==================================================================================================================================
#if 0
void ORRVertexCoverOptimizer::optimizeGreedy_NodeDegree(ORRUndirectedGraph& graph, list<ORRPointSetShape*>& out)
{
	int numOfNodes = graph.getNumberOfNodes();
	vector<ORRGraphNode*> nodes(numOfNodes);
	map<ORRPointSetShape*, ORRGraphNode*>::iterator iter;
	int i, j, maxScore, maxId;

	// Put the nodes to a vector
	for ( i = 0, iter = graph.getNodes().begin() ; iter != graph.getNodes().end() ; ++i, ++iter )
	{
		nodes[i] = iter->second;
		iter->second->getKey()->setSceneState(ORRPointSetShape::SCENE_FREE);
	}
	// Sort the vector according to the number of neighbours
	sort(nodes.begin(), nodes.end(), compareORRGraphNodes_NodeDegree);

	for ( i = 0 ; i < numOfNodes ; )
	{
		// Proceed to the next node only if the current node is 'on' or 'off'
		if ( nodes[i]->getKey()->getSceneState() != ORRPointSetShape::SCENE_FREE )
		{
			++i;
			continue;
		}
		// Get the score of the current node
		maxScore = nodes[i]->getKey()->getNumberOfVoxelIds();
		maxId = i;
		// Get the best node among all nodes with the same number of neighbours
		for ( j = i+1 ; j < numOfNodes ; ++j )
		{
			// Be sure that the next node has the same number of neighbours
			if ( nodes[i]->getNumberOfNeighbours() != nodes[j]->getNumberOfNeighbours() )
				break;
			// Be sure that the next node is still free
			if ( nodes[j]->getKey()->getSceneState() != ORRPointSetShape::SCENE_FREE )
				continue;
			// Check if the next node has a better score than the current one
			if ( nodes[j]->getKey()->getNumberOfVoxelIds() > maxScore )
			{
				maxScore = nodes[j]->getKey()->getNumberOfVoxelIds(); // We have a new max score
				maxId = j; // Save this id as the best id
			}
		}
		// "Switch off" the neighbours of the best node
		map<ORRPointSetShape*, ORRGraphNode*>& neighs = nodes[maxId]->getNeighbours();
		for ( map<ORRPointSetShape*, ORRGraphNode*>::iterator neigh = neighs.begin() ; neigh != neighs.end() ; ++neigh )
			neigh->first->setSceneState(ORRPointSetShape::SCENE_OFF);
		// "Switch on" the best node and save its geometric shape in 'out'
		nodes[maxId]->getKey()->setSceneState(ORRPointSetShape::SCENE_ON);
		out.push_back(nodes[maxId]->getKey());
	}
}

//==================================================================================================================================

void ORRVertexCoverOptimizer::optimizeGreedy_NumOfVoxels(ORRUndirectedGraph& graph, list<ORRPointSetShape*>& out)
{
	int i, numOfNodes = graph.getNumberOfNodes();
	vector<ORRGraphNode*> nodes(numOfNodes);
	map<ORRPointSetShape*, ORRGraphNode*>::iterator iter;

	// Put the nodes to a vector
	for ( i = 0, iter = graph.getNodes().begin() ; iter != graph.getNodes().end() ; ++i, ++iter )
	{
		nodes[i] = iter->second;
		iter->second->getKey()->setSceneState(ORRPointSetShape::SCENE_FREE);
	}
	// Sort the vector according to the score of the corresponding shape
	sort(nodes.begin(), nodes.end(), compareORRGraphNodes_NumOfVoxels);

	// Do the real processing
	for ( i = 0 ; i < numOfNodes ; ++i )
	{
		// Proceed to the next node only if the current node is 'free'
		if ( nodes[i]->getKey()->getSceneState() == ORRPointSetShape::SCENE_FREE )
		{
			// "Switch off" the neighbours of the current node
			map<ORRPointSetShape*, ORRGraphNode*>& neighs = nodes[i]->getNeighbours();
			for ( map<ORRPointSetShape*, ORRGraphNode*>::iterator neigh = neighs.begin() ; neigh != neighs.end() ; ++neigh )
				neigh->first->setSceneState(ORRPointSetShape::SCENE_OFF);
			// "Switch on" the current node and save its geometric shape in 'out'
			nodes[i]->getKey()->setSceneState(ORRPointSetShape::SCENE_ON);
			out.push_back(nodes[i]->getKey());
		}
	}
}
#endif
//==================================================================================================================================
