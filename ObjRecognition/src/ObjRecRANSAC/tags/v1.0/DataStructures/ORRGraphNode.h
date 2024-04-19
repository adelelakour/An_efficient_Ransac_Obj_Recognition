/*
 * ORRGraphNode.h
 *
 *  Created on: Jan 27, 2010
 *      Author: papazov
 */

#ifndef ORRGRAPHNODE_H_
#define ORRGRAPHNODE_H_

#include <BasicTools/Graph/GraphNode.h>
#include "../Shapes/ORRPointSetShape.h"


#define ORRGraphNode GraphNode<ORRPointSetShape*, std::less<ORRPointSetShape*> >

#endif /* ORRGRAPHNODE_H_ */
