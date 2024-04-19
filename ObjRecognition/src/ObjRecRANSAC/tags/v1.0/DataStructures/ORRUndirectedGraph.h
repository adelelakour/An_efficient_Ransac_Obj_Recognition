/*
 * ORRUndirectedGraph.h
 *
 *  Created on: Jan 27, 2010
 *      Author: papazov
 */

#ifndef _ORR_UNDIRECTED_GRAPH_H_
#define _ORR_UNDIRECTED_GRAPH_H_

#include <BasicTools/Graph/UndirectedGraph.h>
#include "../Shapes/ORRPointSetShape.h"


#define ORRUndirectedGraph UndirectedGraph<ORRPointSetShape*, std::less<ORRPointSetShape*> >

#endif /* _ORR_UNDIRECTED_GRAPH_H_ */
