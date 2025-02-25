/*
 * EdgeStructure.h
 *
 *  Created on: Feb 14, 2011
 *      Author: papazov
 */

#ifndef _EDGE_STRUCTURE_H_
#define _EDGE_STRUCTURE_H_

#include "PointSet.h"
#include <vtkPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkCell.h>
#include <set>
#include <list>

using namespace std;
using namespace btl1;

/** This is basically a graph embedded in R^3. We have points (inherited from 'PointSet') and edges
  * connecting neighboring points. For each point there is a list of integers storing the ids of
  * the neighboring points. A point is *not* a neighbor of itself. */
class EdgeStructure: public PointSet
{
public:
	inline EdgeStructure();
	virtual ~EdgeStructure(){ EdgeStructure::clear();}

	class PointFunc{
		public: virtual void process(EdgeStructure* pointSet, int pointId) = 0;
	};

	inline void copyfrom(vtkPolyData* src, bool copyNormals = true);
	inline void copyto(vtkPolyData* dst, bool copyNormals = false);

	/** NOT thread safe! If 'forEachPoint' != NULL the method calls 'forEachPoint->process()' for each neighbor. Note that this method
	  * needs to perform computations each time it is called. If you need to call it often and for each point, it is better to compute
	  * the K-ring neighborhood for all points once and use the 'getNeighborhood()' method. */
	inline void getNeighbors(int pointId, int neighRadius, list<int>& ids, list<const double*>& points, PointFunc* forEachPoint = NULL);

	void addToNeighborhood(int i, int neigh_id){ mNeighStruct[i].push_back(neigh_id);}

	/** Computes the K-ring neighborhood. It is assumed that the 1-ring neighborhood is available. Otherwise nothing happens.
	  * If 'radius' <= 1 the method immediately returns. */
	inline void buildKRingNeighborhood(int neighborhoodRadius);
	int getNeighborhoodRadius(){ return mNeighRadius;}
	const list<int>& getNeighborhood(int id)const{ return mNeighStruct[id];}

	inline void setNormal(int id, double x, double y, double z);
	bool validNormal(int id)const{ return (bool)(mValidNormal[id]);}
	const double* getNormal(int id)const{ return mNormals + 3*id;}
	const double* getNormals()const{ return mNormals;}

	virtual inline void alloc(int numberOfPoints);
	virtual inline void clear();

	/** Saves the points and edges between them. */
	inline bool saveAsES(const char* filename, bool verbose = true) const;
	/** Loads from an .es file */
	inline bool loadFromES(const char* filename, bool verbose = true);

protected:
	inline void sort2(vtkIdType array[2]);

protected:
	list<int>* mNeighStruct;
	double* mNormals;
	unsigned char *mValidNormal;
	int *mWorkingMemory, mNeighRadius;
};

//=== inline methods ===============================================================================================

inline EdgeStructure::EdgeStructure()
{
	mNeighStruct = NULL;
	mNormals = NULL;
	mValidNormal = NULL;
	mWorkingMemory = NULL;
	mNeighRadius = 0;
}

//==================================================================================================================

inline void EdgeStructure::alloc(int numberOfPoints)
{
	EdgeStructure::clear();
	PointSet::alloc(numberOfPoints);

	mNeighStruct = new list<int>[numberOfPoints];
	mNormals = new double[3*numberOfPoints];

	mValidNormal = new unsigned char[numberOfPoints];
	memset(mValidNormal, 0, sizeof(unsigned char)*numberOfPoints);

	mWorkingMemory = new int[numberOfPoints];
	memset(mWorkingMemory, 0, sizeof(int)*numberOfPoints);
}

//==================================================================================================================

inline void EdgeStructure::clear()
{
	if ( mNeighStruct ){ delete[] mNeighStruct; mNeighStruct = NULL;}
	if ( mNormals ){ delete[] mNormals; mNormals = NULL;}
	if ( mValidNormal ){ delete[] mValidNormal; mValidNormal = NULL;}
	if ( mWorkingMemory ){ delete[] mWorkingMemory; mWorkingMemory = NULL;}
	mNeighRadius = 0;

	PointSet::clear();
}

//==================================================================================================================

inline void EdgeStructure::setNormal(int id, double x, double y, double z)
{
	mValidNormal[id] = 1;
	id *= 3;
	mNormals[id]   = x;
	mNormals[id+1] = y;
	mNormals[id+2] = z;
}

//==================================================================================================================

inline bool es_compare_edges(vtkIdType edge1[2], vtkIdType edge2[2])
{
	if ( edge1[0] == edge2[0] )
		return (edge1[1] < edge2[1]);

	return (edge1[0] < edge2[0]);
}

typedef set<vtkIdType*, bool(*)(vtkIdType*, vtkIdType*)> edge_set;

//==================================================================================================================

inline void EdgeStructure::copyfrom(vtkPolyData* input, bool copyNormals)
{
	this->alloc(input->GetNumberOfPoints());

	edge_set edges(es_compare_edges);
	pair<edge_set::iterator, bool> ret;

	double *own_pts = mPoints;
	vtkIdType c, e;
	vtkCell *cell, *edge;

	// Copy the points
	for ( c = 0 ; c < mNumOfPoints ; ++c, own_pts += 3 )
		input->GetPoint(c, own_pts);

	if ( input->GetNumberOfLines() )
	{
		// Using lines
		vtkCellArray *lines = input->GetLines();
		  lines->InitTraversal();
		vtkIdType num_ids, *ids;

		// Go through all lines
		while ( lines->GetNextCell(num_ids, ids) )
		{
			if ( num_ids != 2 )
				continue;

			// Create a new edge
			vtkIdType* new_edge = new vtkIdType[2];
			  new_edge[0] = ids[0];
			  new_edge[1] = ids[1];
			  this->sort2(new_edge);

			// Check if we already have this edge
			ret = edges.insert(new_edge);
			if ( ret.second )
			{
				// Make the edge points neighbors
				mNeighStruct[new_edge[0]].push_back(new_edge[1]);
				mNeighStruct[new_edge[1]].push_back(new_edge[0]);
			}
			else
				delete[] new_edge;
		}
	}
	else if ( input->GetNumberOfCells() )
	{
		for ( c = 0 ; c < input->GetNumberOfCells() ; ++c )
		{
			// Get the current cell (not necessarily a triangle)
			cell = input->GetCell(c);

			// Copy each edge (we assume that each edge is a line)
			for ( e = 0 ; e < cell->GetNumberOfEdges() ; ++e )
			{
				edge = cell->GetEdge(e);
				// Create a new edge
				vtkIdType* new_edge = new vtkIdType[2];
				// Compose the edge
				new_edge[0] = edge->GetPointId(0); new_edge[1] = edge->GetPointId(1);
				// Sort in order to avoid inserting both (a, b) and (b, a), for some integers a, b.
				this->sort2(new_edge);
				// Check if we already have this edge
				ret = edges.insert(new_edge);
				if ( ret.second )
				{
					// Make the edge points neighbors
					mNeighStruct[new_edge[0]].push_back(new_edge[1]);
					mNeighStruct[new_edge[1]].push_back(new_edge[0]);
				}
				else
					delete[] new_edge;
			}
		}
	}

	// We have the 1-ring neighborhood
	mNeighRadius = 1;

	// Should we copy the normals as well
	if ( copyNormals )
	{
		// Get the input normals
		vtkDataArray* normals = input->GetPointData()->GetNormals();
		if ( normals )
		{
			double *own_normals = mNormals;
			for ( c = 0 ; c < mNumOfPoints ; ++c, own_normals += 3 )
				normals->GetTuple(c, own_normals);
		}
	}

	// Cleanup
	for ( edge_set::iterator it = edges.begin() ; it != edges.end() ; ++it )
		delete[] *it;
	edges.clear();
}

//==================================================================================================================

void EdgeStructure::copyto(vtkPolyData* dst, bool copyNormals)
{
	edge_set edges(es_compare_edges);
	pair<edge_set::iterator, bool> ret;

	vtkPoints* points = vtkPoints::New(VTK_DOUBLE);
	  points->SetNumberOfPoints(mNumOfPoints);
	vtkCellArray* lines = vtkCellArray::New();
	const double *own_pts;
	int i;

	// For all points
	for ( i = 0, own_pts = mPoints ; i < mNumOfPoints ; ++i, own_pts += 3 )
	{
		points->SetPoint(i, own_pts);

		// Insert the edges connecting the current point with its neighbors
		for ( list<int>::iterator it = mNeighStruct[i].begin() ; it != mNeighStruct[i].end() ; ++it )
		{
			vtkIdType* edge = new vtkIdType[2];
			// Compose the edge
			edge[0] = i; edge[1] = *it;
			// Sort in order to avoid inserting both (a, b) and (b, a), for some integers a, b.
			this->sort2(edge);
			// Try to insert
			ret = edges.insert(edge);
			if ( ret.second ) // Check if insertion was successful
				lines->InsertNextCell(2, edge);
			else
				delete[] edge;
		}
	}

	if ( copyNormals )
	{
		// Create the vtk normals object
		vtkDoubleArray* normals = vtkDoubleArray::New();
		  normals->SetNumberOfComponents(3);

		const double *own_normals = mNormals;
		// Initialize the vtk-normals
		for ( i = 0 ; i < mNumOfPoints ; ++i, own_normals += 3 )
			normals->InsertNextTuple(own_normals);

		// Save the normals
		dst->GetPointData()->SetNormals(normals);
		normals->Delete();
	}

	// Save the points and lines
	dst->SetPoints(points);
	dst->SetLines(lines);

	// Cleanup
	points->Delete();
	lines->Delete();
	for ( edge_set::iterator it = edges.begin() ; it != edges.end() ; ++it )
		delete[] *it;
	edges.clear();
}

//==================================================================================================================

inline void EdgeStructure::sort2(vtkIdType array[2])
{
	if ( array[0] > array[1] )
	{
		const vtkIdType tmp = array[1];
		array[1] = array[0];
		array[0] = tmp;
	}
}

//==================================================================================================================

inline void EdgeStructure::getNeighbors(int pointId, int neighRadius,
		list<int>& ids, list<const double*>& points, PointFunc* forEachPoint)
{
	list<int> idsToCheck, idsToReset;
	int id, radius, nextRadius;

	// Init
	idsToCheck.push_back(pointId);
	idsToReset.push_back(pointId);
	mWorkingMemory[pointId] = 1;

	while ( idsToCheck.size() )
	{
		// Get the current id
		id = idsToCheck.front();
		idsToCheck.pop_front();
		radius = mWorkingMemory[id];

		// Check if the current point is already at the border
		if ( radius > neighRadius )
			continue;

		// Compute the radius all neighbors of the current point will get
		nextRadius = radius + 1;

		// Go for all neighbors and expand them if they are not checked
		for ( list<int>::iterator neigh = mNeighStruct[id].begin() ; neigh != mNeighStruct[id].end() ; ++neigh )
		{
			// Check if unexpanded and within 'neighRadius'
			if ( mWorkingMemory[*neigh] == 0 )
			{
				idsToCheck.push_back(*neigh);
				idsToReset.push_back(*neigh);
				mWorkingMemory[*neigh] = nextRadius;
				// Save in the solution lists
				ids.push_back(id);
				points.push_back(this->getPoint(id));
				// Should the current point be processed any further?
				if ( forEachPoint )
					forEachPoint->process(this, *neigh);
			}
		}
	}

	// Reset the working memory
	while ( idsToReset.size() )
	{
		mWorkingMemory[idsToReset.front()] = 0;
		idsToReset.pop_front();
	}
}

//==================================================================================================================

inline void EdgeStructure::buildKRingNeighborhood(int neighborhoodRadius)
{
	if ( neighborhoodRadius <= 1 )
		return;

	list<int> idsToCheck, *newNeighStruct = new list<int>[mNumOfPoints];
	int i, id, radius, nextRadius;

	for ( i = 0 ; i < mNumOfPoints ; ++i )
	{
		// Init
		mWorkingMemory[i] = 1;

		// Push the 1-ring neighborhood
		for ( list<int>::iterator neigh = mNeighStruct[i].begin() ; neigh != mNeighStruct[i].end() ; ++neigh )
		{
			idsToCheck.push_back(*neigh);
			newNeighStruct[i].push_back(*neigh);
			mWorkingMemory[*neigh] = 1;
		}

		// Now expand the 1-ring neighborhood
		while ( idsToCheck.size() )
		{
			// Get the current id
			id = idsToCheck.front();
			idsToCheck.pop_front();
			radius = mWorkingMemory[id];

			// Check if the current point is already at the "border" of the K-ring neighborhood
			if ( radius >= neighborhoodRadius )
				continue;

			// Compute the radius which all neighbors of the current point will get
			nextRadius = radius + 1;

			// Go for all neighbors and expand them if they are not already checked
			for ( list<int>::iterator neigh = mNeighStruct[id].begin() ; neigh != mNeighStruct[id].end() ; ++neigh )
			{
				// Check if unexpanded
				if ( mWorkingMemory[*neigh] == 0 )
				{
					idsToCheck.push_back(*neigh);
					mWorkingMemory[*neigh] = nextRadius;
					// Insert the new neighbor in the new neighborhood structure
					newNeighStruct[i].push_back(*neigh);
				}
			}
		}

		// Reset the working memory
		for ( list<int>::iterator it = newNeighStruct[i].begin() ; it != newNeighStruct[i].end() ; ++it )
			mWorkingMemory[*it] = 0;
		mWorkingMemory[i] = 0;
	}

	// Delete the old neighborhood structure and replace it with the new one
	delete[] mNeighStruct;
	mNeighStruct = newNeighStruct;
}

//==================================================================================================================

inline bool EdgeStructure::saveAsES(const char* filename, bool verbose) const
{
	FILE* fp = fopen(filename, "w");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'EdgeStructure::%s()': can not open '%s'\n", __func__, filename);
		return false;
	}

	if ( verbose )
	{
		printf("Saving '%s' ... ", filename);
		fflush(stdout);
	}

	const double *p = mPoints;
	int i;

	// Save the points first
	fprintf(fp, "$P\n");
	for ( i = 0 ; i < mNumOfPoints ; ++i, p += 3 )
		fprintf(fp, "%lf %lf %lf\n", p[0], p[1], p[2]);

	// Now save the edges
	fprintf(fp, "$E\n");
	for ( i = 0 ; i < mNumOfPoints ; ++i )
		for ( list<int>::iterator it = mNeighStruct[i].begin() ; it != mNeighStruct[i].end() ; ++it )
			fprintf(fp, "%i %i\n", i, *it);

	fflush(fp);

	if ( verbose )
	{
		printf("OK\n");
		fflush(stdout);
	}

	return true;
}

//==================================================================================================================

inline bool EdgeStructure::loadFromES(const char* filename, bool verbose)
{
	FILE* fp = fopen(filename, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'EdgeStructure::%s()': can not open '%s'\n", __func__, filename);
		return false;
	}

	char line[2048];

	while ( fgets(line, 2048, fp) )
	{
		if ( strlen(line) <= 1 )
			continue;
		else if ( line[0] == '$' && (line[1] == 'P' || line[1] == 'p') ) // The starting point of the point coordinates
		{
			list<double*> point_list;
			double x, y, z;

			// Start reading the point coordinates
			while ( fscanf(fp, "%lf %lf %lf\n", &x, &y, &z) == 3 )
			{
				double *p = new double[3];
				p[0] = x; p[1] = y; p[2] = z;
				point_list.push_back(p);
			}
			if ( verbose )
			{
				printf("[%i point(s)] ", (int)point_list.size());
				fflush(stdout);
			}
			// Allocate memory for the points
			this->alloc((int)point_list.size());
			double *points = mPoints;
			// Copy to the point array
			for ( list<double*>::iterator it = point_list.begin() ; it != point_list.end() ; ++it, points += 3 )
			{
				vec_copy3(*it, points);
				delete[] (*it);
			}
			point_list.clear();
		}
		else if ( line[0] == '$' && (line[1] == 'E' || line[1] == 'e') ) // The starting point of the edges
		{
			int i, j;
			while ( fscanf(fp, "%i %i\n", &i, &j) == 2 )
				mNeighStruct[i].push_back(j);
		}
	}

	return true;
}

//==================================================================================================================


#endif /* _EDGE_STRUCTURE_H_ */
