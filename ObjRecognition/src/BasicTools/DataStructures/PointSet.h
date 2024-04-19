/*
 * PointSet.h
 *
 *  Created on: Aug 18, 2010
 *      Author: papazov
 */

#ifndef POINTSET_H_
#define POINTSET_H_

#include <cstdio>
#include <cstring>
#include <btl1/Vector.h>
#include <list>

using namespace std;
using namespace btl1;

class PointSet
{
public:
	inline PointSet();
	inline PointSet(int numberOfPoints);
	inline virtual ~PointSet();

	virtual inline void alloc(int numberOfPoints);
	virtual inline void clear();

	inline void copyfrom(const PointSet* src);
	inline void copy_to_array(double *points) const;

	/** Performs a memset on the array with the point coordinates. */
	void memsetPoints(int value){ memset(mPoints, value, 3*mNumOfPoints*sizeof(double));}

	int getNumberOfPoints()const{ return mNumOfPoints;}
	int getNumberOfBytes()const{ return 3*mNumOfPoints*sizeof(double);}
	const double* getPoints_const()const{ return mPoints;}
	double* getPoints(){ return mPoints;}
	const double* getBounds()const{ return mBounds;}
	inline void setBounds(const double* b);
	inline void getBounds(double* b)const;
	const double* getPoint(int id)const{ return mPoints + 3*id;}
	void setPoint(int id, const double* p){ vec_copy3(p, mPoints + 3*id);}

	inline void addToPoint(int id, const double* u);

	inline bool saveAsTXT(const char* filename, bool verbose = true) const;
	/** Does not allocate new memory but just rewrites the points of this point set with the ones saved in 'filename'. */
	inline bool updateFromTXT(const char* filename, bool verbose = true);
	/** Load from a .txt file */
	inline bool loadFromTXT(const char* filename, bool verbose = true);

protected:
	double* mPoints;
	int mNumOfPoints;
	double mBounds[6];
};

//=== inline methods =======================================================================================

inline PointSet::PointSet()
{
	mPoints = NULL;
	mNumOfPoints = 0;
}

//==========================================================================================================

inline PointSet::PointSet(int numberOfPoints)
{
	mPoints = new double[3*numberOfPoints];
	mNumOfPoints = numberOfPoints;
}

//==========================================================================================================

inline PointSet::~PointSet()
{
	PointSet::clear();
}

//==========================================================================================================

inline void PointSet::clear()
{
	if ( mPoints )
	{
		delete[] mPoints;
		mPoints = NULL;
	}
	mNumOfPoints = 0;
}

//==========================================================================================================

inline void PointSet::alloc(int numberOfPoints)
{
	PointSet::clear();
	mPoints = new double[3*numberOfPoints];
	mNumOfPoints = numberOfPoints;
}

//==========================================================================================================

inline void PointSet::copyfrom(const PointSet* src)
{
	PointSet::alloc(src->getNumberOfPoints());
	memcpy(this->mPoints, src->getPoints_const(), 3*mNumOfPoints*sizeof(double));
	src->getBounds(mBounds);
}

//==========================================================================================================

inline void PointSet::copy_to_array(double *points) const
{
	memcpy(points, mPoints, 3*mNumOfPoints*sizeof(double));
}

//==========================================================================================================

inline void PointSet::getBounds(double* b) const
{
	b[0] = mBounds[0];
	b[1] = mBounds[1];
	b[2] = mBounds[2];
	b[3] = mBounds[3];
	b[4] = mBounds[4];
	b[5] = mBounds[5];
}

//==========================================================================================================

inline void PointSet::setBounds(const double* b)
{
	mBounds[0] = b[0];
	mBounds[1] = b[1];
	mBounds[2] = b[2];
	mBounds[3] = b[3];
	mBounds[4] = b[4];
	mBounds[5] = b[5];
}

//==========================================================================================================

inline void PointSet::addToPoint(int id, const double* u)
{
	double* p = mPoints + 3*id;
	p[0] += u[0];
	p[1] += u[1];
	p[2] += u[2];
}

//==================================================================================================================

inline bool PointSet::saveAsTXT(const char* filename, bool verbose) const
{
	FILE* fp = fopen(filename, "w");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'PointSet::%s()': can not open '%s'\n", __func__, filename);
		return false;
	}

	if ( verbose )
	{
		printf("Saving '%s' ... ", filename);
		fflush(stdout);
	}

	double *p = mPoints;

	for ( int i = 0 ; i < mNumOfPoints ; ++i, p += 3 )
		fprintf(fp, "%lf %lf %lf\n", p[0], p[1], p[2]);

	fflush(fp);

	if ( verbose )
	{
		printf("OK\n");
		fflush(stdout);
	}

	return true;
}

//==========================================================================================================

inline bool PointSet::updateFromTXT(const char* filename, bool verbose)
{
	FILE* fp = fopen(filename, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'PointSet::%s()': can not open '%s'\n", __func__, filename);
		return false;
	}

	if ( verbose )
	{
		printf("Updating from '%s' ... ", filename);
		fflush(stdout);
	}

	double p[3], *points = mPoints;
	int i;

	// Update the points
	for ( i = 0 ; fscanf(fp, "%lf %lf %lf\n", &p[0], &p[1], &p[2]) == 3 && i < mNumOfPoints ; ++i, points += 3 )
		vec_copy3(p, points);

	if ( verbose )
	{
		printf("%i of %i points were updated.\n", i, mNumOfPoints);
		fflush(stdout);
	}

	return true;
}

//==========================================================================================================

inline bool PointSet::loadFromTXT(const char* filename, bool verbose)
{
	FILE* fp = fopen(filename, "r");
	if ( fp == NULL )
	{
		fprintf(stderr, "ERROR in 'PointSet::%s()': can not open '%s'\n", __func__, filename);
		return false;
	}

	if ( verbose )
	{
		printf("Reading from '%s' ... ", filename);
		fflush(stdout);
	}

	double x, y, z;
	list<double*> point_list;

	// Read the points from disk
	while ( fscanf(fp, "%lf %lf %lf\n", &x, &y, &z) == 3 )
	{
		double *p = new double[3];
		p[0] = x; p[1] = y; p[2] = z;
		point_list.push_back(p);
	}

	// Allocate memory
	this->alloc((int)point_list.size());
	double *points = mPoints;
	// Copy to the point array
	for ( list<double*>::iterator it = point_list.begin() ; it != point_list.end() ; ++it, points += 3 )
	{
		vec_copy3(*it, points);
		delete[] (*it);
	}
	point_list.clear();

	if ( verbose )
	{
		printf("we have %i point(s).\n", mNumOfPoints);
		fflush(stdout);
	}

	return true;
}

//==========================================================================================================

#endif /* POINTSET_H_ */
