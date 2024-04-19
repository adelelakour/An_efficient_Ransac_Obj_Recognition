/*
 * PixelVector.h
 *
 *  Created on: Jun 16, 2010
 *      Author: papazov
 */

#ifndef _PIXEL_VECTOR_H_
#define _PIXEL_VECTOR_H_

#include "ORRBasicTypes.h"
#include <list>

using namespace std;

class PixelVector
{
public:
	PixelVector(int maxNumOfPixels);
	virtual ~PixelVector(){ delete[] mPixelCoords;}

	int getNumberOfOccupiedPixels()const{ return mNumOfOccupiedPixels;}
	const int_2* getPixelCoords()const{ return mPixelCoords;}
	int getMaxNumberOfPixels()const{ return mMaxNumOfPixels;}

	const list<int>& getLinearIds()const{ return mLinearIds;}

	inline void pushBackPixelCoords(int x, int y, int linearId);
	void resetNumberOfOccupiedPixels(){ mNumOfOccupiedPixels = 0; mLinearIds.clear();}

	/** Make sure that this pixel vector has space for at least as many pixels as 'src'. */
	inline void copyfrom(const PixelVector& src);

protected:
	int_2 *mPixelCoords;
	int mNumOfOccupiedPixels, mMaxNumOfPixels;
	list<int> mLinearIds;
};

//=== inline methods ==========================================================================================

inline PixelVector::PixelVector(int maxNumOfPixels)
{
	mNumOfOccupiedPixels = 0;
	mPixelCoords = new int_2[maxNumOfPixels];
	mMaxNumOfPixels = maxNumOfPixels;
}

//=============================================================================================================

inline void PixelVector::pushBackPixelCoords(int x, int y, int linearId)
{
	mPixelCoords[mNumOfOccupiedPixels].x = x;
	mPixelCoords[mNumOfOccupiedPixels].y = y;
	mLinearIds.push_back(linearId);
	++mNumOfOccupiedPixels;
}

//=============================================================================================================

inline void PixelVector::copyfrom(const PixelVector& src)
{
	mNumOfOccupiedPixels = src.getNumberOfOccupiedPixels();
	mLinearIds = src.getLinearIds();

	for ( int i = 0 ; i < mNumOfOccupiedPixels ; ++i )
	{
		mPixelCoords[i].x = src.getPixelCoords()[i].x;
		mPixelCoords[i].y = src.getPixelCoords()[i].y;
	}
}

//=============================================================================================================

#endif /* _PIXEL_VECTOR_H_ */

