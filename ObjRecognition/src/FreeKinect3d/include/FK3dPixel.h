/*
 * FK3dPixel.h
 *
 *  Created on: Nov 30, 2010
 *      Author: papazov
 */

// Copyright (C) 2010 Chavdar Papazov

// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along with
// this program. If not, see http://www.gnu.org/licenses/.

#ifndef _FK3D_PIXEL_H_
#define _FK3D_PIXEL_H_

class FK3dPixel
{
public:
	FK3dPixel(){ mValid = false;}
	FK3dPixel(const double* p){ mCoords[0] = p[0]; mCoords[1] = p[1]; mCoords[2] = p[2]; mValid = true;}
	virtual ~FK3dPixel(){}

	void set(const double* p){ mCoords[0] = p[0]; mCoords[1] = p[1]; mCoords[2] = p[2]; mValid = true;}
	const double* p()const{ return mCoords;}
	void get(double* p)const{ p[0] = mCoords[0]; p[1] = mCoords[1]; p[2] = mCoords[2];}
	bool isValid()const{ return mValid;}
	void setInvalid(){ mValid = false;}

protected:
	double mCoords[3];
	bool mValid;
};

#endif /* _FK3D_PIXEL_H_ */
