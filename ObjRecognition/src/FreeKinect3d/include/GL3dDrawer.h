/*
 * GL3dDrawer.h
 *
 *  Created on: Dec 13, 2010
 *      Author: papazov
 */

#ifndef GL3DDRAWER_H_
#define GL3DDRAWER_H_

class GL3dDrawer
{
public:
	GL3dDrawer(){}
	virtual ~GL3dDrawer(){}

	virtual void draw3d() = 0;
};

#endif /* GL3DDRAWER_H_ */
