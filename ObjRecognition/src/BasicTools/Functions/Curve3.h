/*
 * Curve3.h
 *
 *  Created on: Aug 30, 2011
 *      Author: papazov
 */

#ifndef CURVE3_H_
#define CURVE3_H_

class Curve3
{
public:
	Curve3(){}
	virtual ~Curve3(){}

	/** y = g(t) */
	virtual void g(double t, double* y) = 0;
};

#endif /* CURVE3_H_ */
