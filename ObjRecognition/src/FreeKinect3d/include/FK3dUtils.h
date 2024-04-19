/*
 * FK3dUtils.h
 *
 *  Created on: Dec 2, 2010
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

#include <cmath>

#define SQR(x) ((x)*(x))

inline void utils_copy3(double *dst, const double *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

inline void utils_subtract3(const double *a, const double *b, double* out)
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

inline void utils_sum3(const double *a, const double *b, double* out)
{
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	out[2] = a[2] + b[2];
}

inline void utils_add3(double *a, const double *b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
}

inline void utils_mult3(double *v, double s)
{
	v[0] *= s; v[1] *= s; v[2] *= s;
}

inline double utils_sqr_len3(const double *v)
{
	return SQR(v[0]) + SQR(v[1]) + SQR(v[2]);
}

inline void utils_normalize3(double *v)
{
	double invlen = 1.0/sqrt(SQR(v[0]) + SQR(v[1]) + SQR(v[2]));
	v[0] *= invlen;
	v[1] *= invlen;
	v[2] *= invlen;
}

inline void utils_normalize3(const double *v, double *w)
{
	double invlen = 1.0/sqrt(SQR(v[0]) + SQR(v[1]) + SQR(v[2]));
	w[0] = v[0]*invlen;
	w[1] = v[1]*invlen;
	w[2] = v[2]*invlen;
}

inline double utils_dot_prod3(const double *u, const double *v)
{
	return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

inline void utils_cross_prod3(const double *v1, const double *v2, double *out)
{
	out[0] = v1[1]*v2[2] - v1[2]*v2[1];
	out[1] = v1[2]*v2[0] - v1[0]*v2[2];
	out[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

inline void utils_rot_about_y_axis(double rad, double pos[3], double pivot[3])
{
	// Translate to origin
	double x = pos[0] - pivot[0];
	double z = pos[2] - pivot[2];

	// Rotate
	pos[0] = x*cos(rad) - z*sin(rad);
	pos[2] = x*sin(rad) + z*cos(rad);

	// Translate back
	pos[0] += pivot[0];
	pos[2] += pivot[2];
}

inline void utils_rot_about_axis(double rad, const double axis[3], const double pivot[3], double pos[3])
{
	// Translate to origin
	double x = pos[0] - pivot[0];
	double y = pos[1] - pivot[1];
	double z = pos[2] - pivot[2];

	// The rotation matrix
	double omc = 1.0 - cos(rad), c = cos(rad), s = sin(rad);
	double xy_omc = axis[0]*axis[1]*omc, yz_omc = axis[1]*axis[2]*omc, xz_omc = axis[0]*axis[2]*omc;
	double m[3][3] = {
			{c + SQR(axis[0])*omc,  xy_omc - axis[2]*s,    xz_omc + axis[1]*s},
			{xy_omc + axis[2]*s,    c + SQR(axis[1])*omc,  yz_omc - axis[0]*s},
			{xz_omc - axis[1]*s,    yz_omc + axis[0]*s,    c + SQR(axis[2])*omc}};

	// Rotate
	pos[0] = x*m[0][0] + y*m[0][1] + z*m[0][2];
	pos[1] = x*m[1][0] + y*m[1][1] + z*m[1][2];
	pos[2] = x*m[2][0] + y*m[2][1] + z*m[2][2];

	// Translate back
	pos[0] += pivot[0];
	pos[1] += pivot[1];
	pos[2] += pivot[2];
}
