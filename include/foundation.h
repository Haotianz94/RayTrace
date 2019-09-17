#ifndef FOUNDATION_H
#define FOUNDATION_H

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SAMPLES 32
#define TRACEDEPTH 5
#define MAXTREEDEPTH 20
#define EPSILON 0.00001

#define DOT(A,B)		(A.x*B.x+A.y*B.y+A.z*B.z)
#define NORMALIZE(A)	{double l=1/sqrt(A.x*A.x+A.y*A.y+A.z*A.z);A.x*=l;A.y*=l;A.z*=l;}
#define LENGTH(A)		(sqrt(A.x*A.x+A.y*A.y+A.z*A.z))
#define SQRDISTANCE(A,B) ((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y)+(A.z-B.z)*(A.z-B.z))
#define PI				3.141592653589793238462

class Vec3
{
public:
	union
	{
		struct { double x, y, z; };
		struct { double cell[3]; };
	};
	
	Vec3() : x(0), y(0), z(0) {};
	Vec3( double X_, double Y_, double Z_ ) : x(X_), y(Y_), z(Z_) {};

	void normalize() { double l = 1.0f / length(); x *= l; y *= l; z *= l; }
	double length() { return (double)sqrt( x * x + y * y + z * z ); }
	double sqrLength() { return x * x + y * y + z * z; }
	Vec3 cross( Vec3 b ) { return Vec3( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x ); }
	double det(Vec3 b, Vec3 c)
	{
		return x*b.y*c.z + b.x*c.y*z + y*b.z*c.x - c.x*b.y*z - y*b.x*c.z - b.z*c.y*x ;
	}
	
	void operator += ( Vec3 a_V ) { x += a_V.x; y += a_V.y; z += a_V.z; }
	void operator += ( Vec3* a_V ) { x += a_V->x; y += a_V->y; z += a_V->z; }
	void operator -= ( Vec3 a_V ) { x -= a_V.x; y -= a_V.y; z -= a_V.z; }
	void operator -= ( Vec3* a_V ) { x -= a_V->x; y -= a_V->y; z -= a_V->z; }
	void operator *= ( double f ) { x *= f; y *= f; z *= f; }
	void operator *= ( Vec3 a_V ) { x *= a_V.x; y *= a_V.y; z *= a_V.z; }
	void operator *= ( Vec3* a_V ) { x *= a_V->x; y *= a_V->y; z *= a_V->z; }

	Vec3 operator- () const { return Vec3( -x, -y, -z ); }
	friend Vec3 operator + ( const Vec3& v1, const Vec3& v2 ) { return Vec3( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z ); }
	friend Vec3 operator - ( const Vec3& v1, const Vec3& v2 ) { return Vec3( v1.x - v2.x, v1.y - v2.y, v1.z - v2.z ); }
	friend Vec3 operator + ( const Vec3& v1, Vec3* v2 ) { return Vec3( v1.x + v2->x, v1.y + v2->y, v1.z + v2->z ); }
	friend Vec3 operator - ( const Vec3& v1, Vec3* v2 ) { return Vec3( v1.x - v2->x, v1.y - v2->y, v1.z - v2->z ); }
	friend Vec3 operator * ( const Vec3& v, double f ) { return Vec3( v.x * f, v.y * f, v.z * f ); }
	friend Vec3 operator * ( const Vec3& v1, const Vec3& v2 ) { return Vec3( v1.x * v2.x, v1.y * v2.y, v1.z * v2.z ); }
	friend Vec3 operator * ( double f, const Vec3& v ) { return Vec3( v.x * f, v.y * f, v.z * f ); }

};

typedef Vec3 Color;

class Ray
{
private:
	Vec3 _origin;
	Vec3 _direction;
public:
	Ray() : _origin( Vec3( 0, 0, 0 ) ), _direction( Vec3( 0, 0, 0 ) ) {};
	Ray( Vec3 origin_, Vec3 dir_ ):_origin(origin_),_direction(dir_) {}
	void setOrigin( Vec3 origin_ ) { _origin = origin_; }
	void setDirection( Vec3 dir_ ) { _direction = dir_; }
	Vec3& getOrigin() { return _origin; }
	Vec3& getDirection() { return _direction; }
};

class Box
{
private:
	Vec3 _pos, _size;
public:
	Box() : _pos( Vec3( 0, 0, 0 ) ), _size( Vec3( 0, 0, 0 ) ) {};
	Box( Vec3 pos_, Vec3 size_ ) : _pos( pos_ ), _size( size_ ) {};
	Vec3& getPos() { return _pos; }
	Vec3& getSize() { return _size; }
	bool overlap( Box& b2 )
	{
		Vec3 p1 = b2.getPos(), p2 = b2.getPos() + b2.getSize();
		Vec3 p3 = _pos, p4 = _pos + _size;
		return ((p4.x > p1.x) && (p3.x < p2.x) && (p4.y > p1.y) && (p3.y < p2.y) && (p4.z > p1.z) && (p3.z < p2.z));  
	}
	bool contains( Vec3 pos )
	{
		Vec3 p1 = _pos, p2 = _pos + _size;
		return ((pos.x > (p1.x - 0.001)) && (pos.x < (p2.x + 0.001)) &&
				(pos.y > (p1.y - 0.001)) && (pos.y < (p2.y + 0.001)) &&
				(pos.z > (p1.z - 0.001)) && (pos.z < (p2.z + 0.001)));
	}
	double w() { return _size.x; }
	double h() { return _size.y; }
	double d() { return _size.z; }
	double x() { return _pos.x; }
	double y() { return _pos.y; }
	double z() { return _pos.z; }
};

class Matrix
{
public:
	enum 
	{ 
		TX=3, 
		TY=7, 
		TZ=11, 
		D0=0, D1=5, D2=10, D3=15, 
		SX=D0, SY=D1, SZ=D2, 
		W=D3 
	};
	Matrix() { identity(); }
	void identity()
	{
		cell[1] = cell[2] = cell[TX] = cell[4] = cell[6] = cell[TY] =
		cell[8] = cell[9] = cell[TZ] = cell[12] = cell[13] = cell[14] = 0;
		cell[D0] = cell[D1] = cell[D2] = cell[W] = 1;
	}
	void rotate( Vec3 a_Pos, double a_RX, double a_RY, double a_RZ )
	{
		Matrix t;
		t.rotateX( a_RZ );
		rotateY( a_RY );
		concatenate( t );
		t.rotateZ( a_RX );
		concatenate( t );
		translate( a_Pos );
	}
	void rotateX( double a_RX )
	{
		double sx = (double)sin( a_RX * PI / 180 );
		double cx = (double)cos( a_RX * PI / 180 );
		identity();
		cell[5] = cx, cell[6] = sx, cell[9] = -sx, cell[10] = cx;
	}
	void rotateY( double a_RY )
	{
		double sy = (double)sin( a_RY * PI / 180 );
		double cy = (double)cos( a_RY * PI / 180 );
		identity ();
		cell[0] = cy, cell[2] = -sy, cell[8] = sy, cell[10] = cy;
	}
	void rotateZ( double a_RZ )
	{
		double sz = (double)sin( a_RZ * PI / 180 );
		double cz = (double)cos( a_RZ * PI / 180 );
		identity ();
		cell[0] = cz, cell[1] = sz, cell[4] = -sz, cell[5] = cz;
	}
	void translate( Vec3 a_Pos ) { cell[TX] += a_Pos.x; cell[TY] += a_Pos.y; cell[TZ] += a_Pos.z; }
	void concatenate( Matrix& m2 )
	{
		Matrix res;
		for ( int c = 0; c < 4; c++ ) for ( int r = 0; r < 4; r++ )
			res.cell[r * 4 + c] = cell[r * 4] * m2.cell[c] +
				  				  cell[r * 4 + 1] * m2.cell[c + 4] +
								  cell[r * 4 + 2] * m2.cell[c + 8] +
								  cell[r * 4 + 3] * m2.cell[c + 12];
		for ( int c = 0; c < 16; c++ ) cell[c] = res.cell[c];
	}
	Vec3 transform( Vec3& v )
	{
		double x  = cell[0] * v.x + cell[1] * v.y + cell[2] * v.z + cell[3];
		double y  = cell[4] * v.x + cell[5] * v.y + cell[6] * v.z + cell[7];
		double z  = cell[8] * v.x + cell[9] * v.y + cell[10] * v.z + cell[11];
		return Vec3( x, y, z );
	}
	void invert()
	{
		Matrix t;
		double tx = -cell[3], ty = -cell[7], tz = -cell[11];
		for ( int h = 0; h < 3; h++ ) for ( int v = 0; v < 3; v++ ) t.cell[h + v * 4] = cell[v + h * 4];
		for ( int i = 0; i < 11; i++ ) cell[i] = t.cell[i];
		cell[3] = tx * cell[0] + ty * cell[1] + tz * cell[2];
		cell[7] = tx * cell[4] + ty * cell[5] + tz * cell[6];
		cell[11] = tx * cell[8] + ty * cell[9] + tz * cell[10];
	}
	double cell[16];
};

#endif