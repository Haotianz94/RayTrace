#ifndef OBJECT2_H
#define OBJECT2_H

#include "foundation.h"
#include <fstream>

namespace Raytracer{

#define HIT		 1		
#define MISS	 0		
#define INTER	-1		

class Texture
{
public:
	Texture( Color** bitmap_, int width_, int height_ ): _bitmap(bitmap_), _width(width_), _height(height_) {}
	Texture( const char* file );
	Color** getBitmap() { return _bitmap; }
	Color getTexel( double U, double V );
	Color getTexel1( double U, double V);
	int getWidth() { return _width; }
	int getHeight() { return _height; }
	char* getName() { return _picname; }
private:
	Color** _bitmap;
	int _width, _height;
	char _picname[40];
};

class Property
{
private:
	Color _color;
	double _refl;
	double _diff;
	double _refr;
	double _spec;
	double _refrIndex;
	double _drefl;
	Texture* _texture;
	double _UScale, _VScale, _RUScale, _RVScale;

public:
	Property():	
		_color( Color( 0.2f, 0.2f, 0.2f ) ),_refl( 0 ), _diff( 0.2f ), _spec( 0.8f ), _refr( 0 ), _refrIndex( 1.5 ), _drefl(0), 
		_texture(0), _UScale(1.0), _VScale(1.0), _RUScale(1.0), _RVScale(1.0) {}
	void setColor( Color color_ ) { _color = color_; }
	void setDiffuse( double diff_ ) { _diff = diff_; }
	void setReflection( double refl_ ) 
	{
		_refl = refl_; 
	}
	void setRefraction( double refr_ ) { _refr = refr_; }
	void setSpecular( double spec_ ) { _spec = spec_; }
	void setRefrIndex( double refrIndex_ ) { _refrIndex = refrIndex_; }
	void setDiffuseReflection( double drefl_) { _drefl = drefl_; }
	void setTexture( Texture* texture_ ) { _texture = texture_ ; }
	void setUVScale( double UScale_,double VScale_) 
	{ _UScale = UScale_; _VScale = VScale_; _RUScale = 1.0/_UScale; _RVScale = 1.0/_VScale ; }
	void setParameters( double refl_, double refr_, Color color_, double refrIndex_ = 1.0, double diff_ = 0.2, double spec_ = 0.8 )
	{
		_refl = refl_;
		_refr = refr_;
		_refrIndex = refrIndex_;
		_color =color_;
		_diff = diff_;
		_spec = spec_;
	}
	Color getColor() { return _color; }
	double getSpecular() { return _spec; }
	double getDiffuse() { return _diff; }
	double getReflection() { return _refl; }
	double getRefraction() { return _refr; }
	double getRefrIndex() { return _refrIndex; }
	double getDiffuseReflection(){ return _drefl ; }
	Texture* getTexture() { return _texture ;}
	double getUScale() { return _UScale ;}
	double getVScale() { return _VScale ;}
	double getRUScale() { return _RUScale ;}
	double getRVScale() { return _RVScale; }
};

class Vertex
{
private:
	Vec3 _pos;
public:
	Vertex() {};
	Vertex( Vec3 a_Pos) : _pos(a_Pos) {};
	Vec3& getPos() { return _pos; }
	void setPos( Vec3& a_Pos ) { _pos = a_Pos; }
};

class Light
{
private:
	Vec3 _pos, _CellX, _CellY;
	Color _color;
	int _type;
	Vec3* _grid;
public:
	enum
	{
		POINT = 1,
		AREA
	};
	Light( int a_Type, Vec3 a_Pos, Color a_Color ) : _type( a_Type ), _pos( a_Pos ), _color( a_Color ), _grid( 0 ) {};
	Light( int a_Type, Vec3 a_P1, Vec3 a_P2, Vec3 a_P3, Color a_Color );
	Vec3& getPos() { return _pos; }
	Vec3& getCellX() { return _CellX; }
	Vec3& getCellY() { return _CellY; }
	Vec3& getGrid( int a_Idx ) { return _grid[a_Idx]; }
	Color& getColor() { return _color; }
	int getType() { return _type; }
};


class Object
{
protected:
	Property* _property;
	double _min[3];
	double _max[3];
public:
	enum
	{
		SPHERE = 1,
		TRIANGLE
	};
	Object() { _property = new Property(); }
	Property* getProperty() { return _property; }
	void setProperty( Property* property_ ) { _property = property_; }
	double getMin( int i ) { return _min[i]; }
	double getMax( int i ) { return _max[i]; }
	
	virtual int getType() = 0;
	virtual int intersect( Ray& ray, double& dist ) = 0;
	virtual Vec3 getNormal( Vec3& pos) = 0;
	virtual Color getColor(Vec3& pos) { return _property->getColor(); }
};

class Sphere : public Object
{
private:
	Vec3 _center;
	double _radius;
	double _radius2;
	double _rradius;
	Vec3 _Ve, _Vn, _Vc;
public:
	Sphere( Vec3 center_, double radius_ );
	int getType() { return SPHERE; }
	Vec3& getCenter() { return _center; }
	double getRadius() { return _radius; }
	double getRadius2() { return _radius2; }
	double getRRadius() { return _rradius; }
	Vec3 getNormal( Vec3& pos ) { return (pos - _center) * _rradius; }
	int intersect( Ray& ray, double& dist );
	Color getColor(Vec3& pos);
};

class Triangle: public Object
{
private:
	Vec3 _N;
	Vertex* _normal[3];
	Vertex* _Vertex[3];
	Vec3 _UAxis, _VAxis;
	double _au,_av, _bu,_bv, _cu,_cv;
	bool model;
public:
	Triangle(Vertex* a_V1, Vertex* a_V2, Vertex* a_V3, bool m ): model(m), Object()
	{
		_Vertex[0] = a_V1;
		_Vertex[1] = a_V2;
		_Vertex[2] = a_V3;

		Vec3 _A = _Vertex[0]->getPos();
		Vec3 _B = _Vertex[1]->getPos();
		Vec3 _C = _Vertex[2]->getPos();
		Vec3 p1 = _B - _A;
		Vec3 p2 = _C - _A;
		_N = p2.cross(p1);
		NORMALIZE(_N);
		_UAxis = Vec3( _N.y, _N.z, -_N.x );
		_VAxis = _UAxis.cross( _N );

		for( int i = 0; i < 3; i++)
		{
			_min[i] = 10000;
			_max[i] = -10000;
			for( int j = 0; j <3; j++)
			{
				double p = _Vertex[j]->getPos().cell[i];
				if( p < _min[i])
					_min[i] = p;
				if( p > _max[i])
					_max[i] = p;
			}
		}
	}
	int getType() { return TRIANGLE; }
	Vec3& getN() { return _N; }
	Vec3 getNormal( Vec3& pos){ return _N;}
	Vertex* getVertex( int id) { return _Vertex[id]; }
	void setNormals( Vertex* n1, Vertex* n2, Vertex* n3) { _normal[0] = n1; _normal[1] = n2; _normal[2] = n3; } 
	int intersect( Ray& ray, double& dist );
	Color getColor(Vec3& pos);
	void getColorNormal(Vec3& color, Vec3& normal, Vec3& pos);  
	bool isModel() { return model; }
	void setTextureScale(double au, double av, double bu, double bv, double cu,double cv)
	{
		_au = au;
		_bu = bu;
		_cu = cu;
		_av = av;
		_bv = bv;
		_cv = cv;
	}
};

class ListNode
{
private:
	Object* _object;
	ListNode* _next;
public:
	ListNode() :	_object( 0 ), _next( 0 ) {}
	~ListNode() { delete _next; }
	void setObject( Object* ob ) { _object = ob; }
	Object* getObject() { return _object; }
	void setNext( ListNode* next_ ) { _next = next_; }
	ListNode* getNext() { return _next; }
};

class KdTree;

class Scene
{
private:
	int _objects, _lights;
	Object** _object;
	Light** _light;
	ListNode** _grid;
	Box _extend;
	KdTree* _kdTree;
public:
	Scene() : _objects( 0 ), _object( 0 ), _lights( 0 ), _light( 0 ),_extend( Box( Vec3(-20,-20,-20), Vec3(40,40,40)))
	{};
	~Scene() { delete _object; delete _light; delete _grid; }
	void initScene0();
	void initScene1();
	void initScene2();
	void initScene3();
	void initScene4();
	void initScene5();
	int getObjects() { return _objects; }
	Object* getObject( int i ) { return _object[i]; }
	int getLights() { return _lights; }
	Light* getLight( int i ) { return _light[i]; }
	Box& getExtend() { return _extend; }
	void setExtend(Box box) { _extend = box; }
	KdTree* getKdTree() { return _kdTree; }
	void addBox( Vec3 a_Pos, Vec3 a_Size );
	void addPlane( Vec3 a_P1, Vec3 a_P2, Vec3 a_P3, Vec3 a_P4, Property * proper);
};

};
#endif
