#include "kdtree.h"
#include "object.h"
#include <cstring>
#include <vector>

using namespace cv;

namespace Raytracer{

Texture::Texture(const char* file)
{
	strcpy(_picname, file);
	Mat img;
	img = imread(file);
	_width = img.cols;
	_height = img.rows;

	_bitmap = new Color* [_height];
	for(int i = 0; i < _height; i++)
		_bitmap[i] = new Color[_width];
	double r = 1.0 / 256;
	for(int y = 0;y < _height; y++)
		for(int x = 0; x < _width; x++)
		{
			Vec3b color = img.at<Vec3b>(y,x);
			_bitmap[y][x] = Color( color[2] * r,color[1] * r,color[0] * r);
		}		
}

Color Texture::getTexel( double U, double V )
{
	double fu = (U + 1000.5f) * _width;
	double fv = (V + 1000.0f) * _width;
	int u1 = ((int)fu) % _width;
	int v1 = ((int)fv) % _height;
	int u2 = (u1 + 1) % _width;
	int v2 = (v1 + 1) % _height;
	double fracu = fu - floor( fu );
	double fracv = fv - floor( fv );
	double w1 = (1 - fracu) * (1 - fracv);
	double w2 = fracu * (1 - fracv);
	double w3 = (1 - fracu) * fracv;
	double w4 = fracu *  fracv;
	Color c1 = _bitmap[v1][u1];
	Color c2 = _bitmap[v1][u2];
	Color c3 = _bitmap[v2][u1];
	Color c4 = _bitmap[v2][u2];
	return c1 * w1 + c2 * w2 + c3 * w3 + c4 * w4;
}

Color Texture::getTexel1(double U, double V )
{
	int fu = U * _width;
	int fv = (1-V) * _height;
	int u1 = ((int)fu) % _width;
	int v1 = ((int)fv) % _height;
	int u2 = (u1 + 1) % _width;
	int v2 = (v1 + 1) % _height;
	double fracu = fu - floor( fu );
	double fracv = fv - floor( fv );
	double w1 = (1 - fracu) * (1 - fracv);
	double w2 = fracu * (1 - fracv);
	double w3 = (1 - fracu) * fracv;
	double w4 = fracu *  fracv;
	Color c1 = _bitmap[v1][u1];
	Color c2 = _bitmap[v1][u2];
	Color c3 = _bitmap[v2][u1];
	Color c4 = _bitmap[v2][u2];
	return c1 * w1 + c2 * w2 + c3 * w3 + c4 * w4;
}
Light::Light( int a_Type, Vec3 a_P1, Vec3 a_P2, Vec3 a_P3, Color a_Color )
{
	_type = a_Type;
	_color = a_Color;
	_grid = new Vec3[16];					//将面光源均分为16块
	_grid[ 0] = Vec3( 1, 2, 0 );			//乱序初始化每个小块的顶点位置
	_grid[ 1] = Vec3( 3, 3, 0 );
	_grid[ 2] = Vec3( 2, 0, 0 );
	_grid[ 3] = Vec3( 0, 1, 0 );
	_grid[ 4] = Vec3( 2, 3, 0 );
	_grid[ 5] = Vec3( 0, 3, 0 );
	_grid[ 6] = Vec3( 0, 0, 0 );
	_grid[ 7] = Vec3( 2, 2, 0 );
	_grid[ 8] = Vec3( 3, 1, 0 );
	_grid[ 9] = Vec3( 1, 3, 0 );
	_grid[10] = Vec3( 1, 0, 0 );
	_grid[11] = Vec3( 3, 2, 0 );
	_grid[12] = Vec3( 2, 1, 0 );
	_grid[13] = Vec3( 3, 0, 0 );
	_grid[14] = Vec3( 1, 1, 0 );
	_grid[15] = Vec3( 0, 2, 0 );
	_CellX = (a_P2 - a_P1) * 0.25f;			//每小格的尺寸
	_CellY = (a_P3 - a_P1) * 0.25f;
	for ( int i = 0; i < 16; i++ )
		_grid[i] = _grid[i].cell[0] * _CellX + _grid[i].cell[1] * _CellY + a_P1;		//标记每个小块的顶点位置
	_pos = a_P1 + 2 * _CellX + 2 * _CellY;												//面光源中心
}

int Triangle::intersect( Ray& ray, double& dist)
{
	Vec3 Rd = ray.getDirection();

	Vec3 E1 = _Vertex[0]->getPos() - _Vertex[1]->getPos();
	Vec3 E2 = _Vertex[0]->getPos() - _Vertex[2]->getPos();
	Vec3 S = _Vertex[0]->getPos() - ray.getOrigin();

	double det0 = Rd.det(E1,E2);
	double det1 = S.det(E1,E2);
	double det2 = Rd.det(S,E2);
	double det3 = Rd.det(E1,S);

	double Dist = det1 / det0;
	double bata = det2 / det0;
	double gama = det3 / det0;
	if( Dist > 0 )
	{
		if( bata >=0 && bata <= 1&& gama>=0 && gama <= 1 && bata+gama <= 1)
			if( Dist < dist)
			{
				dist = Dist;
				return HIT;
			}
	}
	return MISS;
}

Color Triangle::getColor(Vec3& pos)
{
	Color res;
	if (_property->getTexture())
	{
		Texture* t = _property->getTexture();
		double u = DOT( pos, _UAxis ) * _property->getUScale();
		double v = DOT( pos, _VAxis ) * _property->getVScale();
		res = t->getTexel( u, v ) * _property->getColor();
	}
	else
	{
		res = _property->getColor();
	}
	return res;
}

void Triangle::getColorNormal(Color& color, Vec3& normal, Vec3& pos)
{
		double arpha, bata, gama;
		Vec3 P1 = _Vertex[0]->getPos();
		Vec3 P2 = _Vertex[1]->getPos();
		Vec3 P3 = _Vertex[2]->getPos();
		double det0 = P1.det(P2, P3);
		double det1 = pos.det(P2,P3);
		double det2 = P1.det(pos,P3);
		arpha = det1 / det0;
		bata = det2 / det0;
		gama = 1 - arpha - bata;
		normal = _normal[0]->getPos()*arpha + _normal[1]->getPos()*bata + _normal[2]->getPos()*gama ;

		Texture* t = _property->getTexture();
		if(!t)
			color = _property->getColor();
		else
		{
			double u = _au * arpha + _bu * bata + _cu * gama;
			double v = _av * arpha + _bv * bata + _cv * gama;
			if( u > 1.0 || u < 0.0 )
			{
				while( u > 1.0)
					u = u - 1;
				while( u < 0.0)
					u = u + 1;
			}
			if( v > 1.0 || v < 0.0 )
			{
				while( v > 1.0 )
					v = v - 1;
				while( v < 0.0 )
					v = v + 1;
			}
			color = t->getTexel1(u,v);
		}
}

Sphere::Sphere( Vec3 center_, double radius_ ): 
		Object(), _center( center_ ), _radius( radius_ ), _radius2( radius_ * radius_ ), _rradius(1.0 / _radius) 
	{
		_Vn = Vec3( 0, 1, 0 );
		_Ve = Vec3( 1, 0, 0 );
		_Vc = _Vn.cross( _Ve );
		for( int i = 0; i < 3; i++)
		{
			_min[i] = _center.cell[i] - _radius;
			_max[i] = _center.cell[i] + _radius;
		}
	}

int Sphere::intersect( Ray& ray, double& dist )
{
	double Loc2 = SQRDISTANCE( ray.getOrigin(), _center );
	if( Loc2 <= _radius2 )
		return INTER;
	else
	{
		double tca = DOT( ( _center - ray.getOrigin() ) , ray.getDirection() );
		if( tca < 0)
			return MISS;
		else
		{
			double thc2 = _radius2 - Loc2 + tca * tca ;
			if( thc2 < 0 )
				return MISS;
			else
			{
				double Dist = tca - sqrt( thc2 );
				if( Dist > 0 && Dist < dist )
				{
					dist = Dist;
					return HIT;
				}
				else
					return MISS;
			}
		}
	}
}

Color Sphere::getColor( Vec3& pos )
{
	Color res;
	if (!_property->getTexture())
		res = _property->getColor();
	else
	{
		Vec3 vp = (pos - _center) * _rradius;
		double phi;
		double p = -DOT( vp, _Vn );
		if(p > 1.0f)//   ???
			phi = 0;
		else
			if(p < -1.0f)
				phi = PI; 
			else
				phi = acos( p );

		double u, v = phi * _property->getRVScale() * (1.0f / PI);
		double theta = (acos( DOT( _Ve, vp ) / sin( phi ))) * (2.0f / PI);
		if (DOT( _Vc, vp ) >= 0) 
			u = (1.0f - theta) * _property->getRUScale();				     
		else 
			u = theta * _property->getRUScale();
			res = _property->getTexture()->getTexel( u, v ) * _property->getColor();
	}
	return res;
}


void Scene::addBox( Vec3 a_Pos, Vec3 a_Size )
{
	Vertex* v[8];
	v[0] = new Vertex( Vec3( a_Pos.x, a_Pos.y, a_Pos.z ));
	v[1] = new Vertex( Vec3( a_Pos.x + a_Size.x, a_Pos.y, a_Pos.z ));
	v[2] = new Vertex( Vec3( a_Pos.x + a_Size.x, a_Pos.y + a_Size.y, a_Pos.z ));
	v[3] = new Vertex( Vec3( a_Pos.x, a_Pos.y + a_Size.y, a_Pos.z ));
	v[4] = new Vertex( Vec3( a_Pos.x, a_Pos.y, a_Pos.z + a_Size.z ));
	v[5] = new Vertex( Vec3( a_Pos.x + a_Size.x, a_Pos.y, a_Pos.z + a_Size.z ));
	v[6] = new Vertex( Vec3( a_Pos.x + a_Size.x, a_Pos.y + a_Size.y, a_Pos.z + a_Size.z ));
	v[7] = new Vertex( Vec3( a_Pos.x, a_Pos.y + a_Size.y, a_Pos.z + a_Size.z ));
	// front plane
	_object[_objects++] = new Triangle( v[0], v[1], v[3] ,0);
	_object[_objects++] = new Triangle(  v[1], v[2], v[3] ,0);
	// back plane
	_object[_objects++] = new Triangle(  v[5], v[4], v[7] ,0);
	_object[_objects++] = new Triangle(  v[5], v[7], v[6] ,0);
	// left plane
	_object[_objects++] = new Triangle(  v[4], v[0], v[3] ,0 );
	_object[_objects++] = new Triangle(  v[4], v[3], v[7] ,0 );
	// right plane
	_object[_objects++] = new Triangle(  v[1], v[5], v[2] ,0);
	_object[_objects++] = new Triangle(  v[5], v[6], v[2] ,0 );
	// top 
	_object[_objects++] = new Triangle(  v[4], v[5], v[1] ,0);
	_object[_objects++] = new Triangle(  v[4], v[1], v[0] ,0 );
	// bottom plane
	_object[_objects++] = new Triangle(  v[6], v[7], v[2] ,0);
	_object[_objects++] = new Triangle(  v[7], v[3], v[2] ,0);
}

void Scene::addPlane( Vec3 a_P1, Vec3 a_P2, Vec3 a_P3, Vec3 a_P4, Property* proper)
{
	Vertex* v[4];
	v[0] = new Vertex( a_P1 );
	v[1] = new Vertex( a_P2 );
	v[2] = new Vertex( a_P3 );
	v[3] = new Vertex( a_P4 );
	_object[_objects] = new Triangle(v[0], v[1], v[3] ,0);
	_object[_objects++]->setProperty( proper );
	_object[_objects] = new Triangle(v[1], v[2], v[3] ,0);
	_object[_objects++]->setProperty( proper );
}

void Scene::initScene0()
{
	_object = new Object*[300000];
	_light = new Light*[10];
		
	//_light[0] = new Light( Light::AREA, Vec3(-1,11,4), Vec3(1,11,4), Vec3(-1,11,6), Color(0.7,0.7,0.7) ); 
	//_light[1] = new Light( Light::AREA, Vec3(-1,11,-1), Vec3(1,11,-1), Vec3(-1,11,1), Color(0.7,0.7,0.7) );
	_light[0] = new Light( Light::POINT, Vec3(-5,11,0), Color(0.9,0.9,0.9));
	_light[1] = new Light( Light::POINT, Vec3(5,11,0), Color(0.9,0.9,0.9));
	_lights = 2;
	
	_object[0] = new Sphere( Vec3( 0, -1.5, -6 ), 1.5f );
	_object[0]->getProperty()->setParameters(0.8, 0, Color( 0.7f, 0.7f, 1.0f ) );

	int ob = 0;
	
	std::ifstream fin("obj/dragon.obj");

	int v, f;
	char c;
	fin>>c;
	fin>>v>>f;

	Vertex** vertex = new Vertex*[v];
	double x, y, z;
	for(int i = 0; i < v; i++)
	{
		fin>>c>>x>>y>>z;
		vertex[i] = new Vertex(Vec3(x*7,z*7,y*7));
	}
	int v1, v2, v3;
	for(int i = 0; i < f; i++)
	{
		fin>>c>>v1>>v2>>v3;
		_object[ob] = new Triangle(vertex[v1-1], vertex[v2-1], vertex[v3-1], 1);
		_object[ob]->getProperty()->setReflection( 0.0f );
		_object[ob]->getProperty()->setRefraction( 0.0f );
		_object[ob]->getProperty()->setDiffuse( 0.8f );
		_object[ob]->getProperty()->setColor( Color( 0.5, 0.5, 0.8 ) );
		//_object[ob]->getProperty()->setTexture( new Texture( "textures/foliage.jpg" ) );
		//_object[ob]->getProperty()->setUVScale( 0.1f, 0.1f );
		ob++;
	}
	_objects = ob;
	
	Property * proper = new Property();
	proper->setParameters(0.6, 0, Color(0.4,0.4,0.4) );
	addPlane(Vec3(-19,-3,-19), Vec3(19,-3,-19), Vec3(19,-3,19), Vec3(-19,-3,19), proper);

	_kdTree = new KdTree();
	_kdTree->build(this);
	std::cout<<"Kdtree build end"<<std::endl;
}

void Scene::initScene1()
{
	_object = new Object*[300000];
	_light = new Light*[10];

	//_light[0] = new Light( Light::AREA, Vec3(-1,11,4), Vec3(1,11,4), Vec3(-1,11,6), Color(0.7,0.7,0.7) ); 
	//_light[1] = new Light( Light::AREA, Vec3(-1,11,-1), Vec3(1,11,-1), Vec3(-1,11,1), Color(0.7,0.7,0.7) );
	_light[0] = new Light( Light::POINT, Vec3(-5,11,0), Color(0.9,0.9,0.9));
	_light[1] = new Light( Light::POINT, Vec3(5,11,0), Color(0.9,0.9,0.9));
	_lights = 2;
	
	int ob = 0;
	
	std::ifstream fin("obj/dinosaur.2k.obj");

	int v=0;
	char c;

	Vertex** vertex = new Vertex*[300000];
	double x, y, z;
	int v1, v2, v3;
	while(!fin.eof())
	{
		fin>>c;
		if(c == 'v')
		{
			fin>>x>>y>>z;
			vertex[v++] = new Vertex(Vec3(x/10,z/10,y/10));
		}
		else
		{
			fin>>v1>>v2>>v3;
			_object[ob] = new Triangle(vertex[v1-1], vertex[v2-1], vertex[v3-1],1);
			//Property* pro = new Property();
			//_object[ob]->setProperty(pro);
			_object[ob]->getProperty()->setReflection( 0.0f );
			_object[ob]->getProperty()->setRefraction( 0.0f );
			_object[ob]->getProperty()->setDiffuse( 0.8f );
			_object[ob]->getProperty()->setColor( Color( 0.6, 0.6, 0.6 ) );
			//_object[ob]->getProperty()->setTexture( new Texture( "textures/foliage.jpg" ) );
			//_object[ob]->getProperty()->setUVScale( 0.1f, 0.1f );
			ob++;
		}
	}
	_objects = ob;
	
	/*Property * proper = new Property();
	addPlane(Vec3(-19,-4,-19), Vec3(19,-4,-19), Vec3(19,-4,19), Vec3(-19,-4,19), proper);*/

	// ground plane
	Property* proper = new Property();
	proper->setParameters(0.6, 0, Color(0.4,0.4,0.4) );
	addPlane(Vec3(-19,-6,-19), Vec3(19,-6,-19), Vec3(19,-6,19), Vec3(-19,-6,19), proper);
	/*// left plane
	Property* proper1 = new Property();
	proper1->setParameters(0.6, 0, Color(0.4,0.4,0.4) );
	addPlane(Vec3(-10,-19,-19), Vec3(-10,19,-19), Vec3(-10,19,19), Vec3(-10,-19,19), proper1);
	// right plane
	Property* proper2 = new Property();
	proper2->setParameters(0.6, 0, Color(0.4,0.4,0.4) );
	addPlane(Vec3(10,-19,-19), Vec3(10,19,-19), Vec3(10,19,19), Vec3(10,-19,19), proper2);
	//back plane
	Property* proper3 = new Property();
	proper3->setParameters(0.6, 0, Color(0.4,0.4,0.4) );
	addPlane(Vec3(-19,-19,20), Vec3(-19,19,20), Vec3(19,19,20), Vec3(19,-19,20), proper3);*/

	_kdTree = new KdTree();
	_kdTree->build(this);
	std::cout<<"Kdtree build end"<<std::endl;
}

void Scene::initScene2()
{
	_object = new Object*[500];
	_light = new Light*[10]; 

	// big sphere
	_object[0] = new Sphere( Vec3( 2, 0.8f, 3 ), 2.5f );
	_object[0]->getProperty()->setParameters(0.1, 1.0, Color( 0.7f, 0.7f, 1.0f ), 1.3);
	
	// small sphere
	_object[1] = new Sphere( Vec3( -5.5f, -0.5, 7 ), 2 );
	_object[1]->getProperty()->setParameters(0.5, 0, Color( 0.7f, 0.7f, 1.0f ), 1.3, 0.1);

	// light source 1
	_light[0] = new Light( Light::POINT, Vec3( 0, 5, 5 ), Color( 0.6f, 0.6f, 0.6f ) );
	// light source 2
	_light[1] = new Light( Light::POINT, Vec3( -3, 5, 1 ), Color( 0.6f, 0.6f, 0.8f ) );
	
	//_light[2] = new Light( Light::AREA, Vec3(-2,4,-2), Vec3(-2,4,2), Vec3(2,4,-2), Color(0.6,0.6,0.6) ); 
	_lights = 2;
	
	// extra sphere
	_object[2] = new Sphere( Vec3( -1.5f, -3.8f, 1 ), 1.5f );
	_object[2]->getProperty()->setParameters(0.8, 0, Color( 1.0f, 0.4f, 0.4f ) );

	int ob = 3;
	for ( int x = 0; x < 8; x++ ) for ( int y = 0; y < 7; y++ )
	{
		_object[ob] = new Sphere( Vec3( -4.5f + x * 1.5f, -4.3f + y * 1.5f, 10 ), 0.3f );
		_object[ob]->getProperty()->setReflection( 0 );
		_object[ob]->getProperty()->setRefraction( 0 );
		_object[ob]->getProperty()->setSpecular( 0.6f );
		_object[ob]->getProperty()->setDiffuse( 0.6f );
		_object[ob]->getProperty()->setColor( Color( 0.3f, 1.0f, 0.4f ) );
		ob++;
	}
	for ( int x = 0; x < 8; x++ ) for ( int y = 0; y < 8; y++ )
	{
		_object[ob] = new Sphere( Vec3( -4.5f + x * 1.5f, -4.3f, 10.0f - y * 1.5f ), 0.3f );
		_object[ob]->getProperty()->setReflection( 0 );
		_object[ob]->getProperty()->setRefraction( 0 );
		_object[ob]->getProperty()->setSpecular( 0.6f );
		_object[ob]->getProperty()->setDiffuse( 0.6f );
		_object[ob]->getProperty()->setColor( Color( 0.3f, 1.0f, 0.4f ) );
		ob++;
	}
	for ( int x = 0; x < 16; x++ ) for ( int y = 0; y < 8; y++ )
	{
		_object[ob] = new Sphere( Vec3( -8.5f + x * 1.5f, 4.3f, 10.0f - y ), 0.3f );
		_object[ob]->getProperty()->setReflection( 0 );
		_object[ob]->getProperty()->setRefraction( 0 );
		_object[ob]->getProperty()->setSpecular( 0.6f );
		_object[ob]->getProperty()->setDiffuse( 0.6f );
		_object[ob]->getProperty()->setColor( Color( 0.3f, 1.0f, 0.4f ) );
		ob++;
	}
	_objects = ob;

	// ground plane

	Property* pro1 = new Property();
	pro1->setParameters(0, 0, Color( 0.4f, 0.3f, 0.3f ), 1.0, 1.0 );
	addPlane( Vec3(-19,-7,-19), Vec3(-19,-7,19), Vec3(19,-7,19), Vec3(19,-7,-19), pro1);


	// back plane
	Property* pro2 = new Property();
	pro1->setParameters(0, 0, Color( 0.5, 0.3, 0.5 ), 1.0, 0.6 );
	addPlane( Vec3(-19,-19,12), Vec3(-19,19,12), Vec3(19,19,12), Vec3(19,-19,12), pro2);
	
	// ceiling plane
	Property* pro3 = new Property();
	pro1->setParameters(0, 0, Color( 0.4f, 0.7f, 0.7f ), 1.0, 0.5, 0 );
	addPlane( Vec3(-19,7.4,-19), Vec3(-19,7.4,19), Vec3(19,7.4,19), Vec3(19,7.4,-19), pro3);

	_kdTree = new KdTree();
	_kdTree->build(this);
	std::cout<<"Kdtree build end"<<std::endl;
}

void Scene::initScene3()
{
	setExtend( Box( Vec3(-20,-20,-20), Vec3(40,50,40)) );
	_object = new Object*[300000];
	_light = new Light*[10];
		
	//_light[0] = new Light( Light::AREA, Vec3(-1,7,-1), Vec3(-1,7,1), Vec3(1,7,-1), Color(0.7,0.7,0.7) ); 
	//_light[1] = new Light( Light::AREA, Vec3(-3,7,-3), Vec3(-1,7,-3), Vec3(-3,7,-1), Color(0.7,0.7,0.7) );
	_light[0] = new Light( Light::POINT, Vec3(-2,8,0), Color(0.8,0.8,0.8));
	_light[1] = new Light( Light::POINT, Vec3(2,8,0), Color(0.8,0.8,0.8));
	//_light[2] = new Light( Light::POINT, Vec3(3,4,6), Color(0.4,0.4,0.4));
	_lights = 2;
	
	_object[0] = new Sphere( Vec3( 3, -4.5, 2 ), 1.5 );
	//_object[0]->getProperty()->setParameters(0.6, 0, Color( 0.7, 0.7, 1.0 ), 1, 0.2);
	/*_object[0]->getProperty()->setTexture(new Texture("textures2/dots.png") );
	_object[0]->getProperty()->setUVScale(0.5, 0.5);*/
	_object[0]->getProperty()->setParameters(0.5, 0, Color( 1.0, 1.0, 1.0 ), 1, 0.1);//0.7,0.7,1.0
	//_object[0]->getProperty()->setDiffuseReflection(0.1);
	

	_object[1] = new Sphere( Vec3( -1, 1, 5), 2.5 );
	//_object[1]->getProperty()->setParameters(0, 0.6, Color( 0.7, 0.7, 0.7 ), 1.1);
	/*_object[1]->getProperty()->setTexture(new Texture("textures2/marble2.jpg") );
	_object[1]->getProperty()->setUVScale(0.5, 0.5);*/
	_object[1]->getProperty()->setParameters(0, 0.6, Color( 0.3, 0.3, 0.3 ), 1.3);
	int ob = 2;

	for( int i = 0 ; i < 5 ;i ++)
		for( int j = 0; j < 5; j++)
		{
			_object[ob] = new Sphere( Vec3( -1+i*1.5, -2+j*1.5, 10), 0.3);
			_object[ob++]->getProperty()->setParameters(0, 0, Color(0.2,0.6,0.2), 1, 0.6, 0.6);
			//_object[ob++]->getProperty()->setParameters(0, 0, Color(0.2,0.6,0.2), 1, 0.6, 0);
		}

	std::ifstream fin("../obj/dragon.obj");
	int v, f;
	char c;
	fin>>c;
	fin>>v>>f;
	Vertex** vertex = new Vertex*[v];
	double x, y, z;
	for(int i = 0; i < v; i++)
	{
		fin>>c>>x>>y>>z;
		vertex[i] = new Vertex(Vec3(y*5-4,z*5-4.2,x*5+4));
	}
	int v1, v2, v3;
	for(int i = 0; i < f; i++)
	{
		fin>>c>>v1>>v2>>v3;
		_object[ob] = new Triangle(vertex[v1-1], vertex[v2-1], vertex[v3-1], 0);
		_object[ob]->getProperty()->setReflection( 0.0f );
		_object[ob]->getProperty()->setRefraction( 0.0f );
		_object[ob]->getProperty()->setDiffuse( 0.8f );
		_object[ob]->getProperty()->setColor( Color( 0.5, 0.5, 0.8 ) );
		//_object[ob]->getProperty()->setTexture( new Texture( "textures/foliage.jpg" ) );
		//_object[ob]->getProperty()->setUVScale( 0.1f, 0.1f );
		ob++;
	}
	_objects = ob;

	// ground plane
	Property* proper = new Property();
	proper->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.8, 0);
	//proper->setDiffuseReflection(0.3);
	proper->setTexture( new Texture( "../textures/wood.jpg" ));
	proper->setUVScale(0.1, 0.1);
	addPlane(Vec3(-19,-6,-19), Vec3(19,-6,-19), Vec3(19,-6,19), Vec3(-19,-6,19), proper);
	// left plane
	Property* proper1 = new Property();
	proper1->setParameters(0.6, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	//proper1->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	proper1->setTexture( new Texture( "../textures/marble.jpg" ));
	proper1->setUVScale(1, 1);
	addPlane(Vec3(-10,-19,-19), Vec3(-10,29,-19), Vec3(-10,29,19), Vec3(-10,-19,19), proper1);
	// right plane
	Property* proper2 = new Property();
	proper2->setParameters(0.6, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	//proper2->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	proper2->setTexture( new Texture( "../textures/marble.jpg" ));
	proper2->setUVScale(1, 1);
	addPlane(Vec3(10,-19,-19), Vec3(10,29,-19), Vec3(10,29,19), Vec3(10,-19,19), proper2);
	//back plane
	Property* proper3 = new Property();
	proper3->setParameters(0.8, 0, Color(0.4,0.4,0.4), 1, 0.2, 0 );
	//proper3->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.2, 0 );
	addPlane(Vec3(-19,-19,15), Vec3(-19,29,15), Vec3(19,29,15), Vec3(19,-19,15), proper3);
	//ceiling
	Property* proper4 = new Property();
	proper4->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.5, 0 );
	proper4->setTexture( new Texture( "../textures/marble.jpg" ));
	proper4->setUVScale(0.5, 0.5);
	addPlane(Vec3(-19,20,-19), Vec3(19,20,-19), Vec3(19,20,19), Vec3(-19,20,19), proper4);
	//front plane
	Property* proper6 = new Property();
	proper6->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.5, 0 );
	proper6->setTexture( new Texture( "../textures/marble.jpg" ));
	proper6->setUVScale(0.5, 0.5);
	addPlane(Vec3(-19,-19,-12), Vec3(-19,29,-12), Vec3(19,29,-12), Vec3(19,-19,-12), proper6);

	_kdTree = new KdTree();
	_kdTree->build(this);
	std::cout<<"Kdtree build end"<<std::endl;
}

void Scene::initScene4()
{
	setExtend( Box( Vec3(-20,-20,-20), Vec3(40,50,40)) );
	_object = new Object*[300000];
	_light = new Light*[10];
		
	//_light[0] = new Light( Light::AREA, Vec3(-1,11,2), Vec3(1,11,2), Vec3(-1,11,4), Color(0.7,0.7,0.7) ); 
	//_light[1] = new Light( Light::AREA, Vec3(-1,11,-1), Vec3(1,11,-1), Vec3(-1,11,1), Color(0.7,0.7,0.7) );
	_light[0] = new Light( Light::POINT, Vec3(-2,8,0), Color(0.9,0.9,0.9));
	_light[1] = new Light( Light::POINT, Vec3(2,8,5), Color(0.9,0.9,0.9));
	_lights = 2;
	
	int ob = 0;

	std::ifstream fin("Mac/Mac.txt");
	//std::ifstream fin("Mac/Mac.txt");
	int v=1, t=1;
	char s[5];
	char c1,c2,c3,c4,c5,c6;
	double x=0, y=0, z=0;
	double us=0, vs=0;
	int x1,x2,x3, y1,y2,y3, z1,z2,z3;
	double r, g, b;
	char pic[40];
	std::vector<Vertex*> vertex;
	std::vector<Vertex*> normal;
	std::vector<Texture*> texture;
	std::vector<double> U;
	std::vector<double> V;
	v = 1;

	while (!fin.eof()) 
	{
		fin >>s;	
		if( s[0] == 'v' && s[1] == 0 )
		{
			fin >>x>>y>>z;
			vertex.push_back(new Vertex(Vec3(x/40,y/40-3,z/40+2)) );
		}
		else if( s[0] == 'v' && s[1] == 't')
		{
			fin>>us>>vs;
			U.push_back(us);
			V.push_back(vs);
		}
		else if( s[0] == 'v' && s[1] == 'n')
		{
			fin>>x>>y>>z;
			normal.push_back(new Vertex( Vec3(x, y, z) ));
		}
		else if( s[0] == 'f')
		{
			fin>>x1>>c1>>y1>>c2>>z1>>x2>>c3>>y2>>c4>>z2>>x3>>c5>>y3>>c6>>z3 >>r>>g>>b>>pic ;
			_object[ob] = new Triangle(vertex[x1-1], vertex[x2-1], vertex[x3-1], 1);
			_object[ob]->getProperty()->setColor( Color( r, g, b ) );
			((Triangle* )_object[ob])->setNormals( normal[z1-1], normal[z2-1], normal[z3-1]);
			if( pic[0] != '0' )
			{
				int f = -1;
				for(int i = 0; i < texture.size(); i++)
					if( strcmp(texture[i]->getName(), pic ) == 0 )
					{
						f = i;
						break;
					}
				if( f == -1)
				{
					texture.push_back(new Texture(pic) );
					_object[ob]->getProperty()->setTexture( texture.back() );
				}
				else
					_object[ob]->getProperty()->setTexture( texture[f] );
				((Triangle* )_object[ob])->setTextureScale( U[y1-1], V[y1-1], U[y2-1], V[y2-1], U[y3-1], V[y3-1]);
			}
			ob++;
		}
	}

	_objects = ob;

	// ground plane
	Property* proper = new Property();
	proper->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.8, 0);
	proper->setTexture( new Texture( "textures/wood.jpg" ));
	proper->setUVScale(0.5, 0.5);
	addPlane(Vec3(-19,-6,-19), Vec3(19,-6,-19), Vec3(19,-6,19), Vec3(-19,-6,19), proper);

	_kdTree = new KdTree();
	_kdTree->build(this);
	std::cout<<"Kdtree build end"<<std::endl;
}

void Scene::initScene5()
{
	setExtend( Box( Vec3(-20,-20,-20), Vec3(40,50,40)) );
	_object = new Object*[300000];
	_light = new Light*[10];
		
	_light[0] = new Light( Light::AREA, Vec3(-1,7,-1), Vec3(-1,7,1), Vec3(1,7,-1), Color(0.7,0.7,0.7) ); 
	_light[1] = new Light( Light::AREA, Vec3(-3,7,-3), Vec3(-1,7,-3), Vec3(-3,7,-1), Color(0.7,0.7,0.7) );
	//_light[0] = new Light( Light::POINT, Vec3(-2,8,0), Color(0.8,0.8,0.8));
	//_light[1] = new Light( Light::POINT, Vec3(2,8,0), Color(0.8,0.8,0.8));
	//_light[2] = new Light( Light::POINT, Vec3(3,4,6), Color(0.4,0.4,0.4));
	_lights = 2;
	
	int ob = 0;

	std::ifstream fin("Optimus/Optimus.txt");
	char s[5];
	char c1,c2,c3,c4,c5,c6;
	double x=0, y=0, z=0;
	double us=0, vs=0;
	int x1,x2,x3, y1,y2,y3, z1,z2,z3;
	double r, g, b;
	char pic[40];
	std::vector<Vertex*> vertex;
	std::vector<Vertex*> normal;
	std::vector<Texture*> texture;
	std::vector<double> U;
	std::vector<double> V;
	Vec3 X(1, 0, 0);
	Vec3 Y(0, 1, 0);
	Vec3 Z(0, 0, -1);
	//Vec3 O(-4, -6, 3);
	Vec3 O(0, -6, 2.5);

	while (!fin.eof()) 
	{
		fin >>s;	
		if( s[0] == 'v' && s[1] == 0 )
		{
			fin >>x>>y>>z;
			vertex.push_back(new Vertex( O + X*x*0.37 + Y*y*0.37 +Z*z*0.37) );
		}
		else if( s[0] == 'v' && s[1] == 't')
		{
			fin>>us>>vs;
			U.push_back(us);
			V.push_back(vs);
		}
		else if( s[0] == 'v' && s[1] == 'n')
		{
			fin>>x>>y>>z;
			normal.push_back(new Vertex( X*x+Y*y+Z*z) );
		}
		else if( s[0] == 'f')
		{
			fin>>x1>>c1>>y1>>c2>>z1>>x2>>c3>>y2>>c4>>z2>>x3>>c5>>y3>>c6>>z3 >>r>>g>>b>>pic ;
			_object[ob] = new Triangle(vertex[x1-1], vertex[x2-1], vertex[x3-1], 1);
			_object[ob]->getProperty()->setColor( Color( r, g, b ) );
			_object[ob]->getProperty()->setDiffuse(1.0);
			((Triangle* )_object[ob])->setNormals( normal[z1-1], normal[z2-1], normal[z3-1]);
			if( pic[0] != '0' )
			{
				int f = -1;
				for(int i = 0; i < texture.size(); i++)
					if( strcmp(texture[i]->getName(), pic ) == 0 )
					{
						f = i;
						break;
					}
				if( f == -1)
				{
					texture.push_back(new Texture(pic) );
					_object[ob]->getProperty()->setTexture( texture.back() );
				}
				else
					_object[ob]->getProperty()->setTexture( texture[f] );
				((Triangle* )_object[ob])->setTextureScale( U[y1-1], V[y1-1], U[y2-1], V[y2-1], U[y3-1], V[y3-1]);
			}
			ob++;
		}
	}

	_objects = ob;

	// ground plane
	Property* proper = new Property();
	proper->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.8, 0);
	//proper->setDiffuseReflection(0.3);
	proper->setTexture( new Texture( "textures/wood.jpg" ));
	proper->setUVScale(0.1, 0.1);
	addPlane(Vec3(-19,-6,-19), Vec3(19,-6,-19), Vec3(19,-6,19), Vec3(-19,-6,19), proper);
	// left plane
	Property* proper1 = new Property();
	proper1->setParameters(0.6, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	//proper1->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	proper1->setTexture( new Texture( "textures/marble.jpg" ));
	proper1->setUVScale(1, 1);
	addPlane(Vec3(-10,-19,-19), Vec3(-10,29,-19), Vec3(-10,29,19), Vec3(-10,-19,19), proper1);
	// right plane
	Property* proper2 = new Property();
	proper2->setParameters(0.6, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	//proper2->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.7, 0 );
	proper2->setTexture( new Texture( "textures/marble.jpg" ));
	proper2->setUVScale(1, 1);
	addPlane(Vec3(10,-19,-19), Vec3(10,29,-19), Vec3(10,29,19), Vec3(10,-19,19), proper2);
	//back plane
	Property* proper3 = new Property();
	proper3->setParameters(0.8, 0, Color(0.4,0.4,0.4), 1, 0.2, 0 );
	//proper3->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.2, 0 );
	addPlane(Vec3(-19,-19,15), Vec3(-19,29,15), Vec3(19,29,15), Vec3(19,-19,15), proper3);
	//ceiling
	Property* proper4 = new Property();
	proper4->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.5, 0 );
	proper4->setTexture( new Texture( "textures/marble.jpg" ));
	proper4->setUVScale(0.5, 0.5);
	addPlane(Vec3(-19,20,-19), Vec3(19,20,-19), Vec3(19,20,19), Vec3(-19,20,19), proper4);
	//front plane
	Property* proper6 = new Property();
	proper6->setParameters(0, 0, Color(0.4,0.4,0.4), 1, 0.5, 0 );
	proper6->setTexture( new Texture( "textures/marble.jpg" ));
	proper6->setUVScale(0.5, 0.5);
	addPlane(Vec3(-19,-19,-12), Vec3(-19,29,-12), Vec3(19,29,-12), Vec3(19,-19,-12), proper6);

	_kdTree = new KdTree();
	_kdTree->build(this);
	std::cout<<"Kdtree build end"<<std::endl;
}

};