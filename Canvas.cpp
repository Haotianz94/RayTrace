#include "canvas.h"
#include <ctime>
#include <cstdlib>
#include<thread>
  
using namespace cv;
using namespace std;

namespace Raytracer {

int Canvas::findNearest( Ray& ray, double& dist, Object*& ob )
{
	int sect = 0;
	double tnear = 0;										//进入盒子点到光源距离
	double tfar = dist;										//出盒子点到光源距离
	double t;												//分割面到光源距离
	int retval = 0;
	Vec3 p1 = _scene->getExtend().getPos();					//左下顶点
	Vec3 p2 = p1 + _scene->getExtend().getSize();			//右上顶点
	Vec3 D = ray.getDirection(), O = ray.getOrigin();

	for ( int i = 0; i < 3; i++ )							//光线远离盒子，不可能找到相交物体 
		if (D.cell[i] < 0) 
		{
			if (O.cell[i] < p1.cell[i]) 
				return 0;
		}
		else
			if (O.cell[i] > p2.cell[i])
				return 0;
	
	// 计算进盒子和出盒子点的准确位置？？？
	for ( int i = 0; i < 3; i++ )
	{
		double pos = O.cell[i] + tfar * D.cell[i];
		if (D.cell[i] < 0)
		{
			// clip end point
			if (pos < p1.cell[i])
				tfar = tnear + (tfar - tnear) * ((O.cell[i] - p1.cell[i]) / (O.cell[i] - pos));
			// clip start point
			if (O.cell[i] > p2.cell[i])
				tnear += (tfar - tnear) * ((O.cell[i] - p2.cell[i]) / (tfar * D.cell[i]));
		}
		else
		{
			// clip end point
			if (pos > p2.cell[i])
				tfar = tnear + (tfar - tnear) * ((p2.cell[i] - O.cell[i]) / (pos - O.cell[i]));
			// clip start point
			if (O.cell[i] < p1.cell[i])
				tnear += (tfar - tnear) * ((p1.cell[i] - O.cell[i]) / (tfar * D.cell[i]));
		}
		if (tnear > tfar) 
			return 0;
	}

	// 初始化进出点栈
	int entrypoint = 0, exitpoint = 1;
	//稍后遍历的节点， 当前节点
	KdTreeNode* farchild, *currnode;
	//从根节点开始
	currnode = _scene->getKdTree()->getRoot();	
	_stack[entrypoint].t = tnear;							//进入点距离
	if (tnear > 0.0f) 
		_stack[entrypoint].pb = O + D * tnear;				//光源在盒外，计算进入点位置
	else 
		_stack[entrypoint].pb = O;							//光源在盒内，进入点记为光源
	_stack[exitpoint].t = tfar;								//出盒点距离
	_stack[exitpoint].pb = O + D * tfar;					//出盒点位置
	_stack[exitpoint].node = 0;								//无后续遍历节点
	
	// 遍历Kd树
	while (currnode)
	{
		while (!currnode->isLeaf())							//未到叶节点
		{
			double splitpos = currnode->getSplitPos();
			int axis = currnode->getAxis();
			if (_stack[entrypoint].pb.cell[axis] <= splitpos)
			{
				if (_stack[exitpoint].pb.cell[axis] <= splitpos)
				{
					currnode = currnode->getLeft();						//只访问左孩子
					continue;
				}
				if (_stack[exitpoint].pb.cell[axis] == splitpos)
				{
					currnode = currnode->getRight();					//只访问右孩子
					continue;
				}
				farchild = currnode->getRight();						//右孩子入栈，稍后访问
				currnode = currnode->getLeft();							//先访问左孩子					

			}
			else
			{
				if (_stack[exitpoint].pb.cell[axis] > splitpos)
				{
					currnode = currnode->getRight();					//只访问右孩子
					continue;
				}
				farchild = currnode->getLeft();							//左孩子入栈，稍后访问
				currnode = currnode->getRight();						//先访问右孩子
			}

			int tmp = exitpoint++;										//把新的出盒点（与分割面的交点）压栈
			if (exitpoint == entrypoint) exitpoint++;
			_stack[exitpoint].prev = tmp;								//指向上一个出盒点					
			t = (splitpos - O.cell[axis]) / D.cell[axis];				//计算光源到新出盒点距离			
			_stack[exitpoint].t = t;
			_stack[exitpoint].node = farchild;							//exitpoint实为farchild的进盒点
			_stack[exitpoint].pb.cell[axis] = splitpos;					//计算新的出盒点位置
			int nextaxis = _Mod[axis + 1];
			int prevaxis = _Mod[axis + 2];
			_stack[exitpoint].pb.cell[nextaxis] = O.cell[nextaxis] + t * D.cell[nextaxis];
			_stack[exitpoint].pb.cell[prevaxis] = O.cell[prevaxis] + t * D.cell[prevaxis];
		}
		//已经到达叶节点，开始求交
		ListNode* list = currnode->getList();
		double dist_ = _stack[exitpoint].t;								//求交距离不能超过本盒子的出盒点距离
		while (list)
		{
			Object* ob_ = list->getObject();
			int result;
			if (result = ob_->intersect( ray, dist_ ))
			{
				retval = result;
				dist = dist_;
				ob = ob_;
			}
			list = list->getNext();
			sect++;
		}
		if (retval)														//求交成功，直接返回
		{
			return retval;
		}
		entrypoint = exitpoint;											//出盒点成为新的进盒点
		currnode = _stack[exitpoint].node;								//找到farchild
		exitpoint = _stack[entrypoint].prev;							//上一个出盒点出栈
	}
	return 0;															//求交失败
}

int Canvas::findOccluder( Ray& ray, double& dist )								//寻找光源外dist距离内是否有别的物体
{
	double tnear = EPSILON, t;
	Vec3 O, D = ray.getDirection();

	int entrypoint = 0, exitpoint = 1;
	KdTreeNode* farchild, *currnode = _scene->getKdTree()->getRoot();
	_stack[entrypoint].t = tnear;
	_stack[entrypoint].pb = O = ray.getOrigin();
	_stack[exitpoint].t = dist;
	_stack[exitpoint].pb = O + D * dist;
	_stack[exitpoint].node = 0;

	while (currnode)
	{
		while (!currnode->isLeaf())
		{
			double splitpos = currnode->getSplitPos();
			int axis = currnode->getAxis();
			if (_stack[entrypoint].pb.cell[axis] <= splitpos)
			{
				if (_stack[exitpoint].pb.cell[axis] <= splitpos)
				{
					currnode = currnode->getLeft();
					continue;
				}
				if (_stack[exitpoint].pb.cell[axis] == splitpos)
				{
					currnode = currnode->getRight();
					continue;
				}
				farchild = currnode->getRight();
				currnode = currnode->getLeft();
			}
			else
			{
				if (_stack[exitpoint].pb.cell[axis] > splitpos)
				{
					currnode = currnode->getRight();
					continue;
				}
				farchild = currnode->getLeft();
				currnode = currnode->getRight();
			}

			int tmp = exitpoint;
			if (++exitpoint == entrypoint)
				exitpoint++;
			_stack[exitpoint].prev = tmp;
			t = (splitpos - O.cell[axis]) / D.cell[axis];
			_stack[exitpoint].t = t;
			_stack[exitpoint].node = farchild;
			_stack[exitpoint].pb.cell[axis] = splitpos;
			int nextaxis = _Mod[axis + 1];
			int prevaxis = _Mod[axis + 2];
			_stack[exitpoint].pb.cell[nextaxis] = O.cell[nextaxis] + t * D.cell[nextaxis];
			_stack[exitpoint].pb.cell[prevaxis] = O.cell[prevaxis] + t * D.cell[prevaxis];
		}
		ListNode* list = currnode->getList();
		while (list)
		{
			if (list->getObject()->intersect( ray, dist )) 
				return 1;
			list = list->getNext();
		}
		entrypoint = exitpoint;
		currnode = _stack[exitpoint].node;
		exitpoint = _stack[entrypoint].prev;
	}
	return 0;
}

Object* Canvas::rayTrace( Ray& ray, Color& color, int depth, double refrIndex, double& dist, double samples )
{
	dist = 10000.0;
	Object * ob = 0;

	int result;	
	if (!(result = findNearest( ray, dist, ob )) )
		return 0;

	Vec3 P = ray.getOrigin() + ray.getDirection() * dist;
	Vec3 N;
	Color color_P;

	if( ob->getType() == Object::TRIANGLE && ((Triangle*)ob)->isModel() )
	{
		((Triangle*)ob)->getColorNormal(color_P, N, P);
	}
	else
	{
		N = ob->getNormal( P );
		if(ob->getType() == Object::TRIANGLE)
			if( DOT( ray.getDirection(), N) > 0)
				N = -N;
		color_P = ob->getColor(P);
	}
		
		for ( int i = 0; i < _scene->getLights(); i++ )
		{
			Light* light = _scene->getLight( i );
				//计算阴影
				Vec3 L;
				double shade = calculateShade( light, P, L, samples );
				if (shade > 0)
				{
					//计算光源的漫反射
					if (ob->getProperty()->getDiffuse() > 0)
					{
						double cos = DOT( N, L );
						if (cos > 0)
						{
							double diff = cos * ob->getProperty()->getDiffuse() * shade;
							color += diff * color_P * light->getColor();
						}
					}

					//计算高光
					if (ob->getProperty()->getSpecular() > 0)
					{
						Vec3 V = ray.getDirection();
						Vec3 R = L - 2.0 * DOT( L, N ) * N;
						NORMALIZE( R );
						double cos= DOT( V, R );
						if (cos > 0)
						{
							//double spec = powf( cos, 20 ) * ob->getProperty()->getSpecular() * shade;
							double spec = cos * ob->getProperty()->getSpecular() * shade / (50 - 50 * cos + cos);
							color += spec * light->getColor();
						}
					}

				}
		}

		//计算反射
		double refl = ob->getProperty()->getReflection();
		if (refl > 0 && depth < TRACEDEPTH)
		{
			double drefl = ob->getProperty()->getDiffuseReflection();
			if ((drefl > 0) && (depth < 3))
			{
				// 计算环境光的漫反射
				Vec3 RP = ray.getDirection() - 2.0 * DOT( ray.getDirection(), N ) * N;
				Vec3 RN1 = Vec3( RP.z, RP.y, -RP.x );
				Vec3 RN2 = RP.cross( RN1 );
				refl = refl / SAMPLES ;
				for ( int i = 0; i < SAMPLES; i++ )
				{
					double xoffs, yoffs;
					do
					{
						xoffs = 1.0*rand()/RAND_MAX * drefl;
						yoffs = 1.0*rand()/RAND_MAX * drefl;
					}
					while ((xoffs * xoffs + yoffs * yoffs) > (drefl * drefl));
					Vec3 R = RP + RN1 * xoffs + RN2 * yoffs * drefl;
					NORMALIZE( R );
					double drefldist;
					Color dreflColor( 0, 0, 0 );
					rayTrace( Ray( P + R * EPSILON, R ), dreflColor, depth + 1, refrIndex, drefldist, samples*0.25 );
					color += refl * dreflColor * color_P;
				}
			}
			else
			{
				// 计算普通反射
				Vec3 R = ray.getDirection() - 2.0 * DOT( ray.getDirection(), N ) * N;
				Color reflColor( 0, 0, 0 );
				double reflDist;
				rayTrace( Ray( P + R * EPSILON , R), reflColor, depth + 1, refrIndex, reflDist, samples*0.5 );
				color += refl * reflColor * color_P;
			}
		}

		//计算透射
		double refr = ob->getProperty()->getRefraction();
		if ((refr > 0) && (depth < TRACEDEPTH))
		{
			double rIndex = ob->getProperty()->getRefrIndex();
			double n = refrIndex / rIndex;
			Vec3 N = ob->getNormal( P ) * (double)result;
			double cosI = -DOT( N, ray.getDirection() );
			double cosR2 = 1.0 - n * n * (1.0 - cosI * cosI);
			if (cosR2 > 0.0)
			{
				Vec3 R = (n * ray.getDirection()) + (n * cosI - sqrt( cosR2 )) * N;
				Color refrColor( 0, 0, 0 );
				double refrDist;
				rayTrace( Ray( P + R*EPSILON, R), refrColor, depth + 1, rIndex, refrDist, samples*0.5 );
				// 应用 Beer's law
				Color absorbance = ob->getProperty()->getColor() * 0.15 * (-refrDist);
				Color transparency = Color( exp( absorbance.x ), exp( absorbance.y ), exp( absorbance.z ) );
				color += refrColor* transparency;
			}
		}
	rayPath[depth-1] = ob;
	return ob;
}
double Canvas::calculateShade( Light* light, Vec3 P, Vec3& L, double samples )
{
	double shade = 0;
	Object* ob = 0;
	if (light->getType() == Light::POINT)
	{
		// 点光源
		L = light->getPos() - P;
		double tdist = LENGTH( L );
		tdist *= 1 - 4*EPSILON;
		NORMALIZE( L );
		if( !findOccluder( Ray( P + L * EPSILON, L), tdist) )
			return 1.0;
		else
			return 0;
	}
	else if (light->getType() == Light::AREA)
	{
		//面光源
		shade = 0;
		L = light->getPos() - P;
		NORMALIZE( L );
		// 蒙特卡罗软阴影
		Vec3 deltax = light->getCellX(), deltay = light->getCellY();
		double ssale = 1.0 / samples;
		for ( int i = 0; i < samples; i++ )
		{
			Vec3 lp = light->getGrid( i & 15 ) + 1.0*rand()/RAND_MAX * deltax + 1.0*rand()/RAND_MAX * deltay;
			Vec3 dir = lp - P;
			double ldist = LENGTH( dir );
			dir *= 1.0f / ldist;
			ldist *= 1 - 4 * EPSILON;
			if (!findOccluder( Ray( P + dir * EPSILON, dir ), ldist ))			//采样结果计入阴影
				shade += ssale;
		}
	}
	return shade;
}

Object* Canvas::renderRay( Vec3 screenPos, Color& color )
{
	Vec3 D = screenPos - _origin;
	D.normalize();
	Ray ray(_origin, D);
	double dist = 10000.0;
	return rayTrace( ray, color, 1, 1.0, dist, SAMPLES );
	//景深处理
	/*Vec3 C = _origin + D * _f;

	Object* ob;
	Color dfcolor(0, 0, 0);
	for( int i = 0; i < 16 ;i++)
	{
		int x_delta = rand()%200 - 100;
		int y_delta = rand()%200 - 100;
		int z_delta = rand()%200 - 100;
		Vec3 r = Vec3(x_delta/400.0, y_delta/400.0, z_delta/400.0);
		Vec3 o = _origin + r;
		Vec3 d = C - o;
		d.normalize();

		Ray ray( o, d);	
		double dist = 10000.0;
		ob = rayTrace( ray, dfcolor, 1, 1.0, dist, SAMPLES );
		int stop =0 ;
	}
	color = dfcolor * (1.0/16);
	return 0;*/
}

void Canvas::init( int w, int h, Vec3& camera_pos, Vec3& camera_target, double f )
{
	_width = w;
	_height = h;
	_f = f;
	
	_picture = new Vec3*[_width+10];
	for( int i =0 ;i < _width; i ++)
		_picture[i] = new Vec3[_height+10];

	_origin = Vec3( 0, 0, -5 );
	_P1 = Vec3( -4,  3, 0 );
	_P2 = Vec3(  4,  3, 0 );
	_P3 = Vec3(  4, -3, 0 );
	_P4 = Vec3( -4, -3, 0 );
	// 计算相机位置
	Vec3 zaxis = camera_target - camera_pos;
	zaxis.normalize();
	Vec3 up( 0, 1, 0 );
	Vec3 xaxis = up.cross( zaxis );
	xaxis.normalize();
	Vec3 yaxis = xaxis.cross( -zaxis );
	yaxis.normalize();

	Matrix m;
	m.cell[0] = xaxis.x, m.cell[1] = xaxis.y, m.cell[2] = xaxis.z;
	m.cell[4] = yaxis.x, m.cell[5] = yaxis.y, m.cell[6] = yaxis.z;
	m.cell[8] = zaxis.x, m.cell[9] = zaxis.y, m.cell[10] = zaxis.z;
	m.invert();
	m.cell[3] = camera_pos.x, m.cell[7] = camera_pos.y, m.cell[11] = camera_pos.z;
	_origin = m.transform( _origin );
	_P1 = m.transform( _P1 );
	_P2 = m.transform( _P2 );
	_P3 = m.transform( _P3 );
	_P4 = m.transform( _P4 );
	_DX = (_P2 - _P1) * (1.0f / _width);
	_DY = (_P4 - _P1) * (1.0f / _height);

	srand( unsigned int(time(0) ));
}

void Canvas::render()
{
	Object* lastob = 0 ; 

	for ( int y = 0 ; y < _height; y ++)
	{
		std::cout<<"rendering row:  "<<y<<std::endl;
		Vec3 lpos = _P1 + (double)y * _DY ;
		Object* ob;
		for ( int x = 0; x < _width; x++ )
		{
			Color color( 0, 0, 0 );
			ob = renderRay( lpos, color);	
			int red, green, blue;
			//抗锯齿超采样
			if(ob != lastob)
			{
				lastob = ob;
				Color color_( 0, 0, 0 );
				for(int i=-2;i<2;i++)
					for(int j=-2;j<2;j++)
					{
						if( i==0 && j==0)
							continue;
						renderRay( lpos + _DX*0.25f*i + _DY*0.25f*j, color_ );
					}
				renderRay( lpos , color_ );
				red = (int)(color_.x * 256 /16);
				green = (int)(color_.y * 256 /16);
				blue = (int)(color_.z * 256 /16);
			}
			else
			{
				red = (int)(color.x * 256);
				green = (int)(color.y * 256);
				blue = (int)(color.z * 256);
			}

			if (red > 255) red = 255;
			if (green > 255) green = 255;
			if (blue > 255) blue = 255;
			_picture[x][y] = Vec3( red , green , blue );

			lpos += _DX;
		}
	}
}

void Canvas::show(const char* pictureName )
{
	Mat img(_height,_width,CV_8UC3);
	for( int x = 0 ; x < _width ; x ++)
		for( int y = 0 ; y < _height ; y ++)
			img.at<Vec3b>(y,x) = Vec3b ( _picture[x][y].z , _picture[x][y].y , _picture[x][y].x );

	imwrite(pictureName, img);

	namedWindow("Result");
	imshow("Result",img);

	cvWaitKey(0);
	cvDestroyWindow ("Result");
}

}; 