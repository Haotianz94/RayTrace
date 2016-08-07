#ifndef CANVAS_H
#define CANVAS_H

#include "KdTree.h"
#include "object.h"

namespace Raytracer {

class Scene;
class Object;

class Canvas
{
protected:

	Vec3 _origin, _P1, _P2, _P3, _P4, _DX, _DY;
	Scene* _scene;
	Vec3** _picture;
	int _width, _height;
	double _f;
	Object** rayPath;
	int* _Mod;
	KdStack* _stack;

public:
	Canvas() 
	{
		_scene = new Scene();
		rayPath = new Object*[TRACEDEPTH];
		for(int i = 0;i < TRACEDEPTH; i++)
			rayPath[i] = 0;
		_Mod = new int[10];
		_Mod[0] = 0, _Mod[1] = 1, _Mod[2] = 2, _Mod[3] = 0, _Mod[4] = 1;
		_stack = new KdStack[100];
	}
	Canvas( Scene* scene_ ) : _scene(scene_) 
	{
		rayPath = new Object*[TRACEDEPTH];
		for(int i = 0;i < TRACEDEPTH; i++)
			rayPath[i] = 0;
		_Mod = new int[10];
		_Mod[0] = 0, _Mod[1] = 1, _Mod[2] = 2, _Mod[3] = 0, _Mod[4] = 1;
		_stack = new KdStack[100];
	}
	~Canvas() 
	{
		delete _scene;
		delete[] _Mod;
		delete[] _stack;
	}
	void init( int w, int h, Vec3& camera_pos, Vec3& camera_target, double f);
	Scene* getScene() { return _scene; }
	Object* rayTrace( Ray& ray, Color& color, int depth, double refrIndex, double& dist, double samples );
	double calculateShade( Light* light, Vec3 P, Vec3& L, double samples );
	int findNearest( Ray& ray, double& dist, Object*& ob );
	int findOccluder( Ray& ray, double& dist );
	Object* renderRay( Vec3 screenPos, Color& color ); 
	void render();
	void show(const char* pictureName );
};

}; 
#endif

