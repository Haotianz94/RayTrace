#include "canvas.h"
#include<ctime>
#include<fstream>

using namespace Raytracer;
using namespace std;

int main()
{
	std::clock_t start = clock(); 
	Scene* myscene = new Scene();
	myscene->initScene3();

	Canvas* mycanvas = new Canvas( myscene );
	mycanvas->init( 800, 600,Vec3( -2, 0, -5 ), Vec3( 2, 0, 5 ), 21);//13
	//mycanvas->init( 800, 600,Vec3( -2, 3, -2 ), Vec3( 0, 0, 5 ));
	//mycanvas->init( 800, 600,Vec3( -7, 3, 0 ), Vec3( 5, 0, 0 ), 0);
	//mycanvas->init( 800 , 600, Vec3(0 , 0, 5), Vec3(0, 0, -5), 12.6); 
	mycanvas->render();

	std::clock_t end =clock();
	std::cout<<"rendertime:  "<<1.0*(end-start)/CLOCKS_PER_SEC <<std::endl;
	mycanvas->show("result.jpg");
	return 0;
}