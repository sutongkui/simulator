// simulation_bvh.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include <iostream>
//#include <cmdline.h>

#include "scene.h"
#include "spring.h"
#include "simulator.h"
#include "ObjLoader.h"
#include "./bvh/bvh.h"
#include "Mesh.h"

using namespace std;

int main(int argc, char** argv)
{
	/*cmdline::parser a;
	a.add<string>("cloth", 'c', "cloth file", true, "");
	a.parse_check(argc, argv);
	cout << a.get<string>("cloth") << endl;*/

	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 

	Mesh cloth("../../../resources/cloth/tshirt2/tshirt2.obj", SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.rotation(-4, Z);
	cloth.scale(3.33);
	cloth.translate(0, 1.98, 0.02);  // maybe we can use model matrix to handle this

	Mesh body("../../../resources/pose/Female.obj");
	body.scale(3.226);
	body.translate(0, 1.8, 0);

	main_scene->add_cloth(cloth);
	main_scene->add_body(body);

	main_scene->init_simulation();
	main_scene->render();

	getchar();
	return 0;
}

