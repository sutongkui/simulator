#pragma once

#define GLEW_STATIC
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>

#include "vao_buffer.h"
#include "GLSLShader.h"
#include "Mesh.h"
#include "simulator.h"
#include "./bvh/bvh.h"

#pragma comment(lib, "glew32s.lib")

// Singleton
class Scene
{
public:
	static Scene* getInstance(int argc, char** argv);
	~Scene(); 
	
	void add_cloth(Mesh& object);   // mesh(cloth) to be simulated
	void add_body(Mesh& object);    // mesh(body) to be collided
	void init_simulation();               // construct simualtion
	void render();

private:
	Scene(int argc, char** argv);  //initial
	void add(Mesh& object);   //add objects,bind VAOs 
	void RenderBuffer(VAO_Buffer vao_buffer);
	void loadShader();
	void save_obj(string file, vector<glm::vec3> vertices);

private:
	vector<VAO_Buffer> obj_vaos;
	static Scene* pscene;       //pscene points to the Scene(singleton)
	GLSLShader renderShader;
	enum attributes { position, texture, normal };

	static Mesh* cloth;
	static Mesh* body;
	Simulator* simulation;

private:
	static void screenshot();
	static void DrawGrid();
    // OPENGL场景的各种函数
	static void RenderGPU_CUDA();
	static void onRender();
	static void OnReshape(int nw, int nh);
	static void OnIdle();
	static void OnMouseMove(int x, int y);
	static void OnMouseDown(int button, int s, int x, int y);
	static void OnKey(unsigned char key, int, int);
	static void OnShutdown();
	inline void check_GL_error();

private:
	static int oldX, oldY;    // OPENGL场景的各种参数declaration
	static float rX, rY;
	static int state;
	static float dist, dy;
	static GLint viewport[4];
	static GLfloat modelview[16];
	static GLfloat projection[16];
	static glm::vec3 Up, Right, viewDir;
	static int selected_index;
	static const int width = 1024, height = 1024;
	static bool start_sim;
};



