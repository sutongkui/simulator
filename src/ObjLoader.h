#pragma once
#include <vector>
#include <string>

#include <GL/glew.h>
#include <glm/glm.hpp>

#include "vao_buffer.h"

using namespace std;

struct Face
{
	unsigned int vertex_index[3];
	unsigned int tex_index[3];
	unsigned int normal_index[3];
};

class ObjLoader
{
public:
	ObjLoader(const string file);

public:
	GLuint g_textureID;

	string obj_file;                
	string mtl_file;
	string texture_file;

	vector<glm::vec4> vertices; 
	vector<glm::vec3> normals;
	vector<glm::vec2> tex;
	vector<Face> faces;

	vector<pair<string,unsigned int>> vertex_object;  //for vertices region division 
	vector<pair<string,unsigned int>> face_group;
};


