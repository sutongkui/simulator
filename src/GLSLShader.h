//A simple class for handling GLSL shader compilation
//Auhtor: Movania Muhammad Mobeen
#pragma once
//#define GLEW_STATIC
#include <GL/glew.h>
#include <map>
#include <string>

using namespace std;


class GLSLShader
{
public:
	GLSLShader(void);
	~GLSLShader(void);	
	void LoadFromString(GLenum whichShader, const string& source);
	void LoadFromFile(GLenum whichShader, const string& filename);
	void CreateAndLinkProgram();
	void Use();
	void UnUse();
	void AddAttribute(const string& attribute);
	void AddUniform(const string& uniform);
    unsigned int GetProgram() const;
	//An indexer that returns the location of the attribute/uniform
	unsigned  int operator[](const string& attribute);
	unsigned  int operator()(const string& uniform);
	//Program deletion
	void DeleteProgram() {glDeleteProgram(_program);_program=-1;}
private:
	enum ShaderType {VERTEX_SHADER, FRAGMENT_SHADER, GEOMETRY_SHADER};
	unsigned  int	_program;
	int _totalShaders;
	unsigned  int _shaders[3];//0->vertexshader, 1->fragmentshader, 2->geometryshader
	map<string,unsigned  int> _attributeList;
	map<string,unsigned  int> _uniformLocationList;
};	
