#pragma once
#include <GL/glew.h>

struct VAO_Buffer
{
	GLuint vao;
	GLuint array_buffer;
	GLuint index_buffer;
	GLuint texture;
	GLuint index_size;
};