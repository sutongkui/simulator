#version 330 core
layout (location = 0) in vec4 position;
layout (location = 1) in vec2 uv;
layout (location = 2) in vec3 normal;

out vec3 FragPos;
out vec2 texCoord;
out vec3 Normal;


uniform mat4 modelview;
uniform mat4 projection;

void main()
{
    gl_Position = projection * modelview * position;
    FragPos = vec3(modelview * position);
	texCoord = uv;
    Normal = mat3(transpose(inverse(modelview))) * normal;  
} 