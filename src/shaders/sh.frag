
#version 330 core
out vec4 color;

in vec2 texCoord;  
in vec3 Normal;  
  
uniform sampler2D ourTexture;
uniform vec3 lightPos; 
uniform vec3 viewPos;
uniform vec3 lightColor;
uniform vec3 objectColor;

const float scaleColor = 1.0;
//sphere harmonic function
const float C1 = 0.429043;
const float C2 = 0.511664;
const float C3 = 0.743125;
const float C4 = 0.886227;
const float C5 = 0.247708;

//old town square coeffients
const vec3 L00 = vec3(0.87, 0.87, 0.86);
const vec3 L1m1 = vec3(0.17, 0.24, 0.31);
const vec3 L10 = vec3(0.03, 0.03, 0.03);
const vec3 L11 = vec3(-0.004, -0.03, -0.05);
const vec3 L2m2 = vec3(-0.12, -0.12, -0.11);
const vec3 L2m1 = vec3(0.003, 0.003, 0.007);
const vec3 L20 = vec3(-0.03, -0.02, -0.02);
const vec3 L21 = vec3(-0.08, -0.08, -0.09);
const vec3 L22 = vec3(-0.16, -0.19, -0.22);


void main()
{
    // Ambient
    vec3 ambient = vec3(0.0);
  	
    // Diffuse 
    vec3 tnorm = normalize(Normal);
    vec3 diffuse =  C1*L22*(tnorm.x*tnorm.x-tnorm.y*tnorm.y) + 
					C3*L20*tnorm.z*tnorm.z +
					C4*L00 -
					C5*L20 +
					2.0*C1*L2m2 * tnorm.x*tnorm.y +
					2.0*C1*L21 * tnorm.x*tnorm.y +
					2.0*C1*L2m1*tnorm.y*tnorm.z +
					2.0*C2*L11*tnorm.x +
					2.0*C2*L1m1 *tnorm.y +
					2.0*C2*L10 * tnorm.z;
	diffuse *= scaleColor;
	vec3 specular = vec3(0.0); 
        
    vec3 result = (ambient + diffuse + specular) * objectColor;
	color = texture(ourTexture, texCoord) * vec4(result,1.0f);
   // color = vec4(result, 1.0f);
} 