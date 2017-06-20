
#version 330 core
out vec4 color;

in vec3 FragPos;
in vec2 texCoord;  
in vec3 Normal;  
  
uniform sampler2D ourTexture;
uniform vec3 lightPos; 
uniform vec3 viewPos;
uniform vec3 lightColor;
uniform vec3 objectColor;

void main()
{
    // Ambient
    float ambientStrength = 0.5f;
    vec3 ambient = ambientStrength * lightColor;
  	
    // Diffuse 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    // Specular
    //float specularStrength = 0.5f;
    //vec3 viewDir = normalize(viewPos);
   // vec3 reflectDir = reflect(-lightDir, norm);  
   // float spec = pow(max(dot(viewDir, reflectDir), 0.0), 12);
    //vec3 specular = specularStrength * spec * lightColor; 
	vec3 specular = vec3(0.0); 
        
    vec3 result = (ambient + diffuse + specular) * objectColor;
	color = texture(ourTexture, texCoord) * vec4(result,1.0f);
   // color = vec4(result, 1.0f);
} 