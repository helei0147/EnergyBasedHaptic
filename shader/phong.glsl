vertex:
#version 450 core
layout(std140, binding = 0) uniform TransformUniforms
{
	mat4 mvp;
	mat4 sky_mvp;
	vec4 eyepos;
};

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location = 2) in vec3 texcoord;

uniform mat4 model;

out vec3 vs_pos;
out vec3 vs_normal;

void main()
{
    vec4 model_position = model * vec4(position,1.0);
	gl_Position   = mvp * model_position;
	vs_pos = model_position.xyz;
    vs_normal = normal;
}

fragment:
#version 450 core

out vec4 FragColor;

in vec3 vs_pos;
in vec3 vs_normal;

uniform vec3 eye;
uniform vec4 diffuse;

void main()
{
    vec3 lightPos = vec3(3,3,3); 
    vec3 lightColor = vec3(1, 1, 0); 

    // ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

    // diffuse
    vec3 norm = normalize(vs_normal);
    vec3 lightDir = normalize(lightPos - vs_pos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuseColor = diff * lightColor;

    // specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(eye - vs_pos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;

    // result
    vec3 result = (ambient + diffuseColor + specular) * diffuse.xyz;
    FragColor = vec4(result, diffuse.a);
}