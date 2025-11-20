#version 450 core
// Physically Based Rendering
// Copyright (c) 2017-2018 Michał Siejak

// Physically Based shading model: Vertex program.

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec3 texcoord;


#if VULKAN
layout(set=0, binding=0) uniform TransformUniforms
#else
layout(std140, binding=0) uniform TransformUniforms
#endif // VULKAN
{
	mat4 mvp;
	mat4 sky_mvp;
	vec4 eye;
};

uniform mat4 model;

layout(location=0) out Vertex
{
	vec3 position;
	vec3 normal;
	vec2 texcoord;
} vout;

void main()
{
	
    vec4 model_position = model * vec4(position,1.0);
	vout.position = model_position.xyz;
	vout.texcoord = vec2(texcoord.x, 1.0-texcoord.y);
	vout.normal = normal;

	gl_Position = mvp * model_position;
}
