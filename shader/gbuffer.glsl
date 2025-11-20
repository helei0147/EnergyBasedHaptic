vertex:
#version 450

layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
// layout(location=2) in vec3 tangent;
// layout(location=3) in vec3 bitangent;
// layout(location=4) in vec2 texcoord;

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

uniform mat4 view;
uniform mat4 model;

out vec4 pos_depth;
out vec3 vs_normal;
out vec2 tex;

void main()
{
	vec4 world_pos = (model * vec4(position, 1.0));
	tex = vec2(1,0);//texcoord;
	vs_normal = mat3(model) * normal;

	pos_depth = (view * world_pos);


	mat3 normalMatrix = transpose(inverse(mat3(view * model)));
	vs_normal = normalMatrix * normal;


	gl_Position = mvp * world_pos;

	pos_depth.w = gl_Position.z;
}
fragment:
#version 450

// NOTE: normal mapping doesn't work well with disocclusion detection
//#define USE_NORMAL_MAPPING

uniform sampler2D sampler0;
uniform sampler2D sampler1;

uniform vec4 baseColor;
uniform vec2 clipPlanes;

in vec4 pos_depth;
in vec3 vs_normal;
in vec2 tex;

layout(location=0)out vec4 my_FragColor0;
layout(location=1)out vec4 my_FragColor1;
layout(location=2)out vec4 my_FragColor2;

const float NEAR = 0.1;
const float FAR = 2500.0f;
float LinearizeDepth(float depth)
{
	float z = depth * 2.0 - 1.0;
	return (2.0 * NEAR * FAR) / (FAR + NEAR - z * (FAR - NEAR));
}

void main()
{

	vec3 n = normalize(vs_normal);

	// view space is right-handed
	//float d = (-vpos.z - clipPlanes.x) / (clipPlanes.y - clipPlanes.x);



	my_FragColor0 = vec4(1,0.0,0.0,1.0);//mix(baseColor, base, matParams.z);
	my_FragColor1 = vec4(n, 1.0);
	my_FragColor2 = pos_depth;
	//my_FragColor2.a = LinearizeDepth(gl_FragCoord.z);
}
