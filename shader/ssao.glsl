vertex:
#version 450 core

layout(location=0) out vec2 screenPosition;

layout(std140, binding = 0) uniform TransformUniforms
{
	mat4 mvp;
	mat4 sky_mvp;
	vec4 eyepos;
};

void main()
{
	if(gl_VertexID == 0) {
		screenPosition = vec2(1.0, 2.0);
		gl_Position = vec4(1.0, 3.0, 0.0, 1.0);
	}
	else if(gl_VertexID == 1) {
		screenPosition = vec2(-1.0, 0.0);
		gl_Position = vec4(-3.0, -1.0, 0.0, 1.0);
	}
	else /* if(gl_VertexID == 2) */ {
		screenPosition = vec2(1.0, 0.0);
		gl_Position = vec4(1.0, -1.0, 0.0, 1.0);
	}
}

fragment:
#version 450

/*
  (C) 2019 David Lettier
  lettier.com
*/


#define NUM_SAMPLES 64
#define NUM_NOISE   16

uniform mat4 projection;

layout(location = 0) in  vec2 screenPosition;

uniform vec3 samples[NUM_SAMPLES];
uniform vec3 noise[NUM_NOISE];

layout(binding = 0) uniform sampler2D gNormal;
layout(binding = 1) uniform sampler2D gPosition;

uniform vec2 enabled;
uniform vec3 cam_pos;
uniform vec4 paras;

out vec3 FragColor;

void main() {

    int kernelSize = NUM_SAMPLES;
    float radius = paras.x;
    float bias = paras.y;

    vec2 texSize = textureSize(gPosition, 0).xy;
    vec2 texCoord = gl_FragCoord.xy / texSize;
    // 
    // get input for SSAO algorithm
    vec3 fragPos = texture(gPosition, texCoord).xyz;
    vec3 normal = normalize(texture(gNormal, texCoord).rgb);

    //radius = 10;// clamp(fragPos.z / 10, 1.0, 100.0)* radius;

    int  noiseS = int(sqrt(NUM_NOISE));
    int  noiseX = int(gl_FragCoord.x - 0.5) % noiseS;
    int  noiseY = int(gl_FragCoord.y - 0.5) % noiseS;
    vec3 randomVec = noise[noiseX + (noiseY * noiseS)];

    //abs(texture(texNoise, TexCoords*noiseScale).xyz);// 

    // create TBN change-of-basis matrix: from tangent-space to view-space
    vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 TBN = mat3(tangent, bitangent, normal);
    // iterate over the sample kernel and calculate occlusion factor
    float occlusion = 0.0;
    for (int i = 0; i < kernelSize; ++i)
    {
         // get sample position
        vec3 samplePos = TBN * samples[i]; // from tangent to view-space
        samplePos = fragPos + samplePos * radius;

        // project sample position (to sample texture) (to get position on screen/texture)
        vec4 offset = vec4(samplePos, 1.0);
        offset = projection * offset; // from view to clip-space
        offset.xyz /= offset.w; // perspective divide
        offset.xyz = offset.xyz * 0.5 + 0.5; // transform to range 0.0 - 1.0

        // get sample depth
        float sampleDepth = texture(gPosition, offset.xy).z; // get depth value of kernel sample
  

        ///
         // get sample position
        vec3 samplePos1 = TBN * -samples[i]; // from tangent to view-space
        samplePos1 = fragPos + samplePos1 * radius;

        // project sample position (to sample texture) (to get position on screen/texture)
        vec4 offset1 = vec4(samplePos1, 1.0);
        offset1 = projection * offset1; // from view to clip-space
        offset1.xyz /= offset1.w; // perspective divide
        offset1.xyz = offset1.xyz * 0.5 + 0.5; // transform to range 0.0 - 1.0

        // get sample depth
        float sampleDepth1 = texture(gPosition, offset1.xy).z; // get depth value of kernel sample

        // range check & accumulate  0 
        float occlusionLocal = (sampleDepth > samplePos.z - bias ? 1.0 : 0.0) * smoothstep(0.0, 1.0, radius / abs(fragPos.z - sampleDepth));
        float rangeCheck = abs(sampleDepth1 - sampleDepth);
        occlusion += occlusionLocal*(rangeCheck > paras.z ? 0 : 1);

    }
    occlusion = fragPos.z>0?1.0:1.0 - (occlusion / kernelSize);//pow(1.0 - (occlusion / kernelSize),2.0);//
    FragColor = vec3(occlusion);//normal;//randomVec;//

}