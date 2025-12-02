#version 450
out vec3 FragColor;

layout(location = 0) in  vec2 screenPosition;
layout(binding = 0) uniform sampler2D ssaoInput;

float _KernelSize = 0;
float space_sigma = 2.5;
float range_sigma = 0.9;
uniform vec3 blurpara;

#define PI 3.1415926

vec3 BilateralBlur(vec2 uv, float space_sigma, float range_sigma){
    float weight_sum = 0;
    vec3 color_sum = vec3(0);

    vec3 color_origin = texture(ssaoInput, uv).rgb;
    vec3 color = vec3(0);

    vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    for(float i = -_KernelSize; i < _KernelSize; i++){
        for(float j = -_KernelSize; j < _KernelSize; j++){
            //空域高斯
            vec2 varible = uv + vec2(float(i), float(j)) * texelSize;
            float space_factor = i * i + j * j;
            space_factor = (-space_factor) / (2 * space_sigma * space_sigma);
            float space_weight = 1/(space_sigma * space_sigma * 2 * PI) * exp(space_factor);

            //值域高斯
            vec3 color_neighbor = texture(ssaoInput, varible).rgb;
            vec3 color_distance = (color_neighbor - color_origin);
            float value_factor = color_distance.r * color_distance.r ;
            value_factor = (-value_factor) / (2 * range_sigma * range_sigma);
            float value_weight = (1 / (2 * PI * range_sigma)) * exp(value_factor);

            weight_sum += space_weight * value_weight;
            color_sum += color_neighbor * space_weight * value_weight;
        }
    }

    if(weight_sum > 0){
        color = color_sum / weight_sum;
    }

    return color;
}


void main() 
{
    //FragColor = texture(ssaoInput, TexCoords).r;
    //return;

    //#if false
    //vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    //float result = 0.0;
    //for (int x = -2; x < 2; ++x) 
    //{
    //    for (int y = -2; y < 2; ++y) 
    //    {
    //        vec2 offset = vec2(float(x), float(y)) * texelSize;
    //        result += texture(ssaoInput, screenPosition + offset).r;
    //    }
    //}
    //FragColor = result / (4.0 * 4.0);
    //#endif
    _KernelSize = blurpara.x;
    space_sigma = blurpara.y;
    range_sigma = blurpara.z;

    vec3 color = BilateralBlur(screenPosition, blurpara.y,blurpara.z);
    FragColor = color;
}  