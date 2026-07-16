#version 440

layout(location = 0) in vec2 qt_TexCoord0;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform buf {
    mat4 qt_Matrix;
    float qt_Opacity;
    float brightness;   // -1 .. 1
    float contrast;     // -1 .. 1
    float saturation;   // -1 .. 1
    float sharpen;      //  0 .. 1
    float grayscale;    //  0 or 1
    vec2 texelStep;
};

layout(binding = 1) uniform sampler2D source;

void main()
{
    vec4 tex = texture(source, qt_TexCoord0);
    vec3 c = tex.rgb;

    // unsharp mask (4-tap)
    if (sharpen > 0.001) {
        vec3 blur = ( texture(source, qt_TexCoord0 + vec2( texelStep.x, 0.0)).rgb
                    + texture(source, qt_TexCoord0 + vec2(-texelStep.x, 0.0)).rgb
                    + texture(source, qt_TexCoord0 + vec2(0.0,  texelStep.y)).rgb
                    + texture(source, qt_TexCoord0 + vec2(0.0, -texelStep.y)).rgb ) * 0.25;
        c += (c - blur) * sharpen * 2.0;
    }

    // contrast around mid-gray, then brightness
    c = (c - 0.5) * (1.0 + contrast) + 0.5;
    c += brightness * 0.6;

    // saturation / grayscale
    float luma = dot(clamp(c, 0.0, 1.0), vec3(0.299, 0.587, 0.114));
    c = mix(vec3(luma), c, 1.0 + saturation);
    c = mix(c, vec3(luma), clamp(grayscale, 0.0, 1.0));

    fragColor = vec4(clamp(c, 0.0, 1.0), 1.0) * tex.a * qt_Opacity;
}
