#version 430

layout (binding = 1) uniform sampler2D particleTexture;
layout (location = 0) in vec4 inColor;

layout (location = 0) out vec4 outFragColor;

void main()
{
    vec4 colour = texture(particleTeture, gl);
    outFragColor.rgb = texture(particleTexture, gl_PointCoord).rgb * inColor.rgb;
}
