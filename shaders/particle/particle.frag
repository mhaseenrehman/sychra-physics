#version 430

in vec2 uv;
out vec4 final_color;
uniform sampler2D tex;

void main()
{
    final_color = texture(tex, uv).rgba;
}