#version 430

layout(location = 0) in vec4 inPosition;
layout(location = 1) in float inSize;
layout(location = 2) in vec4 inColor;

layout(set = 0, binding = 0) uniform  SceneData{   

	mat4 view;
	mat4 proj;
	mat4 viewproj;
	vec4 ambientColor;
	vec4 sunlightDirection; //w for sun power
	vec4 sunlightColor;
} sceneData;

layout (location = 0) out vec4 outColor;

void main()
{
	// Pass to Fragment shader
	outColor = inColor;

	// Set position of vertex
    gl_Position = sceneData.viewproj * vec4(inPosition.xyz, 1.0);

	// Set particle size
	// Base size of the point sprites
	float spriteSize = 8.0 * 1;

	// Scale particle size depending on camera projection
	vec4 eyePos = sceneData.view * vec4(inPosition.xyz, 1.0);
	vec4 projectedCorner = sceneData.projection * vec4(0.5 * spriteSize, 0.5 * spriteSize, eyePos.z, eyePos.w);
	gl_PointSize = 1700 * projectedCorner.x / projectedCorner.w;	
}