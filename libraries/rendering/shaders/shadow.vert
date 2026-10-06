#version 330 core
// Shadow map depth pass: transforms into the sun's orthographic light space.
layout(location=0)in vec3 position;
layout(location=2)in vec2 uv;
out vec2 texcoord;
uniform mat4 model,lightMatrix;

void main(){texcoord=uv;gl_Position=lightMatrix*model*vec4(position,1);}
