#version 330 core
// Point cloud vertex shader: per-point colour, fixed pixel size (already scaled for supersampling).
layout(location=0) in vec3 position;
layout(location=1) in vec3 color;
uniform mat4 model,view,projection;
uniform float pointSize;
out vec3 pointColor;

void main(){
    pointColor=color;
    gl_Position=projection*view*model*vec4(position,1);
    gl_PointSize=pointSize;
}
