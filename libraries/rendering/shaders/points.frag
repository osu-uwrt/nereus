#version 330 core
// Point cloud: unlit round points.
in vec3 pointColor;
out vec4 frag;

void main(){
    vec2 offset=gl_PointCoord*2.-1.;
    if(dot(offset,offset)>1.) discard; // round points
    frag=vec4(pointColor,1);
}
