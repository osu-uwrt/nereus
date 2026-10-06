#version 330 core
// Post-processing pass vertex shader (also shared by depth_sample.frag).
// Full-screen triangle generated from gl_VertexID (no vertex buffer): vertices (0,0), (2,0), (0,2)
// in uv, so uv spans [0, 1] over the viewport.
out vec2 uv;
void main(){vec2 p=vec2((gl_VertexID<<1)&2,gl_VertexID&2);uv=p;gl_Position=vec4(p*2.-1.,0,1);}
