#version 330 core
// Label pass vertex shader: world transform plus UVs for part maps, cutouts and alpha tests.
layout(location=0) in vec3 position;
layout(location=2) in vec2 uv;
uniform mat4 model, view, projection;
out vec2 texcoord;

// Same position arithmetic as scene.vert (no `invariant`): label depth equals colour depth on the drivers tested.
void main(){
  vec4 p=model*vec4(position,1);
  texcoord=uv;
  gl_Position=projection*view*p;
}
