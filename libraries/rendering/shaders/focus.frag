#version 330 core
// Orbit focus marker: translucent yellow disc with fixed view-space diffuse and highlight lights.
in vec3 viewNormal;
out vec4 frag;

void main(){
    vec3 n=normalize(viewNormal);
    float diffuse=max(0.,dot(n,normalize(vec3(-.4,.6,1.))));
    float highlight=pow(max(0.,dot(n,normalize(vec3(-.3,.4,1.)))),24.);
    frag=vec4(vec3(1.,.8,.06)*(.3+.7*diffuse)+vec3(.35)*highlight,.75);
}
