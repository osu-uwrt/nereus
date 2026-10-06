#version 330 core
// Shadow map pass fragment shader.
in vec2 texcoord;
uniform sampler2D albedo;
uniform int hasTexture;
uniform int holeCount;
uniform vec3 holes[4];

// Depth only; discards UV cutouts and alpha-tested texels so they cast no shadow.
void main(){for(int i=0;i<holeCount;i++)if(distance(texcoord,holes[i].xy)<holes[i].z)discard;if(hasTexture==1 && texture(albedo,texcoord).a<.4)discard;}
