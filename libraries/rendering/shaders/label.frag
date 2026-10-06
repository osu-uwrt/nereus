#version 330 core
// Label pass: writes id << 8 | part per fragment, with the same discards as the color pass.
in vec2 texcoord;
layout(location=0) out uint label;
uniform sampler2D albedo;
uniform usampler2D partMap;
uniform int hasTexture, hasPartMap, holeCount;
uniform vec3 holes[4];
uniform uint id, part;

void main(){
  float alpha=texture(albedo,texcoord).a; // sampled in uniform control flow (implicit derivatives)
  uint value=part;
  if(hasPartMap==1){
    // Nearest texel with repeat wrap; rows uploaded like diffuse textures (uv v=0 is the image bottom).
    ivec2 size=textureSize(partMap,0);
    value=texelFetch(partMap,clamp(ivec2(floor(fract(texcoord)*vec2(size))),ivec2(0),size-1),0).r;
  }
  // UV cutouts discard as in scene.frag, except where a part map labels the hole of a labeled instance (a
  // ring's value fills it). Id 0 only occludes: no part, and its holes stay open.
  bool hole=false;
  for(int i=0;i<holeCount;i++)hole=hole || distance(texcoord,holes[i].xy)<holes[i].z;
  if(hole && (hasPartMap==0 || value==0u || id==0u))discard;
  if(!hole && hasTexture==1 && alpha<.4)discard;
  // Pixel value: 24-bit instance id in the high bits, 8-bit part in the low byte.
  label=id==0u?0u:(id<<8u)|value;
}
