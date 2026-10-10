#version 330 core
// Bloom: bright-pass downsample of the HDR scene to quarter size, halving downsamples for the coarser levels,
// and a separable Gaussian blur of each level.
in vec2 uv;
out vec4 frag;
uniform sampler2D source;
uniform vec2 stepSize;
// 0: bright extract (level 0), 1: blur along stepSize, 2: downsample the level before.
uniform int mode;

void main(){
  vec3 c=vec3(0);
  if(mode==0){
    // Average a 4x4 block of the HDR excess above 1.2 (the bloom threshold).
    // Preserve small bright emitters when reducing the image to quarter size.
    for(int y=0;y<4;++y)for(int x=0;x<4;++x)
      c+=max(texture(source,uv+(vec2(x,y)-vec2(1.5))*stepSize).rgb-vec3(1.2),vec3(0));
    c/=16.;
  }else if(mode==2){
    // Half size: four bilinear taps a source texel diagonally off center, a 4x4 box of the level before.
    for(int y=-1;y<=1;y+=2)for(int x=-1;x<=1;x+=2)
      c+=texture(source,uv+vec2(x,y)*stepSize).rgb;
    c/=4.;
  }else{
    // Separable 9-tap Gaussian blur along stepSize (horizontal, then vertical).
    const float weights[5]=float[5](.227027,.1945946,.1216216,.054054,.016216);
    c=texture(source,uv).rgb*weights[0];
    for(int i=1;i<5;++i)
      c+=(texture(source,uv+stepSize*float(i)).rgb+texture(source,uv-stepSize*float(i)).rgb)*weights[i];
  }
  frag=vec4(c,1);
}
