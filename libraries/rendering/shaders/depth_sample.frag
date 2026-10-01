#version 330 core
// One supersampled depth per output pixel: block offset samples/2 (the pixel centre's sample for odd factors,
// 1/(2 samples) pixel off it for even ones), copied, never averaged across an edge.
uniform sampler2D sceneDepth;
uniform int samples;
void main(){gl_FragDepth=texelFetch(sceneDepth,ivec2(gl_FragCoord.xy)*samples+ivec2(samples/2),0).r;}
