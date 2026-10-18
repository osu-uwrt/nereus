#version 330 core
// Sensor-ready depth: the same opaque-pass sample per output pixel as depth_sample.frag, rows flipped so the
// readback starts at the top, converted from the nonlinear buffer to meters along the optical axis exactly as
// cameras::Processor does on the CPU; cleared background (z >= .999999) is NaN.
uniform sampler2D sceneDepth;
uniform int samples, height;
uniform float nearPlane, farPlane;
out float meters;

void main(){
    ivec2 p=ivec2(gl_FragCoord.xy);
    p.y=height-1-p.y;
    float z=texelFetch(sceneDepth,p*samples+ivec2(samples/2),0).r;
    meters=z>=.999999?uintBitsToFloat(0x7fc00000u)
                     :2.*nearPlane*farPlane/(farPlane+nearPlane-(2.*z-1.)*(farPlane-nearPlane));
}
