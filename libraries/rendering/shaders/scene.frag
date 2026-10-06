#version 330 core
// Main scene shader: materials, sun lighting with shadows, caustics and underwater attenuation.
// Writes linear HDR color; post.frag tone-maps it.
in vec3 world,norm,poolPosition,poolNormal;
in vec2 texcoord;
in vec4 lightPosition;
out vec4 frag;

uniform sampler2D albedo;
uniform sampler2DShadow shadowMap;
uniform vec4 tint;
uniform vec3 eye,sunDirection;
// material: SurfaceMaterial (0 Asset, 1 Tiles, 2 Deck, 3 Lamp, 4 Liner, 5 Clear, 6 Emissive, 7 Marking).
// clipWater: reflection pass, drop everything below the surface.
uniform int hasTexture,material,useShadow,clipWater;
uniform float time,caustics,directLight,ambientLight;
uniform float ledRadiance;

// Water optics (Appearance::water) and the pool finish.
uniform vec3 waterTint,waterAbsorption;
uniform float waterLevel;
uniform float tileSize;
uniform vec2 waterlineBand;
uniform vec3 waterlineColor;
uniform float waterScattering,waterDistanceScale,waterDistancePower,waterClearDistance;
uniform int outdoor;
uniform int waterEnabled;

// UV cutouts: (u, v, radius) each.
uniform int holeCount;
uniform vec3 holes[4];

// Cheap per-cell pseudo-random value in [0, 1).
float hash(vec2 p){return fract(sin(dot(p,vec2(127.1,311.7)))*43758.5453);}

// Grout lines: ~0 within `thickness` of a grid line every `spacing` meters, 1 on the tile face.
float grid(vec2 p,float spacing,float thickness){
  vec2 footprint=max(fwidth(p),vec2(.00001));
  vec2 d=abs(fract(p/spacing-.5)-.5)*spacing;
  vec2 edge=smoothstep(vec2(thickness)-footprint*.5,vec2(thickness)+footprint*.5,d);
  // Fade subpixel grout to its area average instead of aliasing against the wall.
  vec2 filtered=mix(edge,vec2(1.-2.*thickness/spacing),smoothstep(spacing*.2,spacing*.9,footprint));
  return filtered.x*filtered.y;
}

// Sun visibility from the shadow map (1 = lit); outside the light volume counts as lit.
float visibility(vec3 n,vec3 l){
  if(useShadow==0)return 1.;
  vec3 p=lightPosition.xyz/lightPosition.w*.5+.5;
  if(p.z<0 || p.z>1 || p.x<0 || p.x>1 || p.y<0 || p.y>1)return 1.;
  float shadow=0,bias=max(.00035*(1-dot(n,l)),.00010);
  vec2 texel=1.0/vec2(textureSize(shadowMap,0));
  // Each tap bilinearly filters four comparison results. Tent weights soften
  // the edge without the discrete brightness steps of nearest-depth samples.
  for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++){
    float weight=float((2-abs(x))*(2-abs(y)));
    shadow+=weight*texture(shadowMap,vec3(p.xy+vec2(x,y)*texel,p.z-bias));
  }
  return shadow/16.;
}

float caustic(vec2 p){
  // Interfering ripple fields focus sunlight into moving bands on submerged surfaces.
  p*=3.7;
  p+=vec2(sin(p.y*.71+time*.41),cos(p.x*.62-time*.36))*.58;
  float a=sin(p.x+time*.72)+sin(p.y*1.12-time*.59);
  float b=sin(p.x*.73-p.y*.84+time*.38)+cos(p.y*.67+p.x*.92-time*.47);
  return pow(max(0.,1.-abs(a)*.68),12.)*.65+pow(max(0.,1.-abs(b)*.75),14.)*.45;
}

void main(){
  // Marking decals: UVs are +-1 at the painted edge; fade over one pixel (derivatives before any discard).
  vec2 edge=(1.-abs(texcoord))/max(fwidth(texcoord),vec2(1e-6));
  float coverage=material==7?clamp(edge.x+.5,0.,1.)*clamp(edge.y+.5,0.,1.):1.;
  float worldZ=waterEnabled==1?world.z-waterLevel:1.,eyeZ=waterEnabled==1?eye.z-waterLevel:1.;
  for(int i=0;i<holeCount;i++)if(distance(texcoord,holes[i].xy)<holes[i].z)discard;
  if(clipWater==1 && worldZ<0.015)discard;
  vec4 sampled=hasTexture==1?texture(albedo,texcoord):vec4(1);
  if(sampled.a<.4 || coverage<.004)discard;

  // Base color and per-material surface finish (in pool coordinates for the pool materials).
  vec3 base=sampled.rgb*tint.rgb;
  vec3 n=normalize(norm);if(!gl_FrontFacing)n=-n;
  vec3 pn=normalize(poolNormal);
  if(material==1){
    // Tiles: grout grid on the dominant plane, slight per-tile brightness jitter, waterline band on walls.
    vec2 tile=abs(pn.z)>.5?poolPosition.xy:(abs(pn.x)>.5?poolPosition.yz:poolPosition.xz);
    if(tileSize>0.){
      float g=grid(tile,tileSize,.0025);
      base*=mix(.63,1.,g)*(1.+(hash(floor(tile/tileSize))-.5)*.035*(1.-smoothstep(.02,.10,max(fwidth(tile.x),fwidth(tile.y)))));
    }
    float z=poolPosition.z-waterLevel;
    if(abs(pn.z)<=.5)base=mix(base,waterlineColor,step(waterlineBand.x,z)*step(z,waterlineBand.y));
  } else if(material==2){
    // Deck: coarse 0.6 m slab grid.
    vec2 tile=abs(pn.z)>.5?poolPosition.xy:poolPosition.xz;
    base*=mix(.8,1.,grid(tile,.6,.004));
  }
  if(material==4){
    // Liner: faint 4 mm-period stripes, faded out when a pixel spans several millimeters.
    float p=abs(n.x)>.5?world.y:world.x;
    float fade=1.-smoothstep(.001,.006,fwidth(p));
    base*=1.-.035*fade*(.5+.5*cos(p*1570.796));
  }

  // Hemisphere ambient plus shadowed Blinn-Phong sun; Clear is glossy, Tiles/Marking semi-gloss.
  vec3 l=normalize(sunDirection),v=normalize(eye-world),h=normalize(l+v);
  float nl=max(dot(n,l),0.),nh=max(dot(n,h),0.);
  float vis=visibility(n,l);
  vec3 ambient=mix(vec3(.24,.32,.37),vec3(.48,.55,.56),clamp(n.z*.5+.5,0.,1.));
  float rough=material==5?.06:(material==1 || material==7?.26:.54);
  float spec=pow(nh,mix(85.,14.,rough))*(material==5?1.4:(material==1 || material==7?.17:.055));
  vec3 lighting=base*(ambient*ambientLight+(outdoor==1?vec3(1.15,1.08,.95):vec3(.92,1.01,1.08))*nl*vis*directLight)+spec*vis*directLight;

  // Submerged surfaces: cooler light, animated caustics, and absorption on the way down from the surface.
  if(worldZ<0){
    lighting*=vec3(.84,.97,1.04);
    float c=caustic(world.xy+world.z*n.xy*.5);
    lighting+=base*c*caustics*directLight*exp(worldZ*.14)*(.3+.7*max(n.z,0.))*(.35+.65*vis);
    // Approximate incoming surface light with a vertical path through the water.
    // Use physical depth in meters; viewing-distance controls apply below.
    lighting*=exp(-waterAbsorption*(-worldZ));
  }
  // Emissive materials generate their own light and only lose it on the way to the camera.
  if(material==3)lighting=base*1.65;
  if(material==6)lighting=base*ledRadiance;

  // `wet`: length of the eye-to-fragment ray that lies under water (all, part, or none of it).
  float d=length(eye-world),wet=0;
  if(eyeZ<0 && worldZ<0)wet=d;
  else if(eyeZ>=0 && worldZ<0)wet=d*(-worldZ)/max(.001,eye.z-world.z);
  else if(eyeZ<0 && worldZ>=0)wet=d*(-eyeZ)/max(.001,world.z-eye.z);

  // Beer-Lambert attenuation along that path, filled in by the water's scattered color.
  float opticalDistance=pow(max(0.,wet-waterClearDistance)*waterDistanceScale,waterDistancePower);
  float transmission=exp(-opticalDistance*waterScattering);
  vec3 attenuation=exp(-opticalDistance*waterAbsorption)*transmission;
  vec3 scatter=waterTint*(.2+.5*ambientLight+.3*directLight);
  lighting=lighting*attenuation+scatter*(1.-transmission);

  // Clear covers get a Fresnel-like rim; Marking decals use their anti-aliased coverage.
  float alpha=material==5 ? clamp(tint.a+pow(1.-abs(dot(n,v)),5.)*.55+spec*.15,0.,.85) : coverage;
  frag=vec4(lighting,alpha);
}
