# Original simulator visual content migration

Read-only source audit at simulator revision `07647eebe706f96ea7b76db3cc9802735a146698`
and `riptide_gui` revision `907c9ded53b0cd6fe7461d8d80c40a4fa9fbc4fc`.
This is a conversion checklist, not a delivered scene. Preserve the same appearance
and behavior in native content; transport bindings stay in optional integrations.

## Files to retain

Existing render resources total 68,435,747 bytes (~65.3 MiB), excluding metadata
and generated geometry. Source roots are `camera_faker/` in the simulator and
`riptide_meshes/meshes/` in riptide_gui.

| Resources | Files | Bytes |
|---|---|---:|
| Talos | models/talos3/Talos3_body.glb and eight rotors/*.glb | 43,595,600 |
| Payloads | models/payloads/launcher.glb and projectile.glb | 2,464,776 |
| Claw | models/claw/{assembly_static,assembly_left,assembly_right,gripper,gripper_right}.glb | 1,363,380 |
| Magnet | models/magnet_lights/{housing,cover,leds,robot_magnet}.glb | 170,068 |
| Course | Listed directories' model.dae and PNG textures | 20,715,443 |
| Calibration board | textures/objects/April Tag.jpg | 126,480 |

Course directories: gate, gate_repair, gate_rescue, slalom, torpedo, bin, bin_vinyl,
table, table_basket_warning, table_basket_helmet, table_pill, table_plug,
table_nut_and_bolt, table_bandage, octagon_buoy, octagon_sos,
octagon_hammer_and_wrench, octagon_compass. Exclude slalom/old.dae, bin_magnet
placeholder, historical Talos3.glb, unrelated meshes and CAD source duplicates.

Retain Talos3_body.json, material_repairs.json, thrusters.yaml and README; payload
and claw provenance.json/README; magnet README/generator references. Record source
revision and hash for every imported resource. Convert status_lights.yaml and
mechanism mounting/slot metadata without ROS topic/message fields.

Task collision resources add 3,069,320 bytes: table.obj,
table_basket_{helmet,warning}.obj, table_{pill,bandage,nut_and_bolt,plug}.obj,
claw_pad.obj and claw_pad_right.obj. Preserve the solid table slab backing.
Static tables/baskets use triangles; moving props/pads use configured convex
representations. Robot/gate/torpedo URDF boxes become native primitives.

## Transform and animation acceptance

- For a COM pose, CAD meshes use worldFromCOM * Translation(-CAD_COM). Old viewer
  body pose was base_link and applied Translation(-CAD_base_link); prove equivalence.
- Body/rotors share CAD coordinates. Animate each rotor around its supplied pivot
  and axis, then apply the common CAD transform. Do not replace precise mesh pivots
  with rounded physics mount values. Input order remains VUS,VUP,HUS,HUP,HLS,HLP,VLS,VLP.
- Preserve realized-force input, signed forward/reverse RPM curves, handedness,
  .01 N visual deadband and .5 s timeout. Topic binding is external to animation.
- Three LED bars: CAD x [-.181,-.143,-.105], y .1573, z .094, size [.012,.055,.003],
  radiance 240. Preserve clear hull alpha, emission, bloom, status and flash behavior.
  The adapter converts LedCommand targets/statuses to named light/color state.
- Claw meshes are already mount-relative. Apply the CAD claw mount once, then
  [0,left,0] / [0,-right,0] finger displacements. Avoid double CAD subtraction.
- Launcher stays CAD-mounted. Loaded projectiles use actuator mount, torpedo
  baseline offsets and measured slot offsets; scale the shared round to
  [length,2*radius,2*radius]. Payload and launcher use one authoritative snapshot.
- Robot magnet uses its mount plus tip offset [0,0,.05].
- Course import resolves mapping parents, marker-local pose/scale and gate offset
  [0,-1.5,0]. Legacy _frame suffix removal happens only during import. Native scene
  references are explicit. Bin vinyl selects Blood/Fire texture from mapped class.
- Pool geometry uses poolToMap independently of vehicle/course transforms.

## Generated geometry and import repairs

Port renderer.cpp geometry: floor/four walls/decks/coping/water (334–367), octagon
8 tubes and 4 sign hangers (543–585), and 4 lattice/corrugated crates (587–645).
Pool dimensions/materials and shader lane markings are part of appearance. Octagon
ring z is surface_z; its mapped origin places hanging signs. Calibration board is
[.002,0,-.345], +X normal, .6096 by .9144 m, with original UVs.

Preserve Assimp Collada IGNORE_UP_DIRECTION=true, node hierarchy transforms,
inverse-transpose normal transforms, opacity and Talos clear material alpha (~.23).
Resolve texture basenames to explicit pack-relative resources during conversion.
Bake the original zero-UV triangle material repairs for table_pill/nut_and_bolt.
Torpedo front and backing near x=-4 mm need identical four cutouts in RGB and depth;
use native generic cutout data or repaired geometry, never renderer name checks.
Declare claw sibling, magnet and material dependencies explicitly.

## Provenance and delivery

riptide_gui/LICENSE is Apache-2.0 but riptide_meshes/package.xml still has TODO
license metadata; asset-specific origins, including competition artwork, need
resolution before public distribution. camera_faker likewise lacks settled license
metadata. Local conversion is authorized. Claw comes from separately supplied
SolidWorks CAD; do not attribute it to Talos3.dae. Magnet resources are documented
procedural approximations without manufacturer CAD/photo textures.

Deliver verified native resources, resolved Talos assembly, resolved course/import
repairs, then fixed-input renderer comparisons with the unchanged original shader
behavior. Each delivered slice must load without the original source trees present.

## Delivered body/rotor slice

The exact body/eight-rotor GLBs and conversion metadata now live in the optional
native visual resource pack. The independent CPU loader retains original geometry,
materials and transparency and compares all 37 submeshes with the original loading
path. See [MESH_ASSETS.md](../MESH_ASSETS.md). This is resource/CPU loading delivery;
mechanisms, LEDs, rotor animation, course assets and original rendering remain open.
For textured course conversion, explicitly preserve the original white texture tint;
the generic loader retains imported tint. Indexed bounds match the original culling
convention, but rendered/depth comparison remains a separate gate.
