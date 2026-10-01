"""Build the RPAC dive well's fixture meshes (OBJ, metres, Z up, water surface at z = 0).

Run from anywhere: python3 make_meshes.py. Writes next to this file:
- tower_stairs.obj: the staircase in the well between the tower pillars (a landing, treads and three
  handrails). Origin at the well's centre on the wall face; the pool is +y, the well runs back to y = -WELL_DEPTH, and
  -x is the deep-end side.
- ladder.obj: a wall ladder (two grab rails over the edge, toe holds on the wall). Origin on the wall face at
  the ladder's centre; the pool is +y, the deck -y.
- raised_grates.obj: a row of three raised square drain grates along x. Origin on the floor at the row's centre.
- wall_vent.obj: a slatted square vent on a wall. Origin at its centre on the wall face; it faces +y.
- floor_vent.obj: a flush slotted square vent. Origin at its centre on the floor; it faces +z.
"""
import math
from pathlib import Path

HERE = Path(__file__).parent
DECK = 0.3048  # deck above the water
WELL_DEPTH = 2.4  # tower stair well, back from the wall face (pool.yaml's recess depth_m must match)

COLORS = {
    "steel": (0.75, 0.77, 0.78),
    "tread": (0.78, 0.84, 0.84),
    "nosing": (0.1, 0.15, 0.18),
    "hold": (0.05, 0.1, 0.13),
    "grate_top": (0.72, 0.85, 0.86),
    "grate_side": (0.3, 0.45, 0.5),
    "vent": (0.05, 0.12, 0.16),
    "slat": (0.3, 0.5, 0.55),
    "floor_vent": (0.75, 0.86, 0.87),
    "floor_slot": (0.12, 0.2, 0.24),
}


class Mesh:
    def __init__(self) -> None:
        self.vertices: list[tuple[float, float, float]] = []
        self.normals: list[tuple[float, float, float]] = []
        self.faces: dict[str, list[list[tuple[int, int]]]] = {}

    def vertex(self, p, n) -> tuple[int, int]:
        self.vertices.append(tuple(p))
        self.normals.append(tuple(n))
        return len(self.vertices), len(self.normals)

    def polygon(self, material: str, points, normal=None) -> None:
        """Counter-clockwise seen from outside; flat normal unless given per point."""
        if normal is None:
            a, b, c = points[0], points[1], points[2]
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            length = math.sqrt(sum(x * x for x in n))
            normal = [[x / length for x in n]] * len(points)
        self.faces.setdefault(material, []).append([self.vertex(p, n) for p, n in zip(points, normal)])

    def box(self, material: str, lo, hi) -> None:
        (x0, y0, z0), (x1, y1, z1) = lo, hi
        c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1),
             (x0, y1, z1)]
        for quad in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            self.polygon(material, [c[i] for i in quad])

    def frustum(self, center, base: float, top: float, height: float) -> None:
        """Square base and smaller square top; the sides slope in equally."""
        cx, cy, cz = center
        b, t = base / 2, top / 2
        lo = [(cx - b, cy - b, cz), (cx + b, cy - b, cz), (cx + b, cy + b, cz), (cx - b, cy + b, cz)]
        hi = [(cx - t, cy - t, cz + height), (cx + t, cy - t, cz + height), (cx + t, cy + t, cz + height),
              (cx - t, cy + t, cz + height)]
        self.polygon("grate_top", hi)
        for k in range(4):
            self.polygon("grate_side", [lo[k], lo[(k + 1) % 4], hi[(k + 1) % 4], hi[k]])

    def tube(self, material: str, path, radius: float = 0.022, bend: float = 0.12, sides: int = 10) -> None:
        """A round rail along a polyline, its corners rounded with `bend` radius."""
        points = [path[0]]
        for a, b, c in zip(path, path[1:], path[2:]):
            u = _unit(_sub(a, b))
            v = _unit(_sub(c, b))
            angle = math.acos(max(-1.0, min(1.0, _dot(u, v))))
            cut = min(bend / math.tan(angle / 2), _norm(_sub(a, b)) / 2, _norm(_sub(c, b)) / 2)
            p0, p1 = _add(b, _scale(u, cut)), _add(b, _scale(v, cut))
            for i in range(9):  # quadratic Bezier through the corner
                s = i / 8
                points.append(tuple((1 - s) ** 2 * p0[k] + 2 * (1 - s) * s * b[k] + s * s * p1[k] for k in range(3)))
        points.append(path[-1])
        rings = []
        normal = None
        for i, p in enumerate(points):
            t = _unit(_sub(points[min(i + 1, len(points) - 1)], points[max(i - 1, 0)]))
            if normal is None:
                helper = (0, 0, 1) if abs(t[2]) < 0.9 else (1, 0, 0)
                normal = _unit(_cross(helper, t))
            normal = _unit(_sub(normal, _scale(t, _dot(normal, t))))  # parallel transport
            binormal = _cross(t, normal)
            ring = []
            for k in range(sides):
                a = 2 * math.pi * k / sides
                d = _add(_scale(normal, math.cos(a)), _scale(binormal, math.sin(a)))
                ring.append((_add(p, _scale(d, radius)), d))
            rings.append(ring)
        for r0, r1 in zip(rings, rings[1:]):
            for k in range(sides):
                q = [r0[k], r0[(k + 1) % sides], r1[(k + 1) % sides], r1[k]]
                self.polygon(material, [x[0] for x in q], [x[1] for x in q])
        for ring, sign in ((rings[0], -1), (rings[-1], 1)):  # end caps
            pts = [x[0] for x in ring]
            self.polygon(material, pts if sign > 0 else pts[::-1])

    def write(self, name: str) -> None:
        obj = HERE / f"{name}.obj"
        lines = [f"# {name}: generated by make_meshes.py", f"mtllib {name}.mtl"]
        lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in self.vertices]
        lines += [f"vn {x:.5f} {y:.5f} {z:.5f}" for x, y, z in self.normals]
        for material, faces in self.faces.items():
            lines.append(f"usemtl {material}")
            lines += ["f " + " ".join(f"{v}//{n}" for v, n in face) for face in faces]
        obj.write_text("\n".join(lines) + "\n")
        mtl = [f"# {name}: generated by make_meshes.py"]
        for material in self.faces:
            r, g, b = COLORS[material]
            mtl += [f"newmtl {material}", f"Kd {r} {g} {b}", "Ka 0 0 0", "Ks 0 0 0", "d 1", ""]
        (HERE / f"{name}.mtl").write_text("\n".join(mtl))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _scale(a, s):
    return tuple(x * s for x in a)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    return _scale(a, 1 / _norm(a))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def tower_stairs() -> Mesh:
    """A landing at deck level against the back of the well, then pool-stair treads stepping down toward the pool
    (0.3 m runs, 0.19 m risers, about 32 degrees, matching the handrail slope in the platform photo) to the well
    floor at z = -1.22, each with a dark nosing. Three identical handrails, the outer two close to
    the well's sides and the middle one off centre toward the shallow end (platform photo), run down the flight
    0.9 m above the nosings from a hooped top on the landing, with a post to a tread partway down, and drop to the
    well floor at the front."""
    m = Mesh()
    half, back, floor, run, risers = 1.05, WELL_DEPTH, -1.22, 0.3, 8
    rise = (DECK - floor) / risers
    landing = run * (risers - 1)  # the landing's front edge, back from the wall; the treads fill the rest to the
    # wall face, where the last riser drops to the well floor
    m.box("tread", (-half, -back, floor), (half, -landing, DECK))
    for k in range(1, risers):
        top, front = DECK - rise * k, -landing + run * k
        m.box("tread", (-half, -landing + run * (k - 1), floor), (half, front, top))
        m.box("nosing", (-half, front - 0.05, top), (half, front, top + 0.002))

    def nosing_z(y: float) -> float:
        return DECK - rise / run * (y + landing)

    for x in (-0.92, 0.25, 0.92):
        m.tube("steel", [(x, -back + 0.08, DECK), (x, -back + 0.08, DECK + 0.95),
                         (x, -landing, nosing_z(-landing) + 0.9), (x, -0.15, nosing_z(-0.15) + 0.9),
                         (x, -0.15, floor)])
        mid = -landing + run * 3
        m.tube("steel", [(x, mid, nosing_z(mid) + 0.9), (x, mid, DECK - rise * 3)])
    return m


def ladder() -> Mesh:
    """Two grab rails rising from the deck behind the edge, over the coping and down into the water close to
    the wall, each with a brace to the deck; four dark toe holds in the wall between them."""
    m = Mesh()
    for x in (-0.3, 0.3):
        m.tube("steel", [(x, -0.55, DECK), (x, -0.55, 0.95), (x, 0.12, 0.95), (x, 0.12, -0.5)], bend=0.15)
        m.tube("steel", [(x, -0.12, DECK), (x, -0.12, 0.95)])
    for z in (-0.28, -0.55, -0.82, -1.09):
        m.polygon("hold", [(-0.15, 0.002, z - 0.075), (0.15, 0.002, z - 0.075), (0.15, 0.002, z + 0.075),
                           (-0.15, 0.002, z + 0.075)][::-1])
    return m


def raised_grates() -> Mesh:
    """Three square grates in a row: 0.75 m at the base, 0.55 m flat tops, 5 cm proud."""
    m = Mesh()
    for x in (-0.75, 0.0, 0.75):
        m.frustum((x, 0.0, 0.0), 0.75, 0.55, 0.05)
    return m


def wall_vent() -> Mesh:
    """0.28 m square, dark, with three light horizontal slats, 3 mm off the wall."""
    m = Mesh()
    m.polygon("vent", [(0.14, 0.003, -0.14), (-0.14, 0.003, -0.14), (-0.14, 0.003, 0.14), (0.14, 0.003, 0.14)])
    for z in (-0.07, 0.0, 0.07):
        m.box("slat", (-0.13, 0.0, z - 0.006), (0.13, 0.006, z + 0.006))
    return m


def floor_vent() -> Mesh:
    """0.40 m square flush vent: pale like the lines, with three dark slots, 3 mm off the floor."""
    m = Mesh()
    m.polygon("floor_vent", [(-0.2, -0.2, 0.003), (0.2, -0.2, 0.003), (0.2, 0.2, 0.003), (-0.2, 0.2, 0.003)])
    for y in (-0.1, 0.0, 0.1):
        m.polygon("floor_slot", [(-0.16, y - 0.02, 0.004), (0.16, y - 0.02, 0.004), (0.16, y + 0.02, 0.004),
                                 (-0.16, y + 0.02, 0.004)])
    return m


# Every mesh the pack uses, by asset name. The test in tests/python/test_rpac_assets.py regenerates these.
MESHES = {
    "tower_stairs": tower_stairs,
    "ladder": ladder,
    "raised_grates": raised_grates,
    "wall_vent": wall_vent,
    "floor_vent": floor_vent,
}

if __name__ == "__main__":
    for name, build in MESHES.items():
        build().write(name)
    print("wrote " + ", ".join(MESHES))
