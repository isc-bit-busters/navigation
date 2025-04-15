from pathfinder import navmesh_baker as nmb
import pathfinder as pf
import numpy
import trimesh
import time

from shapely.geometry import LineString, Point, box, Polygon
import math

# We're generating our navmesh in pixel space, but the navmesh baker
# gets really slow with a scale that big (in the thousandss).
# So we scale down the polygons to a more reasonable size.
SCALE = 50

def generate_navmesh(input_file="polygons.txt", output_file="navmesh.txt"):
    polygons = numpy.fromfile(input_file, dtype=numpy.int32)
    polygons = polygons.reshape(-1, 4)
    polygons = polygons / SCALE
    max_x, max_y = polygons[:, 0].max(), polygons[:, 1].max()

    # polygons = polygons[(polygons[:, 2] > 0.1) & (polygons[:, 3] > 0.1)]

    wall_blocks = []
    for wall in polygons:
        x, y, w, h = wall
        w = abs(w - x)
        h = abs(h - y)

        # Ensure the wall is not too small
        if w < 0.1:
            w = 0.1
        if h < 0.1:
            h = 0.1

        block = trimesh.creation.box(extents=(w, 6, h))

        # Trimesh mesh position is at the center of the mesh 
        # Our polygons are defined by the top left corner
        block.apply_translation((x + w / 2, 0, y + h / 2))

        wall_blocks.append(block)

    plane = trimesh.creation.box(extents=(max_x, 0.1, max_y))
    plane.apply_translation((max_x/2, 0, max_y/2))

    meshes = [plane, *wall_blocks]

    all_vertices = []
    all_faces = []
    vertex_offset = 0

    # Convert meshes to vertices + faces as list vertices indices
    for mesh in meshes:
        vertices = mesh.vertices
        faces = mesh.faces

        all_vertices.extend(vertices)

        offset_faces = []
        for face in faces:
            offset_face = [idx + vertex_offset for idx in face]
            offset_faces.append(offset_face)

        all_faces.extend(offset_faces)

        vertex_offset += len(vertices)

    baker = nmb.NavmeshBaker()

    baker.add_geometry(
        vertices=all_vertices,
        polygons=all_faces,
    )

    baker.bake(
        agent_radius=0.4,   # TODO: unhardcode this
        cell_size=0.0333,   # TODO: unhardcode this
        verts_per_poly=3
    )

    baker.save_to_text(output_file)

def find_path(start, end, navmesh_file="navmesh.txt"):
    start = numpy.array(start) / SCALE
    end = numpy.array(end) / SCALE
    vertices, polygons = pf.read_from_text(navmesh_file)
    pathfinder = pf.PathFinder(vertices, polygons)

    start = pathfinder.sample(start)
    end = pathfinder.sample(end)

    path = pathfinder.search_path(start, end)

    if path and not path_is_blocked(path, obstacles):
        print("Path not blocked")
        pass
    else:
        print("Path is blocked")
        path = None

    if path is not None:
        path = numpy.array(path) * SCALE

    return path
def create_oriented_rectangle(center, width, height, angle_deg):
    cx, cy = center
    angle_rad = math.radians(angle_deg)

    hw, hh = width / 2, height / 2

    local_corners = [
        (-hw, -hh),
        ( hw, -hh),
        ( hw,  hh),
        (-hw,  hh)
    ]

    world_corners = []
    for x, y in local_corners:
        xr = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        yr = x * math.sin(angle_rad) + y * math.cos(angle_rad)
        world_corners.append((cx + xr, cy + yr))

    return Polygon(world_corners)


obstacles = [
    # TODO : be careful to add SCALE conversion, and add car size
    {"type": "circle", "center": (9.0, 4.0), "radius": 0.5},
    # {"type": "aabb", "min": (6.0, 5.0), "max": (5.0, 3.0)},
    {"type": "polygon", "shape": create_oriented_rectangle(center=(5, 5), width=3, height=2, angle_deg=30)}
]


def path_is_blocked(path, obstacles):
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        print(f"Checking path segment {a} to {b}")

        a = (a[0], a[2])
        b = (b[0], b[2])
        for obs in obstacles:
            if obs["type"] == "circle" and line_intersects_circle(a, b, obs["center"], obs["radius"]):
                print(f"Path intersects circle at {obs['center']} with radius {obs['radius']}")
                return True
            if obs["type"] == "aabb" and line_intersects_aabb(a, b, obs["min"], obs["max"]):
                print(f"Path intersects AABB at {obs['min']} to {obs['max']}")
                return True
            if obs["type"] == "polygon" and line_intersects_polygon(a, b, obs["shape"]):
                print(f"Path intersects polygon at {obs['shape']}")
                return True
    return False

def line_intersects_circle(p1, p2, center, radius):
    return LineString([p1, p2]).distance(Point(center)) <= radius

def line_intersects_aabb(p1, p2, min_pt, max_pt):
    return LineString([p1, p2]).intersects(box(*min_pt, *max_pt))

def line_intersects_polygon(p1, p2, polygon):
    return LineString([p1, p2]).intersects(polygon)


def find_blocked_segments(path, obstacles):
    blocked_segments = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        a = (a[0], a[2])
        b = (b[0], b[2])
        for obs in obstacles:
            if obs["type"] == "circle" and line_intersects_circle(a, b, obs["center"], obs["radius"]):
                blocked_segments.append((a, b))
            if obs["type"] == "aabb" and line_intersects_aabb(a, b, obs["min"], obs["max"]):
                blocked_segments.append((a, b))
            if obs["type"] == "polygon" and line_intersects_polygon(a, b, obs["shape"]):
                blocked_segments.append((a, b))
    return blocked_segments


if __name__ == "__main__":
    start_time = time.time()
    generate_navmesh(
        input_file="polygons.txt",
        output_file="navmesh.txt"
    )

    print(f"Navmesh generation took {time.time() - start_time:.2f} seconds")
    start_time = time.time()

    sl = find_path(
        start=(300, 0, 100), 
        end=(100, 0, 100),
        navmesh_file="navmesh.txt"
    )
    print(f"Pathfinding took {time.time() - start_time:.2f} seconds")

    print("Path:")
    print(sl)
