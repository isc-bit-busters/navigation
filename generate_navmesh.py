from pathfinder import navmesh_baker as nmb
import numpy
import trimesh

def generate_navmesh(input_file="polygons.txt", output_file="navmesh.txt"):
    polygons = numpy.fromfile(input_file, dtype=numpy.int32)
    polygons = polygons.reshape(-1, 4)
    polygons = polygons / 100   # TODO: pixel-space to world-space
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
        cell_size=0.1,      # TODO: unhardcode this
        verts_per_poly=3
    )

    baker.save_to_text(output_file)

    # vertices, polygons = baker.get_polygonization()


if __name__ == "__main__":
    generate_navmesh()