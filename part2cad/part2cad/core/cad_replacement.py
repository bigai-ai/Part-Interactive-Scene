import trimesh
import numpy as np

from part2cad.geom import centerialize_mesh



def create_box(extents):
        return trimesh.primitives.Box(extents=extents)


def create_sphere(extents):
    return trimesh.primitives.Sphere(radius=min(extents) / 2, subdivisions=3)


def create_cylinder(extents):
    # m = trimesh.primitives.Cylinder(radius=max(extents) / 2, height=min(extents))
    # mid = np.sum(extents) - max(extents) - min(extents)
    # m.apply_scale([1, mid / max(extents), 1])
    # return m
    return trimesh.primitives.Cylinder(radius=min(extents) / 2, height=max(extents))


def create_capsule(extents):
    cap = trimesh.primitives.Capsule(radius=min(extents) / 2, height=max(extents))
    cap = centerialize_mesh(cap)
    return cap


def create_cone(extents):
    cone = trimesh.creation.cone(radius=min(extents) / 2, height=max(extents) - 0.15)
    cone = centerialize_mesh(cone)
    return cone


def create_part_candidates(pc):
    obb_extents = pc.get_obb_extents()

    if obb_extents is None:
        return []

    candidates = [
        create_box(obb_extents),
        create_sphere(obb_extents),
        create_cylinder(obb_extents),
        create_capsule(obb_extents),
        create_cone(obb_extents)
    ]

    return candidates


def split_tf_scale(tf):
    scale = np.sqrt(np.dot(tf[:3, :3], tf[:3, :3].T)[0, 0])
    tf[:3, :3] /= scale

    return tf, scale


def align_primitive_cad(pc, enable_scale=True):
    candidates = create_part_candidates(pc)
    results = []

    # failed to generate any candidate
    if len(candidates) == 0:
        return None

    for mesh_part in candidates:
        tf, cost = trimesh.registration.mesh_other(mesh_part, pc.points, scale=enable_scale)

        # if the determinant of tf is negative, then take it complement
        if np.linalg.det(tf[:3, :3]) < 0:
            tf[:3, :3] *= -1

        tf, scale = split_tf_scale(tf)
        results.append( (mesh_part, tf, scale, cost) )
    
    results.sort(key=lambda x: x[3])
    
    # return the best results    
    return results[0]


def object_to_part_cad(part_pcs, enable_scale):
    mesh_parts = []

    for pc in part_pcs:
        # skip point cloud with super low resolution
        if pc.n_points < 4:
            continue
        
        mesh_state = align_primitive_cad(pc, enable_scale)

        if mesh_state is None:
            continue

        mesh, tf, scale, cost = mesh_state
        mesh_parts.append( (mesh, tf, {"obj_id": pc.obj_id, "part_id": pc.part_id, "scale": scale, "cost": cost}) )

    return mesh_parts