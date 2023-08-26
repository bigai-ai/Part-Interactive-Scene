import numpy as np
import trimesh
from transforms3d.affines import compose


def normalize_vec(v):
    return v / np.sqrt(np.sum(v ** 2))


def centerialize_mesh(mesh):
    trans = np.average(mesh.bounds, axis=0)
    
    tf = compose(-trans, np.eye(3), np.ones(3))
    mesh.apply_transform(tf)

    return mesh


def centeralize_points(data):
    # deep copy the array
    data = data.copy()

    points = data[:, :3]
    normals = data[:, 3:6]

    pc = trimesh.PointCloud(points)
    pcn = trimesh.PointCloud(normals)

    trans = np.average(pc.bounds, axis=0)
    pc.apply_transform(compose(-trans, np.eye(3), np.ones(3)))
    pcn.apply_transform(compose(-trans, np.eye(3), np.ones(3)))

    data[:, :3] = pc.vertices
    data[:, 3:6] = pcn.vertices

    global_tf = compose(trans, np.eye(3), np.ones(3))

    return data, global_tf


def opt_rot_a2b(V_a, V_b):
    """Rotation a set of vector A to another set of vector B

    Args:
        V_a (list of vec3): each row is vec3
        V_b (list of vec3): each row is vec3

    Returns:
        3x3 matrix: rotation matrix
    """
    V_a = np.array(V_a).T
    V_b = np.array(V_b).T

    H = np.dot(V_a, V_b.T)
    U, S, V = np.linalg.svd(H)

    R = np.dot(V.T, U.T)

    if np.linalg.det(R) < 0:
        V[2, :] *= -1
        R = np.dot(V.T, U.T)
    
    return R