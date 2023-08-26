import os
import json

import numpy as np
import open3d as o3d
import trimesh

from part2cad.constants import RAW_OBJECT_FILENAME, GT_OBJECT_FILENAME
from part2cad.constants import SEG_INPUT_FILENAME, COMPLETE_OBJECT_FILENAME


def mkdir(target):
    if os.path.isdir(target):
        return
    
    os.system("mkdir -p {}".format(target))


def transform_points(points, tf):
    pts_mat = np.ones((points.shape[0], 4), dtype="float")
    pts_mat = np.dot(tf, pts_mat.T).T

    return pts_mat[:, :3]


def transform_all_files(object_root, tf):
    filename = "{}/{}".format(object_root, SEG_INPUT_FILENAME)
    data = np.load(filename)
    data[:, :3] = transform_points(data[:, :3], tf)
    data[:, 3:6] = transform_points(data[:, 3:6], tf)
    np.save(SEG_INPUT_FILENAME, data)

    filename = "{}/{}".format(object_root, GT_OBJECT_FILENAME)
    data = np.load(filename)
    data[:, :3] = transform_points(data[:, :3], tf)
    data[:, 3:6] = transform_points(data[:, 3:6], tf)
    np.save(GT_OBJECT_FILENAME, data)

    filename = "{}/{}".format(object_root, RAW_OBJECT_FILENAME)
    data = np.load(filename)
    data[:, :3] = transform_points(data[:, :3], tf)
    np.save(RAW_OBJECT_FILENAME, data)

    filename = "{}/{}".format(object_root, COMPLETE_OBJECT_FILENAME)
    data = np.load(filename)
    data[:, :3] = transform_points(data[:, :3], tf)
    np.save(COMPLETE_OBJECT_FILENAME, data)


def export_trimesh_to_ply(mesh, output_dir):
    res = trimesh.exchange.ply.export_ply(mesh, encoding='ascii', vertex_normal=True)
    with open(output_dir, "wb") as fout:
        fout.write(res)


def to_o3d_pcd(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(20))

    return pcd


def to_o3d_color_pcd(data):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:,:3])
    pcd.colors = o3d.utility.Vector3dVector(data[:, 3:6])

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(20))

    return pcd


def show_o3d_object(pcd):
    o3d.visualization.draw_geometries([pcd])


def o3d_pcd_to_mesh(pcd):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,o3d.utility.DoubleVector([radius, radius * 2]))

    return bpa_mesh


def o3d_mesh_to_trimesh(o3d_mesh):
    m = trimesh.Trimesh(vertices=o3d_mesh.vertices, vertex_normals=o3d_mesh.vertex_normals,
        faces=o3d_mesh.triangles)
    
    return m 


def to_color_trimesh_pcd(points):
    """To colored Trimesh.PointCloud

    Args:
        points ((n_points, 6) np.ndarray): a list of points, where each point
            is represented by <x, y, z, r, g, b>
    """
    colors_rgba = np.ones((len(points), 4), dtype="uint8") * 255
    colors_rgba[:, 0:3] = (points[:, 3:6] * 255).astype("uint8")

    pcd = trimesh.PointCloud(points[:, :3], colors=colors_rgba)
    return pcd


def export_mesh_to_ply(mesh, output_dir):
    res = trimesh.exchange.ply.export_ply(mesh, encoding='ascii', vertex_normal=True)
    with open(output_dir, "wb") as fout:
        fout.write(res)


def save_json(dic, output_dir):
    with open(output_dir, "w") as fout:
        fout.write(json.dumps(dic, indent=4))
    
    print("JSON saved at: {}".format(output_dir))