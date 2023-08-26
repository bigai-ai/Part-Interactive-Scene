import numpy as np
from sklearn.neighbors import KDTree
from transforms3d.affines import compose
import trimesh

from part2cad.loader import load_object_sample
from part2cad.types import parse_seg_object_pointclouds
from part2cad.visualization import show_part_pointclouds, show_part_cads
from part2cad.core.cad_replacement import object_to_part_cad


def get_point_color(raw_pts, ref_pts, ref_colors):
    tree = KDTree(ref_pts)

    _, indices = tree.query(raw_pts, k=1)
    indices = indices.reshape((-1,))

    return ref_colors[indices]


# def get_point_color(raw_pts, ref_pts, ref_colors, chunk_size=50):
#     color_pts = np.zeros((raw_pts.shape[0], 4), dtype="uint8")
#     ref_chunk = np.repeat(ref_pts[np.newaxis, :, :], chunk_size, axis=0)

#     for i in range(0, raw_pts.shape[0], chunk_size):
#         if chunk_size + i > raw_pts.shape[0]:
#             chunk_size = raw_pts.shape[0] - i
#             ref_chunk = np.repeat(ref_pts[np.newaxis, :, :], chunk_size, axis=0)

#         pts = raw_pts[i:i+chunk_size, :].reshape((chunk_size, 1, -1))
#         diff = np.sum((pts - ref_chunk) ** 2, axis=2)
#         color_pts[i:i+chunk_size, :] = ref_colors[np.argmin(diff, axis=1)]
    
#     return color_pts


def show_object_sample(scene_root_dir, object_dir):
    raw_pts, gt_pts, scene_pcd = load_object_sample(scene_root_dir, object_dir)

    ref_pts = np.array(scene_pcd.vertices.tolist())
    ref_colors = np.array(scene_pcd.colors.tolist())

    pcds = parse_seg_object_pointclouds(gt_pts)
    
    distance = 2
    roll = 1.1
    pitch = 0
    yaw = -1.9
    angles = [roll, pitch, yaw]
    # show_part_pointclouds(pcds, angles=angles, distance=distance)

    raw_color_pts = get_point_color(raw_pts, ref_pts, ref_colors)
    raw_pcd = trimesh.PointCloud(raw_pts, colors=raw_color_pts)
    
    scene = trimesh.Scene([raw_pcd])
    scene.set_camera(distance=distance, angles=angles)
    # scene.show()

    obj_pcs = parse_seg_object_pointclouds(gt_pts)
    mesh_states = object_to_part_cad(obj_pcs, enable_scale=True)

    show_part_cads(mesh_states, angles=angles, distance=distance)


if __name__ == "__main__":
    # scene_root_dir = "/home/zeyu/Workspace/OnGoing/Part-SLAM-ROS/dataset/scannet/scene0477_01"
    # object_dir = "scene0477_01-37746-7-Chair"

    scene_root_dir = "/home/ribet/Synology_Shared/ICRA23-PartSlam/dataset/scannet/scene0335_01"
    object_dir = "scene0335_01-7297-10-Microwave"

    show_object_sample(scene_root_dir, object_dir)







    # del obj_pcs[0]

    # mesh_states = object_to_part_cad(obj_pcs, enable_scale=True)

    # handle, _, _ = mesh_states[0]
    # T = compose([0.01, 0, 0], np.eye(3), np.ones(3))
    # handle.apply_transform(T)

    # screen, _, _ = mesh_states[2]
    # T = compose([-0.01, 0, 0], np.eye(3), np.ones(3))
    # screen.apply_transform(T)