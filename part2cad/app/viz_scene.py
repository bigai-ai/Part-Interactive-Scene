import sys

from part2cad.loader import load_gt_scene, load_raw_scene
from part2cad.types import ScenePointCloud


def viz_gt_scene(scene_root_dir):
    bg_points, obj_points_list = load_gt_scene(scene_root_dir)

    scene_pcd = ScenePointCloud(bg_points, obj_points_list)
    scene_pcd.show(viz_part=True)

    output_dir = scene_root_dir.split('/')[-1] + "_gt.ply"
    scene_pcd.save(output_dir, viz_part=True)
    print("Scene GT pointcloud was saved at: {}".format(output_dir))


def viz_raw_scene(scene_root_dir):
    bg_points, obj_points_list = load_raw_scene(scene_root_dir)

    scene_pcd = ScenePointCloud(bg_points, obj_points_list)
    scene_pcd.show(viz_part=False)

    output_dir = scene_root_dir.split('/')[-1] + "_raw.ply"
    scene_pcd.save(output_dir, viz_part=False)
    print("Scene raw pointcloud was saved at: {}".format(output_dir))


if __name__ == "__main__":
    viz_gt_scene(sys.argv[1])