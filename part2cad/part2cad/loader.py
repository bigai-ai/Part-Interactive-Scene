import os
import glob

import trimesh
import numpy as np


BACKGROUND_NPY_FILE = "scen_without_objects_nature_color.npy"
RAW_OBJECT_FILENAME = "source_object_point.npy"
GT_OBJECT_FILENAME = "gt_partnet_in_sannet_xyz_normals_24id_307id_ourid.npy"
SEG_INPUT_FILENAME = "6144_gt_input.npy"
SEG_LABEL_FILENAME = "net_pred_id.npy"
STRUCTURENET_OBJECT_FILENAME = "source_object_point_xyz_partid_structurenet.npy"
STRUCTURENET_OBJECT_SCENENN_FILENAME = "gt_partnet_in_sannet_xyz_normals_24id_307id_ourid_xyz_partid_structurenet.npy"
RAW_SCENE_PLY = "soure_scen_point_clouds.ply"


def get_object_dir(scene_root):
    obj_dirs = glob.glob("{}/*/".format(scene_root))

    obj_dirs = [d[:-1] for d in obj_dirs]

    return obj_dirs


def load_seg_scene(scene_root):
    bg_file = "{}/{}".format(scene_root, BACKGROUND_NPY_FILE)
    obj_dirs = get_object_dir(scene_root)
    
    input_obj_files = ["{}/{}".format(d, SEG_INPUT_FILENAME) for d in obj_dirs]
    part_label_files = ["{}/{}".format(d, SEG_LABEL_FILENAME) for d in obj_dirs]

    bg_points = np.load(bg_file)
    obj_points_list = [np.load(f) for f in input_obj_files]
    part_labels_list = [np.load(f) for f in part_label_files]

    for points, labels in zip(obj_points_list, part_labels_list):
        points[:, 7] = labels
        points[:, 8] = labels

    return bg_points, obj_points_list


def load_raw_scene(scene_root):
    bg_file = "{}/{}".format(scene_root, BACKGROUND_NPY_FILE)
    obj_dirs = get_object_dir(scene_root)
    raw_obj_files = ["{}/{}".format(d, RAW_OBJECT_FILENAME) for d in obj_dirs]

    bg_points = np.load(bg_file)
    obj_points_list = [np.load(f) for f in raw_obj_files]

    return bg_points, obj_points_list


def load_det_scene(scene_root):
    bg_file = "{}/{}".format(scene_root, BACKGROUND_NPY_FILE)
    obj_dirs = get_object_dir(scene_root)
    raw_obj_files = ["{}/{}".format(d, RAW_OBJECT_FILENAME) for d in obj_dirs]
    gt_obj_files = ["{}/{}".format(d, GT_OBJECT_FILENAME) for d in obj_dirs]

    bg_points = np.load(bg_file)
    obj_points_list = [np.load(f) for f in raw_obj_files]
    gt_pts_list = [np.load(f) for f in gt_obj_files]
    obj_types = [int(pts[0][6]) for pts in gt_pts_list]

    return bg_points, obj_points_list, obj_types


def load_gt_scene(scene_root):
    bg_file = "{}/{}".format(scene_root, BACKGROUND_NPY_FILE)
    obj_dirs = get_object_dir(scene_root)
    gt_obj_files = ["{}/{}".format(d, GT_OBJECT_FILENAME) for d in obj_dirs]

    bg_points = np.load(bg_file)
    obj_points_list = [np.load(f) for f in gt_obj_files]

    return bg_points, obj_points_list


def load_structurenet_scene(scene_root):
    bg_file = "{}/{}".format(scene_root, BACKGROUND_NPY_FILE)
    obj_dirs = get_object_dir(scene_root)

    obj_files, gt_files = [], []

    for d in obj_dirs:
        file_dir = "{}/{}".format(d, STRUCTURENET_OBJECT_FILENAME)
        gt_dir = "{}/{}".format(d, GT_OBJECT_FILENAME)

        if not os.path.isfile(file_dir):
            file_dir = "{}/{}".format(d, STRUCTURENET_OBJECT_SCENENN_FILENAME)

        if not os.path.isfile(file_dir):
            file_dir = gt_dir
        
        obj_files.append(file_dir)
        gt_files.append(gt_dir)

    obj_points_list = []
    
    for pf, gf in zip(obj_files, gt_files):
        points = np.load(pf)

        if points.shape[1] == 4:
            gt_pts = np.load(gf)

            tmp = np.ones((points.shape[0], 9))
            tmp[:, :3] = points[:, :3]
            tmp[:, 6] = gt_pts[0, 6]
            tmp[:, 7] = points[:, 3]
            tmp[:, 8] = points[:, 3]

            points = tmp
        
        obj_points_list.append(points)

    bg_points = np.load(bg_file)

    return bg_points, obj_points_list


def load_object_sample(scene_root, object_dir):
    gt_object_file = "{}/{}/{}".format(scene_root, object_dir, GT_OBJECT_FILENAME)
    raw_object_file = "{}/{}/{}".format(scene_root, object_dir, RAW_OBJECT_FILENAME)
    raw_scene_file = "{}/{}".format(scene_root, RAW_SCENE_PLY)

    raw_scene_ply = trimesh.load(raw_scene_file)
    raw_obj_pts = np.load(raw_object_file)
    gt_obj_pts = np.load(gt_object_file)

    return raw_obj_pts, gt_obj_pts, raw_scene_ply
