import os
import sys
import argparse

from part2cad.types import parse_seg_object_pointclouds
from part2cad.loader import load_gt_scene, load_structurenet_scene
from part2cad.core import CadScene
from part2cad.visualization import show_part_pointclouds

##############################################################
# Convert scene to part-based CAD objects
##############################################################
def cvt_scene(scene_root_dir, loader_mode):
    if loader_mode == "gt":
        print("Load from ground-truth outputs")
        bg_points, obj_points_list = load_gt_scene(scene_root_dir)
    elif loader_mode == "snet":
        print("Load from structurenet outputs")
        bg_points, obj_points_list = load_structurenet_scene(scene_root_dir)
    
    scene = CadScene()

    scene.add_background(bg_points)

    for obj_points in obj_points_list:
        obj_pcs = parse_seg_object_pointclouds(obj_points)
        scene.add_object(obj_pcs)

    kgraph = scene.create_kino_graph()
    
    output_dir = os.path.join("scene_builder", "input", scene_root_dir.split('/')[-1])
    kgraph.save(output_dir)


def arg_parser():
    parser = argparse.ArgumentParser(prog='Convert Part Scene')
    parser.add_argument(
        "--src",
        dest="src",
        type=str,
        required=True,
        help="Input scene directory"
    )
    parser.add_argument(
        "--loader",
        dest="loader",
        type=str,
        required=True,
        help="Loader mode: <gt>, <snet>"
    )
    # by default args.output == False
    parser.add_argument("-v", "--verbose", action="store_true")
    
    args = parser.parse_args()

    if args.loader not in ["gt", "snet"]:
        raise Exception("Does not support loader: `{}`".format(args.loader))
    
    return args


if __name__ == "__main__":
    args = arg_parser()

    scene_dir = args.src
    loader_mode = args.loader

    cvt_scene(scene_dir, loader_mode)