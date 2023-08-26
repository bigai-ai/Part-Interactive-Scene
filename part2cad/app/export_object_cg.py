import numpy as np
import networkx as nx
import json

from part2cad.loader import load_object_sample
from part2cad.types import parse_seg_object_pointclouds
from part2cad.core.cad_replacement import object_to_part_cad
from part2cad.geom import calc_contact_score

from part2cad.utils import mkdir 


def infer_kinematic_graph(mesh_states):
    """Infer kinematic relations between parts in terms of contact
    
    Args:
        mesh_part_states (list of tuple): a list tuple, each tuple
            contains three elements:
                mesh_states (list of tuple): a list of tuple of 3 elements,
                    mesh (trimesh.Trimesh): mesh of the part
                tf (4x4 matrix): a homogenenous transformation matrix
                metadata (dictionary): a dictionary of metadata
    """
    # there is a single part that forms the whole
    if len(mesh_states) == 1:
        return None, 0

    edges = dict()
    for i in range(len(mesh_states)):
        for j in range(i + 1, len(mesh_states)):
            score_ij, score_ji = calc_contact_score(
                mesh_states[i][0], mesh_states[i][1],
                mesh_states[j][0], mesh_states[j][1]
            )
            edges[(i, j)] = np.exp(score_ij)
            edges[(j, i)] = np.exp(score_ji)
    
    return edges

    


def export_object_cg(scene_root_dir, object_dir, output_dir="test.json"):
    raw_pts, gt_pts, scene_pcd = load_object_sample(scene_root_dir, object_dir)
    obj_pcs = parse_seg_object_pointclouds(gt_pts)
    mesh_states = object_to_part_cad(obj_pcs, enable_scale=True)

    edges = infer_kinematic_graph(mesh_states)
    
    edge_rankings = {}
    for i in range(len(mesh_states)):
        ranking = [((i, j), edges[(i, j)]) for j in range(len(mesh_states)) if i != j]
        ranking.sort(key=lambda x: -x[1])

        print(ranking)

        edge_rankings[i] = ranking
    
    with open(output_dir, "w") as fout:
        json.dump(edge_rankings, fout, indent=4)

    print("Object contact graph was saved at: {}".format(output_dir))


if __name__ == "__main__":
    # scene_root_dir = "/home/zeyu/Workspace/OnGoing/Part-SLAM-ROS/dataset/scannet/scene0477_01"
    # object_dir = "scene0477_01-37746-7-Chair"

    scene_root_dir = "/home/ribet/Synology_Shared/ICRA23-PartSlam/dataset/scannet/scene0335_01"
    object_dir = "scene0335_01-7297-10-Microwave"

    mkdir(object_dir)
    output_dir = object_dir + "/" + object_dir + ".json"

    export_object_cg(scene_root_dir, object_dir, output_dir)