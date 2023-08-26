import sys
import numpy as np

from part2cad.types import parse_seg_object_pointclouds
from part2cad.visualization import show_nx_graph, show_part_pointclouds, show_part_cads
from part2cad.core import object_to_part_cad, assemble_object


#################################################################
# Convert segmented object point clouds to part CADs
#################################################################
def cvt_object(object_npy_file):
    data = np.load(object_npy_file)
    
    pcs = parse_seg_object_pointclouds(data)
    show_part_pointclouds(pcs)

    mesh_states = object_to_part_cad(pcs, enable_scale=True)
    show_part_cads(mesh_states)

    pg, mesh_states = assemble_object(mesh_states)
    show_nx_graph(pg.nxg_)
    show_part_cads(mesh_states)


if __name__ == "__main__":
    cvt_object(sys.argv[1])