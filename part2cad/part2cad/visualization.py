
import trimesh
from matplotlib.pyplot import cm
import random

import numpy as np
import igraph as ig

from part2cad.utils import mkdir

def create_palette(labels, shuffle=False):
    labels = [i for i in labels]

    if shuffle:
        random.shuffle(labels)

    colors = cm.rainbow(np.linspace(0, 1, len(labels)))

    palette = dict()
    for label, color in zip(labels, colors):
        palette[label] = (color * 255).astype("uint8")

    return palette


def show_part_pointclouds(pcs, angles=None, distance=None):
    """Show colored point clouds

    Args:
        pcs (list of PartPointCloud): a list of PointCloud objects
        angle (list of 3 floats): camera angles of rendering camera
        distance (float): distance of rendering camera towards to the origin
    """
    part_labels = [pc.part_id for pc in pcs]
    palette = create_palette(np.unique(part_labels))

    trimesh_pcs = []
    for pc in pcs:
        trimesh_pcs.append(
            pc.to_trimesh_pc(palette[pc.part_id])
        )

    scene = trimesh.Scene(geometry=trimesh_pcs)
    scene.set_camera(angles=angles, distance=distance)
    scene.show()


def show_part_cads(mesh_states, angles=None, distance=None):
    """Show whole object from mesh parts

    Args:
        mesh_states (list of tuple): a list of tuple of 3 elements,
            mesh (trimesh.Trimesh): mesh of the part
            tf (4x4 matrix): a homogenenous transformation matrix
            metadata (dictionary): a dictionary of metadata
        angle (list of 3 floats): camera angles of rendering camera
        distance (float): distance of rendering camera towards to the origin
    """
    part_cads = []
    part_types = [s[2]["part_id"] for s in mesh_states]

    palette = create_palette(np.unique(part_types))

    idx = 0
    mkdir("mesh")
    for mesh, tf, meta in mesh_states:
        m = mesh.copy()
        m.apply_transform(tf)
        m.visual.vertex_colors = palette[meta["part_id"]]

        part_cads.append(m)

        print("Exporting part {}...".format(idx))
        m.export("mesh/{}.obj".format(idx))
        # m.show()
        idx += 1
    
    scene = trimesh.Scene(geometry=part_cads)
    scene.set_camera(angles=angles, distance=distance)
    scene.show()
    scene.export("mesh/cad.obj")


def show_nx_graph(nxg):
    """Show Graph created by NetworkX

    Args:
        nxg (networkx.Graph): Graph object
    """
    node_set = set(nxg.nodes)
    for e in nxg.edges:
        if e[1] in node_set:
            node_set.remove(e[1])
    root = list(node_set)[0]

    g = ig.Graph(nxg.edges, directed=True)

    layout = g.layout("rt", root=root)

    visual_style = {
        "edge_width": 2,
        # "edge_color": [],
        # "edge_curved": [],
        "edge_arrow_size": 1,
        # "edge_arrow_width": 2,
        # "vertex_label_angle": [],
        "vertex_label_dist": 1.1,
        "vertex_label_size": 15,
        "vertex_label": [i for i in range(g.vcount())],
        "vertex_size": 30,
        # "vertex_color": self.generate_vertex_color_(g),
        # "vertex_shape": self.generate_vertex_shape_(g),
        # "autocurve": False,
        "bbox": (1000, 500),
        "margin": 50,
        "layout": layout
    }

    ig.plot(g, **visual_style)


def to_color_pcd(points):
    colors_rgba = np.ones((len(points), 4), dtype="uint8") * 255
    colors_rgba[:, 0:3] = (points[:, 3:6] * 255).astype("uint8")

    pcd = trimesh.PointCloud(points[:, :3], colors=colors_rgba)
    return pcd