import numpy as np
import networkx as nx
from transforms3d.quaternions import mat2quat
from transforms3d.affines import compose

from part2cad.types import PartGraph
from part2cad.geom import calc_contact_score
from part2cad.core.part_alignment import refine_part_alignment
from part2cad.visualization import create_palette
from part2cad.constants import REVOLUTE_PART_ID, PRISMATIC_PART_ID, OBJ_ID_TO_SEMANTIC
from part2cad.constants import GRAVITY_DIRECTION


def infer_kinematic_relation(mesh_states):
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
    
    G = nx.DiGraph()
    G.add_edges_from([(k[0], k[1], {"weight": v}) for k, v in edges.items()])
    
    branching = nx.algorithms.tree.branchings.Edmonds(G)
    mdst = branching.find_optimum(kind='max')

    root = [nidx for nidx, d in mdst.in_degree if d == 0][0]

    return mdst, root


def get_parent_tf(node_id, pg, mesh_states):
    parent_idx = pg.parent(node_id)

    if parent_idx is not None:
        return mesh_states[parent_idx][1].copy()
    else:
        return np.eye(4)


def register_rigid_node(pg, mesh, object_id, nid, global_tf, parent_tf, meta):
    local_tf = np.dot(np.linalg.inv(parent_tf), global_tf)
    
    pg.set_node_info(
        nid,
        mesh,
        {
            "id": nid,
            "cad_id": nid,
            "object_id": object_id,
            "label": OBJ_ID_TO_SEMANTIC[meta["obj_id"]],
            "part_label": int(meta["part_id"]),
            "type": "ObjectNode",
            "orientation": mat2quat(local_tf[:3, :3]).tolist(),
            "position": local_tf[:3, 3].tolist(),
            "scale": meta["scale"],
            "joint_type": "fixed",
            "color": meta["color"].tolist()
        }
    )


def register_revolute_node(pg, mesh, object_idx, nid, tf, parent_tf, meta):
    # each row presents an axis
    xyz_axis = global_tf[:3, :3].T

    # compute rotation axis
    gravity_axis_idx = np.argmax(np.abs(np.dot(GRAVITY_DIRECTION, xyz_axis.T)))
    rot_axis = [1.0 if gravity_axis_idx == i else 0.0 for i in range(3)]

    # move the revolute axis to the edge of the part
    rot_horz_axis_idx = np.argmax([mesh.extents[i] if i != gravity_axis_idx else -1 for i in range(3)])
    rot_horz_extent = mesh.extents[rot_horz_axis_idx]
    # the translation is with respect to the world frame
    rot_axis_trans = xyz_axis[rot_horz_axis_idx] * (rot_horz_extent * 0.5)
    global_tf = np.dot(compose(rot_axis_trans, np.eye(3), np.ones(3)), global_tf)

    # calculate the link offset transform, the move the object to its original place
    # after moving the revolute axis to the edge
    # the translation is with respect to the local frame
    link_offset_trans = np.zeros(3)
    link_offset_trans[rot_horz_axis_idx] = -(rot_horz_extent * 0.5)
    link_offset_quat = [0, 0, 0, 1]

    local_tf = np.dot(np.linalg.inv(parent_tf), global_tf)

    pg.set_node_info(
        id,
        mesh,
        {
            "id": id,
            "cad_id": id,
            "label": OBJ_ID_TO_SEMANTIC[meta["obj_id"]],
            "part_label": int(meta["part_id"]),
            "type": "ObjectNode",
            "orientation": mat2quat(local_tf[:3, :3]).tolist(),
            "position": local_tf[:3, 3].tolist(),
            "scale": meta["scale"],
            "joint_axis": rot_axis,
            "link_offset_position": link_offset_trans.tolist(),
            "link_offset_orientation": link_offset_quat,
            "joint_type": "revolute",
            "color": meta["color"].tolist()
        }
    )

def register_prismatic_node(pg, mesh, object_idx, nid, tf, parent_tf, meta):
    # each row presents an axis
    xyz_axis = global_tf[:3, :3].T

    # compute rotation axis
    gravity_axis_idx = np.argmax(np.abs(np.dot(GRAVITY_DIRECTION, xyz_axis.T)))
    rot_axis = [1.0 if gravity_axis_idx == i else 0.0 for i in range(3)]

    # move the prismatic axis to the edge of the part
    rot_horz_axis_idx = np.argmax([mesh.extents[i] if i != gravity_axis_idx else -1 for i in range(3)])
    rot_horz_extent = mesh.extents[rot_horz_axis_idx]
    # the translation is with respect to the world frame
    rot_axis_trans = xyz_axis[rot_horz_axis_idx] * (rot_horz_extent * 0.5)
    global_tf = np.dot(compose(rot_axis_trans, np.eye(3), np.ones(3)), global_tf)

    # calculate the link offset transform, the move the object to its original place
    # after moving the prismatic axis to the edge
    # the translation is with respect to the local frame
    link_offset_trans = np.zeros(3)
    link_offset_trans[rot_horz_axis_idx] = -(rot_horz_extent * 0.5)
    link_offset_quat = [0, 0, 0, 1]

    local_tf = np.dot(np.linalg.inv(parent_tf), global_tf)

    pg.set_node_info(
        id,
        mesh,
        {
            "id": id,
            "cad_id": id,
            "label": OBJ_ID_TO_SEMANTIC[meta["obj_id"]],
            "part_label": int(meta["part_id"]),
            "type": "ObjectNode",
            "orientation": mat2quat(local_tf[:3, :3]).tolist(),
            "position": local_tf[:3, 3].tolist(),
            "scale": meta["scale"],
            "joint_axis": rot_axis,
            "link_offset_position": link_offset_trans.tolist(),
            "link_offset_orientation": link_offset_quat,
            "joint_type": "prismatic",
            "color": meta["color"].tolist()
        }
    )


def assemble_object(mesh_states, object_idx=-1, refine_alignment=True):
    # infer kinematic relations between parts
    nxg, root = infer_kinematic_relation(mesh_states)

    if refine_alignment:
        mesh_states = refine_part_alignment(mesh_states, nxg, root)

    pg = PartGraph(nxg, root)

    # set node meta information
    palette = create_palette(pg.node_indices, shuffle=True)
    for nid, (mesh, tf, meta) in zip(pg.node_indices, mesh_states):
        parent_tf = get_parent_tf(nid, pg, mesh_states)
        meta["color"] = palette[nid] / 255

        if meta["part_id"] in REVOLUTE_PART_ID:
            register_revolute_node(pg, mesh, object_idx, nid, tf, parent_tf, meta)
        elif meta["part_id"] not in PRISMATIC_PART_ID:
            register_prismatic_node(pg, mesh, object_idx, nid, tf, parent_tf, meta)  
        else:
            register_rigid_node(pg, mesh, object_idx, nid, tf, parent_tf, meta)
            
    return pg, mesh_states
