from queue import Queue

import numpy as np
from transforms3d.affines import compose

from part2cad.geom import opt_rot_a2b, find_near_obb_axis


def refine_part_alignment(mesh_states, nxg, root, theta=0.97):
    if len(mesh_states) == 1:
        return mesh_states

    refined_tf = {i: mesh_states[i][1] for i in range(len(mesh_states))}
    mesh_list = [m.copy() for m, _, _ in mesh_states]

    for i, m in enumerate(mesh_list):
        m.apply_transform(refined_tf[i])
    
    mesh_list, refined_tf = refine_rot_alignment(mesh_list, refined_tf, nxg, root, theta)

    new_mesh_states = [
        [mesh_states[i][0], refined_tf[i], mesh_states[i][2]]
            for i in range(len(mesh_states))
    ]

    return new_mesh_states


def refine_rot_alignment(mesh_list, refined_tf, nxg, root, angle_threshold=0.975):
    q = Queue()

    for c in nxg.successors(root):
        q.put((c, root))

    while not q.empty():
        c, p = q.get()

        for cc in nxg.successors(c):
            q.put( (cc, c) )

        pmesh = mesh_list[p]
        cmesh = mesh_list[c]

        p_axis, c_axis, matched_axis = find_near_obb_axis(pmesh, cmesh, angle_threshold=angle_threshold)

        if len(matched_axis) == 0:
            continue
        
        to_axis = p_axis[matched_axis[:, 0]]
        from_axis = c_axis[matched_axis[:, 1]]

        if len(matched_axis) != 3:
            for i in range(3):
                # keep unmatched axis as the same
                if i not in matched_axis[:, 1]:
                    to_axis = np.vstack([to_axis, c_axis[i].reshape((-1, 3))])
                    from_axis = np.vstack([from_axis, c_axis[i].reshape((-1, 3))])

        # calculate the direction, 1 stands for same direction, -1 for opposite direction
        direction = np.sign(np.diag(np.dot(to_axis, from_axis.T))).reshape((-1, 1))
        to_axis *= direction

        rot = opt_rot_a2b(from_axis, to_axis)
        rot_tf = compose([0, 0, 0], rot, np.ones(3))

        # since the rotation is computed in the coord system at the origin
        # we need to translate the object back to the origin, rotate it, 
        # and translate back to its original place
        trans_origin_tf = compose(-refined_tf[c][:3, 3], np.eye(3), np.ones(3))
        trans_back_tf = compose(refined_tf[c][:3, 3], np.eye(3), np.ones(3))
        rot_tf = np.dot(trans_back_tf, np.dot(rot_tf, trans_origin_tf))

        refined_tf[c] = np.dot(rot_tf, refined_tf[c])
        cmesh.apply_transform(rot_tf)
        mesh_list[c] = cmesh

    return mesh_list, refined_tf