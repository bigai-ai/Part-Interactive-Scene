import random
import trimesh
import numpy as np
from transforms3d.quaternions import mat2quat

from part2cad.core.cad_assemble import assemble_object
from part2cad.core.cad_replacement import object_to_part_cad
from part2cad.types import PartGraph, KinoGraph


class CadScene(object):

    def __init__(self):
        self.id_cnt_ = 0

        self.objects_ = []
        self.backgrounds_ = []

        self.rg_ = self.create_scene_root_()

        random.seed(10)

    
    def next_object_idx_(self):
        return len(self.objects_)

    
    def create_scene_root_(self):
        pgraph = PartGraph(root=self.id_cnt_)

        pgraph.set_node_info(
            self.id_cnt_,
            None,
            {
                "id": self.id_cnt_,
                "label": "scene",
                "type": "ConceptNode"
            }
        )

        self.id_cnt_ += 1
        return pgraph


    def add_object(self, part_pcs):
        mesh_states = object_to_part_cad(part_pcs, enable_scale=True)
        pg, _ = assemble_object(mesh_states, self.next_object_idx_())
        
        pg.offset_idx(self.id_cnt_)
        
        self.objects_.append(pg)

        self.id_cnt_ += pg.n_nodes


    def add_background(self, points, global_tf=np.eye(4)):
        colors_rgba = np.ones((len(points), 4), dtype="uint8") * 255
        colors_rgba[:, 0:3] = (points[:, 3:6] * 255).astype("uint8")
        pcd = trimesh.PointCloud(points[:, :3], colors=colors_rgba)

        pgraph = PartGraph(root=self.id_cnt_)

        pgraph.set_node_info(
            self.id_cnt_,
            pcd,
            {
                "id": self.id_cnt_,
                "cad_id": self.id_cnt_,
                "label": "Background",
                "part_label": "None",
                "type": "ObjectNode",
                "orientation": mat2quat(global_tf[:3, :3]).tolist(),
                "position": global_tf[:, 3].tolist(),
                "scale": 1,
                "joint_type": "fixed"
            }
        )

        self.id_cnt_ += 1
        self.backgrounds_.append(pgraph)

    
    def create_kino_graph(self):
        contact_relations = []

        # add objects to root
        for og in self.objects_:
            contact_relations.append( (self.rg_.root_idx, og.root_idx) )

        # add background to root
        for bg in self.backgrounds_:
            contact_relations.append( (self.rg_.root_idx, bg.root_idx) )
        
        kinog = KinoGraph(
            self.rg_.root_idx,
            self.objects_ + self.backgrounds_ + [self.rg_],
            contact_relations
        )

        return kinog