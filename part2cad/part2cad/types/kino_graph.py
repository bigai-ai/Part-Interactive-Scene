import json
import copy

import trimesh
import networkx as nx

from part2cad.constants import ASSET_DIR
from part2cad.constants import SCENE_GRAPH_FILE
from part2cad.utils import mkdir


class PartGraph(object):
    
    def __init__(self, nxg=None, root=0):
        if nxg is None:
            nxg = nx.DiGraph()
            nxg.add_node(root)

        self.nxg_ = nxg.copy()

        self.nodes_ = dict()
        self.node_meshes_ = dict()
        self.root_idx_ = root

        for idx in self.node_indices:
            self.nodes_[idx] = dict()

    @property
    def root_idx(self):
        assert(self.root_idx_ is not None)
        return self.root_idx_


    @property
    def node_indices(self):
        return list(self.nxg_.nodes())

    @property
    def edges(self):
        return self.nxg_.edges


    @property
    def n_nodes(self):
        return self.nxg_.number_of_nodes()


    def parent(self, idx):
        parent = list(self.nxg_.predecessors(idx))
        
        if len(parent) == 0:
            return None

        return parent[0]


    def children(self, idx):
        return list(self.nxg_.successors(idx))


    def set_node_info(self, node_idx, mesh, attributes):
        if node_idx not in self.nodes_:
            raise Exception("Node ID: `{}` does not exist".format(node_idx))
        
        for k, v in attributes.items():
            self.nodes_[node_idx][k] = v

        self.node_meshes_[node_idx] = mesh


    def offset_idx(self, offset):
        # update root index
        self.root_idx_ += offset

        # re-index nx.DiGraph
        g = self.nxg_
        
        self.nxg_ = nx.DiGraph()
        if g.number_of_nodes() == 1:
            self.nxg_.add_node(self.root_idx)
        else:
            self.nxg_.add_edges_from([(e[0] + offset, e[1] + offset) for e in g.edges])

        # update meta information in terms of new indices
        self.nodes_ = {k + offset: v for k, v in self.nodes_.items()}
        self.node_meshes_ = {k + offset: v for k, v in self.node_meshes_.items()}

        for k, v in self.nodes_.items():
            v["id"] = k
            v["cad_id"] = k


    def save_mesh(self, output_dir):
        mkdir(output_dir)

        for id, m in self.node_meshes_.items():
            if m is None:
                continue
            
            if isinstance(m, trimesh.PointCloud):
                m.export("{}/{}.ply".format(output_dir, id))
            else:
                m.export("{}/{}.stl".format(output_dir, id))


    def dump(self):
        gjson = {
            "edges": [],
            "nodes": [],
            "root_id": self.root_idx
        }

        for e in self.edges:
            gjson["edges"].append(
                {
                    "dst_id": e[1],
                    "src_id": e[0]
                }
            )

        for node_idx, meta in self.nodes_.items():
            gjson["nodes"].append(
                {k: v for k, v in meta.items()}
            )

        return gjson


class KinoGraph(object):

    def __init__(self, root_idx, object_pgs, edges):
        self.root_idx_ = root_idx

        self.edges_ = copy.deepcopy(edges)
        self.obj_graphs_ = object_pgs


    def dump(self):
        gjson = {
            "edges": [],
            "nodes": [],
            "root_id": self.root_idx_
        }

        # nodes and edges of each objects
        for og in self.obj_graphs_:
            data = og.dump()
            gjson["edges"].extend(data["edges"])
            gjson["nodes"].extend(data["nodes"])

        # contextual relations
        for e in self.edges_:
            gjson["edges"].append(
                {
                    "dst_id": e[1],
                    "src_id": e[0]
                }
            )

        return gjson


    def save(self, output_dir):
        mkdir(output_dir)
        
        gjson = self.dump()
        with open("{}/{}".format(output_dir, SCENE_GRAPH_FILE), "w") as fout:
            fout.write(json.dumps(gjson, indent=4))
        
        asset_dir = "{}/{}".format(output_dir, ASSET_DIR)
        for og in self.obj_graphs_:
            og.save_mesh(asset_dir)