import sys

import trimesh

from part2cad.geom import export_pc_to_mesh_ply
from part2cad.loader import load_det_scene, load_gt_scene
from part2cad.utils import to_color_trimesh_pcd, to_o3d_pcd, o3d_pcd_to_mesh, o3d_mesh_to_trimesh
from part2cad.utils import export_mesh_to_ply, mkdir, save_json, show_o3d_object


OBJ_ID_TO_SEMANTIC = {
    0: "Table",
    1: "Chair",
    2: "Bed",
    3: "Cabinet",
    4: "Cup",
    5: "Microwave",
    6: "Oven",
    7: "TV"
}


class PanopticMap(object):


    def __init__(self, scene_name):
        self.scene_name_ = scene_name

        self.id2cad_ = dict()
        self.id_cnt_ = 1

        self.create_dummy_background_()


    def add_instance(self, instance_pts, obj_type):
        pcd = to_o3d_pcd(instance_pts)
        mesh = o3d_mesh_to_trimesh( o3d_pcd_to_mesh(pcd) )
        
        self.add_instance_mesh_(mesh, obj_type)


    def add_background(self, bg_pts, obj_type):
        pcd = to_color_trimesh_pcd(bg_pts)
        self.add_instance_mesh_(pcd, obj_type)
        

    def export(self, output_dir):
        output_dir = "{}/{}/panoptic_segments".format(output_dir, self.scene_name_)
        mkdir(output_dir)

        for idx, (mesh, obj_type) in self.id2cad_.items():
            if obj_type in ["Floor"]:
                export_pc_to_mesh_ply(mesh, "{}/{}.ply".format(output_dir, idx))
            else:
                export_mesh_to_ply(mesh, "{}/{}.ply".format(output_dir, idx))
        
        id2type = {k: v[1] for k, v in self.id2cad_.items()}
        save_json(id2type, "{}/id.json".format(output_dir))

    
    def add_instance_mesh_(self, ins_mesh, obj_type):
        print("Instance {} added".format(self.id_cnt_))

        self.id2cad_[self.id_cnt_] = (ins_mesh, obj_type)
        self.id_cnt_ += 1


    def create_dummy_background_(self):
        mesh = trimesh.creation.icosphere(radius=0.005)

        self.add_instance_mesh_(mesh, "Background")
        self.add_instance_mesh_(mesh, "Wall")


def export_panoptic_map(scene_root, export_gt=True):
    if export_gt:
        bg_points, obj_points_list = load_gt_scene(scene_root)
        obj_types = [OBJ_ID_TO_SEMANTIC[int(points[0][6])] for points in obj_points_list]
    else:
        bg_points, obj_points_list, obj_types = load_det_scene(scene_root)
        obj_types = [OBJ_ID_TO_SEMANTIC[t] for t in obj_types]
    
    scene_name = scene_root.split('/')[-1]
    pm = PanopticMap(scene_name)

    for points, obj_type in zip(obj_points_list, obj_types):
        pm.add_instance(points, obj_type)

    pm.add_background(bg_points, "Floor")
    
    pm.export("./")
    

if __name__ == "__main__":
    export_gt = False

    if sys.argv[1] == 'true':
        export_gt = True
    elif sys.argv[1] != 'false':
        print("Arguement ERROR!")
        exit(0)


    for root_dir in sys.argv[2:]:
        export_panoptic_map(root_dir, export_gt=export_gt) 