OBJ_ID_TO_SEMANTIC = {
    0: "Table",
    1: "Chair",
    2: "Bed",
    3: "StorageFurniture",
    4: "TrashCan",
    5: "Microwave",
    6: "Dishwasher",
    7: "Display"
}

REVOLUTE_PART_ID = [

]
    
PRISMATIC_PART_ID = [

]

SCENE_GRAPH_FILE = "kino_graph.json"
ASSET_DIR = "assets"

GRAVITY_DIRECTION = [0, 0, -1]

#############################################
# dataset filename format
#############################################
BACKGROUND_NPY_FILE = "scen_without_objects_nature_color.npy"
RAW_OBJECT_FILENAME = "source_object_point.npy"
GT_OBJECT_FILENAME = "gt_partnet_in_sannet_xyz_normals_24id_307id_ourid.npy"
SEG_INPUT_FILENAME = "6144_gt_input.npy"
SEG_LABEL_FILENAME = "net_pred_id.npy"
RAW_SCENE_PLY = "soure_scen_point_clouds.ply"
COMPLETE_OBJECT_FILENAME = "net_complete_object.npy"