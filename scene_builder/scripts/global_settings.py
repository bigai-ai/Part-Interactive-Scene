import os
from utils import load_json
from utils import print_err, print_warn


#################################################################
# Deprecated: replace meshlabserver with pymeshlab
#################################################################
# # get MESH_LAB_SERVER executable
# if os.system("type meshlab.meshlabserver") == 0:
#     # meshlab is installed via snap
#     MESH_LAB_SERVER = "meshlab.meshlabserver"
# elif os.system("type meshlabserver") == 0:
#     # meshlab is installed via apt
#     MESH_LAB_SERVER = "meshlabserver"
# else:
#     print_warn("[WARN] Fail to find meshlabserver, use PyMeshLab instead")
#     MESH_LAB_SERVER = "pymeshlab"
    
# MESHLAB_COLOR_TO_TEXTURE_TEMPLATE = "assets/meshlab_scripts/vertex_color_to_texture.mlx"
# TMP_MESHLAB_SCRIPT = ".tmp_meshlab_filter_script.mlx"
# MESHLAB_EXE_TEMPLATE_SCRIPT = MESH_LAB_SERVER + " -i {} -o {} -m {} -s {}"
# MESHLAB_EXE_TEMPLATE = MESH_LAB_SERVER + " -i {} -o {} -m vc vn fc wt"


VERTEX_TO_FACE_TEXTURE_DIM = 4096
MESHLAB_TEXTURE_FILE_TEMPLATE = "{}_texture.png"


SCENE_PG_FILENAME = "kino_graph.json"
SCENE_SEGMENTS_DIR = "assets"

MAIN_XACRO_FILENAME = "main.xacro"
RIGIT_OBJ_XACRO_FILENAME = "rigid_objects.xacro"
LINK_TO_MESH_FILENAME = "link_to_mesh.csv"
PARENT_CHILD_PAIR_FILENAME = "link_parent_child_pair.csv"

DEFAULT_SCENE_ROOT = "Room_0_link"
RIGID_OBJ_XACRO_MACRO_NAME = "create_rigid_objects"

XACRO_INC_SCENE_PREFIX = "$(find scene_builder)"
XACRO_PKG_SCENE_PREFIX = "package://scene_builder"

RIGID_DEFAULT_XYZ = [0, 0, 0]
RIGID_DEFAULT_RPY = [0, 0, 0]
RIGID_DEFAULT_SCALE = [1, 1, 1]
RIGID_DEFAULT_MASS = 5.0
RIGID_DEFAULT_INERTIA = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0]
RIGID_DEFAULT_JOINT_TYPE = "fixed"
DYNAMICS_DEFAULT_FRICTION = 0.65
DYNAMICS_DEFAULT_EFFORT = 1.0
DYNAMICS_DEFAULT_VELOCITY = 0.5
REVOLUTE_DEFAULT_LOWER = 0.0
REVOLUTE_DEFAULT_UPPER = 1.57
PRISMATIC_DEFAULT_LOWER = 0
PRISMATIC_DEFAULT_UPPER = 0.4


SCENE_BUILDER_OUTPUT_DIR = "output"

RIGID_MESH_FOLDER = "rigid"
INTERACTIVE_MESH_FOLDER = "interactive"
BACKGROUND_MESH_FOLDER = "background"

SCENE_RIGID_MESH_FOLDER = "assets/{}".format(RIGID_MESH_FOLDER)
SCENE_INTERACTIVE_MESH_FOLDER = "assets/{}".format(INTERACTIVE_MESH_FOLDER)
SCENE_BACKGROUND_MESH_FOLDER = "assets/{}".format(BACKGROUND_MESH_FOLDER)

DATABASE_RIGID_MESH_DIR = "assets/{}".format("rigid_open3d")
DATABASE_INTERACTIVE_MESH_DIR = "assets/{}".format(INTERACTIVE_MESH_FOLDER)
DATABASE_INTERACTIVE_DEFAULT_TF_FILE = "assets/{}/interactive_transform.json".format(INTERACTIVE_MESH_FOLDER)

VRGYM_SCALED_MESH_DIR_ROOT = "assets/scaled"


INTERACTIVE_CATEGORY = [
    "Cabinet",
    "Fridge",
    # "Microwave",
    "Drawer",
    "Door",
    "Refrigerator"
]

BACKGROUND_CATEGORY = [
    "Ceiling",
    "Floor",
    "Wall",
    "Background"
]