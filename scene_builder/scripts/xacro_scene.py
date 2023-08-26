import os
import copy
from shutil import ExecError

import numpy as np

from xacro_ros import XacroROS
from obj_type import ObjType
from utils import print_ok, print_warn, print_info, print_err
from transforms3d.euler import quat2euler, euler2mat
from transforms3d.quaternions import quat2mat
from transforms3d.affines import compose

from global_settings import MAIN_XACRO_FILENAME
from global_settings import RIGIT_OBJ_XACRO_FILENAME
from global_settings import RIGID_OBJ_XACRO_MACRO_NAME

from global_settings import XACRO_INC_SCENE_PREFIX
from global_settings import XACRO_PKG_SCENE_PREFIX

from global_settings import SCENE_RIGID_MESH_FOLDER
from global_settings import SCENE_BACKGROUND_MESH_FOLDER
from global_settings import SCENE_INTERACTIVE_MESH_FOLDER

from global_settings import DEFAULT_SCENE_ROOT

from global_settings import RIGID_DEFAULT_XYZ
from global_settings import RIGID_DEFAULT_RPY
from global_settings import RIGID_DEFAULT_JOINT_TYPE
from global_settings import RIGID_DEFAULT_MASS
from global_settings import RIGID_DEFAULT_INERTIA
from global_settings import RIGID_DEFAULT_SCALE

from global_settings import DYNAMICS_DEFAULT_FRICTION
from global_settings import DYNAMICS_DEFAULT_EFFORT
from global_settings import DYNAMICS_DEFAULT_VELOCITY
from global_settings import REVOLUTE_DEFAULT_LOWER
from global_settings import REVOLUTE_DEFAULT_UPPER
from global_settings import PRISMATIC_DEFAULT_LOWER
from global_settings import PRISMATIC_DEFAULT_UPPER


from global_settings import INTERACTIVE_CATEGORY
from global_settings import BACKGROUND_CATEGORY


def safe_quat2euler(quat, axis="sxyz"):
    quat = [0.0 if abs(q) < 1e-10 else q for q in quat]
    return quat2euler(quat, axis)


class XacroScene(object):
    """
    A simulation scene maintains 2 xacro files
        - main.xacro: will include and create 1 rigid_object.xacro 
            and n interactive_object.xacro
        - rigid_object.xacro: will specify all rigid-body objects in
            the xacro file
    """


    def __init__(self, scene_name, output_dir, pg, 
        enable_physics=False, enable_gazebo=False):
        """
        Constructor

        @param scene_name (string): the name of the scene
        @param output_name (string): the output directory w.r.t. the ROS scene_builder package
        @param pg (ParseGraph object): the parse graph of scene
        @param enable_physics (boolean): enable physical properties of every link
        @param enable_gazebo (boolean): enable gazebo output 
            (include some gazebo tag for generating the .sdf file)
        """
        self.scene_name_ = scene_name
        self.pg_ = pg
        
        self.enable_physics_ = enable_physics
        self.enable_gazebo_ = enable_gazebo

        # output directory of the scene w.r.t. ROS scene_builder package
        self.output_dst_ = os.path.normpath("{}/{}".format(os.path.normpath(output_dir), scene_name))

        # store background mesh filenames
        self.bgm_files_ = []
        # store rigid mesh filenames
        self.rigid_files_ = []
        # store interactive mesh filenames
        self.interactive_files_ = []
        
        # store the link to mesh file mapping
        # <link-name>: {
        #   "mesh_dir": <mesh_dir>,
        #   "scale": <scale>
        # }
        self.link_to_mesh_ = {}

        # store the parent-child link relations
        self.parent_child_pairs_ = []

        self.main_xacro_ = self.create_main_xacro_(scene_name)
        self.rigid_obj_xacro_ = self.create_rigid_obj_xacro_()

        # include rigid object xacro in main
        self.main_xacro_.add_include("{}/{}/{}".format(XACRO_INC_SCENE_PREFIX, self.output_dst_, RIGIT_OBJ_XACRO_FILENAME))
        self.main_xacro_.instantiate_macro(RIGID_OBJ_XACRO_MACRO_NAME)

        if self.enable_gazebo_:
            self.add_gazebo_world_joint_()

    
    def __str__(self):
        ret = ""

        ret += "* [INFO] Filename: main.xacro\n"
        ret += self.main_xacro_.__str__() + "\n"

        ret += "* [INFO] Filename: rigid_objects.xacro\n"
        ret += self.rigid_obj_xacro_.__str__() + "\n"

        return ret


    def save(self, scene_builder_root):
        """
        Save current scene into organized xacro files

        @param: scene_builder_root (string): the directory of the scene_builder package

        @return (string): return the output destination if xacro is 
            successfully saved, otherwise return an exampty string ""
        """
        # check if the output directory already exists
        output_dst = os.path.normpath("{}/{}".format(scene_builder_root, self.output_dst_))

        if os.path.isdir(output_dst):
            print_warn("Output directory `{}` is already exist.".format(output_dst))
            ret = input("Replace it? [y/n]")
            if ret in ["y", "Y"]:
                os.system("rm -r {}".format(output_dst))
            else:
                print_info("Scene builder exit")
                return ""

        # create the output directory
        try:
            os.system("mkdir {}".format(output_dst))
            os.system("mkdir -p {}/{}".format(output_dst, SCENE_BACKGROUND_MESH_FOLDER))
            os.system("mkdir -p {}/{}".format(output_dst, SCENE_RIGID_MESH_FOLDER))
            os.system("mkdir -p {}/{}".format(output_dst, SCENE_INTERACTIVE_MESH_FOLDER))
        except:
            print_err("[ERROR] `{}` does not exist".format(output_dst))
            return ""
        
        self.main_xacro_.save("{}/{}".format(output_dst, MAIN_XACRO_FILENAME))
        self.rigid_obj_xacro_.save("{}/{}".format(output_dst, RIGIT_OBJ_XACRO_FILENAME))

        print_info("[INFO] xacro scene is save at: {}".format(output_dst))
        return output_dst


    def get_bgm_files(self):
        """
        Get background mesh filenames

        @return (list of string): the list of destination directories of 
            the background mesh files 
        """
        return self.bgm_files_[:]


    def get_rigid_files(self):
        """
        Get rigid mesh filenames

        @return (list of string): the list of destination directories of 
            the rigid mesh files
        """
        return self.rigid_files_[:]


    def get_interactive_files(self):
        """
        Get interactive mesh filenames

        @return (list of string): the list of destination directories of 
            the interactive mesh files
        """
        return self.interactive_files_[:]


    def get_link_mesh_mapping(self):
        """
        Get the link to mesh mapping

        @return (dictionary): where the key is the link name where the
            value is the mesh directory
        """
        return copy.deepcopy(self.link_to_mesh_)


    def get_parent_child_pairs(self):
        """
        Get the parent-child link relations in the scene

        @return (a list tuples): (parent_link_name, child_link_name)
        """
        return self.parent_child_pairs_[:]


    def add(self, target, parent=None):
        """
        Add a new node into current xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        if target["type"] == "ConceptNode":
            self.add_concept_link_(target, parent)
        else:
            if target["joint_type"] == "fixed":
                if target["label"] == "Background":
                    self.add_background_object_(target, parent)
                else:
                    self.add_rigid_object_(target, parent)
            elif target["joint_type"] == "revolute":
                self.add_revolute_object_(target, parent)
            elif target["joint_type"] == "prismatic":
                self.add_prismatic_object_(target, parent)
            else:
                raise Exception("[XacroScene::add] Does not support type `{}`".format(target["joint_type"]))


    def add_background_object_(self, target, parent):
        """
        Add a background object into the xacro scene

        Background object only have visual effect, did not have any 
        collision detection or interaction capability.

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        mesh_dir = self.get_xacro_bg_dir_(target)
        scale = 1.0 if target["scale"] is None else target["scale"]

        mass = RIGID_DEFAULT_MASS if self.enable_physics_ or self.enable_gazebo_ else None
        inertia = RIGID_DEFAULT_INERTIA if self.enable_physics_ or self.enable_gazebo_ else None
        enable_collision = self.enable_physics_ or self.enable_gazebo_
        
        # Add object link into the xacro
        # the <xyz> and <rpy> specify the origin of the mesh
        self.rigid_obj_xacro_.add_background(
            link_name=link_name,
            mesh_dir=mesh_dir,
            xyz=RIGID_DEFAULT_XYZ,
            rpy=RIGID_DEFAULT_RPY,
            scale=[scale] * 3,
            mass=mass,
            inertia=inertia,
            add_collision=enable_collision,
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )

        # noted: target["position"] and target["orientation"] defines
        # the transform between target and its parent
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type=target["joint_type"],
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                xyz=target["position"],
                rpy=safe_quat2euler(target["orientation"], "sxyz"),
                add_gazebo_tag=self.enable_gazebo_,
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )
        
        # record background mesh filename
        self.bgm_files_.append(self.get_scene_bg_dir_(target))

    
    def add_rigid_object_(self, target, parent):
        """
        Add a rigid object into the xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        mesh_dir = self.get_xacro_rigid_dir_(target)

        if mesh_dir is None:
            mesh_dir=self.get_xacro_bg_dir_(target)

            ply_file = mesh_dir.split('/')[-1]
            print_warn("[WARNING] Use `{}` for rigid object `{}_{}` instead".format(
                ply_file, target["label"], target["id"]))

            # record background mesh filename
            self.bgm_files_.append(self.get_scene_bg_dir_(target))
            self.record_link2mesh_(target, ObjType.Background)
            self.record_parent_link_relation_(target, parent, ObjType.Background)
        else:
            # record rigid mesh filename
            self.rigid_files_.append(self.get_scene_rigid_dir_(target))
            self.record_link2mesh_(target, ObjType.Rigid)
            self.record_parent_link_relation_(target, parent, ObjType.Rigid)

        mass = RIGID_DEFAULT_MASS if self.enable_physics_ or self.enable_gazebo_ else None
        inertia = RIGID_DEFAULT_INERTIA if self.enable_physics_ or self.enable_gazebo_ else None
        enable_collision = self.enable_physics_ or self.enable_gazebo_

        # Add object link into the xacro
        # the <xyz> and <rpy> specify the origin of the mesh
        self.rigid_obj_xacro_.add_object(
            link_name=link_name,
            mesh_dir=mesh_dir,
            xyz=RIGID_DEFAULT_XYZ,
            rpy=RIGID_DEFAULT_RPY,
            scale=[target["scale"], target["scale"], target["scale"]],
            mass=mass,
            inertia=inertia,
            color=target["color"],
            add_collision=enable_collision,
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )

        # noted: target["position"] and target["orientation"] defines
        # the transform between target and its parent
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type="fixed",
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                # xyz=xyz,
                # rpy=rpy,
                xyz=target["position"],
                rpy=safe_quat2euler(target["orientation"], "sxyz"),
                add_gazebo_tag=self.enable_gazebo_,
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )


    def add_revolute_object_(self, target, parent):
        """
        Add a revolute object into the xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        mesh_dir = self.get_xacro_rigid_dir_(target)

        if mesh_dir is None:
            mesh_dir=self.get_xacro_bg_dir_(target)

            ply_file = mesh_dir.split('/')[-1]
            print_warn("[WARNING] Use `{}` for rigid object `{}_{}` instead".format(
                ply_file, target["label"], target["id"]))

            # record background mesh filename
            self.bgm_files_.append(self.get_scene_bg_dir_(target))
            self.record_link2mesh_(target, ObjType.Background)
            self.record_parent_link_relation_(target, parent, ObjType.Background)
        else:
            # record rigid mesh filename
            self.rigid_files_.append(self.get_scene_rigid_dir_(target))
            self.record_link2mesh_(target, ObjType.Rigid)
            self.record_parent_link_relation_(target, parent, ObjType.Rigid)

        mass = RIGID_DEFAULT_MASS if self.enable_physics_ or self.enable_gazebo_ else None
        inertia = RIGID_DEFAULT_INERTIA if self.enable_physics_ or self.enable_gazebo_ else None
        enable_collision = self.enable_physics_ or self.enable_gazebo_

        # Add object link into the xacro
        # the <xyz> and <rpy> specify the origin of the mesh
        self.rigid_obj_xacro_.add_object(
            link_name=link_name,
            mesh_dir=mesh_dir,
            xyz=target["link_offset_position"],
            rpy=safe_quat2euler(target["link_offset_orientation"], "sxyz"),
            scale=[target["scale"], target["scale"], target["scale"]],
            mass=mass,
            inertia=inertia,
            color=target["color"],
            add_collision=enable_collision,
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )

        # noted: target["position"] and target["orientation"] defines
        # the transform between target and its parent
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type="revolute",
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                xyz=target["position"],
                rpy=safe_quat2euler(target["orientation"], "sxyz"),
                joint_axis=target["joint_axis"],
                joint_friction=DYNAMICS_DEFAULT_FRICTION,
                joint_effort=DYNAMICS_DEFAULT_EFFORT,
                joint_velocity=DYNAMICS_DEFAULT_VELOCITY,
                joint_lower=REVOLUTE_DEFAULT_LOWER,
                joint_upper=REVOLUTE_DEFAULT_UPPER,
                add_gazebo_tag=self.enable_gazebo_,
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )

    def add_prismatic_object_(self, target, parent):
        """
        Add a prismatic object into the xacro scene

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        mesh_dir = self.get_xacro_rigid_dir_(target)

        if mesh_dir is None:
            mesh_dir=self.get_xacro_bg_dir_(target)

            ply_file = mesh_dir.split('/')[-1]
            print_warn("[WARNING] Use `{}` for rigid object `{}_{}` instead".format(
                ply_file, target["label"], target["id"]))

            # record background mesh filename
            self.bgm_files_.append(self.get_scene_bg_dir_(target))
            self.record_link2mesh_(target, ObjType.Background)
            self.record_parent_link_relation_(target, parent, ObjType.Background)
        else:
            # record rigid mesh filename
            self.rigid_files_.append(self.get_scene_rigid_dir_(target))
            self.record_link2mesh_(target, ObjType.Rigid)
            self.record_parent_link_relation_(target, parent, ObjType.Rigid)

        mass = RIGID_DEFAULT_MASS if self.enable_physics_ or self.enable_gazebo_ else None
        inertia = RIGID_DEFAULT_INERTIA if self.enable_physics_ or self.enable_gazebo_ else None
        enable_collision = self.enable_physics_ or self.enable_gazebo_

        # Add object link into the xacro
        # the <xyz> and <rpy> specify the origin of the mesh
        self.rigid_obj_xacro_.add_object(
            link_name=link_name,
            mesh_dir=mesh_dir,
            xyz=target["link_offset_position"],
            rpy=safe_quat2euler(target["link_offset_orientation"], "sxyz"),
            scale=[target["scale"], target["scale"], target["scale"]],
            mass=mass,
            inertia=inertia,
            color=target["color"],
            add_collision=enable_collision,
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )

        # noted: target["position"] and target["orientation"] defines
        # the transform between target and its parent
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type="prismatic",
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                xyz=target["position"],
                rpy=safe_quat2euler(target["orientation"], "sxyz"),
                joint_axis=target["joint_axis"],
                joint_friction=DYNAMICS_DEFAULT_FRICTION,
                joint_effort=DYNAMICS_DEFAULT_EFFORT,
                joint_velocity=DYNAMICS_DEFAULT_VELOCITY,
                joint_lower=PRISMATIC_DEFAULT_LOWER,
                joint_upper=PRISMATIC_DEFAULT_UPPER,
                add_gazebo_tag=self.enable_gazebo_,
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )

    def add_concept_link_(self, target, parent):
        """
        Add a concept node into the xacro scene

        A concept node does not have an actual object associated with, which
        only contains a link frame in the xacro scene.

        @param target (igraph.Vertex): target node to be added
        @param parent (igraph.Vertex): parent node (if exists) of the target node
        """
        link_name = self.get_link_name_(target)
        
        # add concept node link into xacro file
        self.rigid_obj_xacro_.add_link(link_name, self.enable_gazebo_, RIGID_OBJ_XACRO_MACRO_NAME)

        # add a joint (transform) between parent and child
        if parent is not None:
            self.rigid_obj_xacro_.add_joint(
                joint_name=self.get_joint_name_(target, parent),
                joint_type=RIGID_DEFAULT_JOINT_TYPE,
                child_link=link_name,
                parent_link=self.get_link_name_(parent),
                xyz=target["position"],
                rpy=safe_quat2euler(target["orientation"], "sxyz"),
                add_gazebo_tag=self.enable_gazebo_,
                ns=RIGID_OBJ_XACRO_MACRO_NAME
            )
        
        # record link to mesh mapping
        self.record_link2mesh_(target, ObjType.ConceptNode)
        
        self.record_parent_link_relation_(target, parent, ObjType.ConceptNode)
        

    def create_main_xacro_(self, scene_name="simulation_scene"):
        """
        Create a xacro object for the main file

        @return (XacroROS) the xacro object of the main file
        """
        urdf_xacro = XacroROS(scene_name)
        
        return urdf_xacro


    def create_rigid_obj_xacro_(self):
        """
        Create a xacro object for rigid-body objects

        @return (XacroROS) the xacro object of the rigid objects
        """
        urdf_xacro = XacroROS(RIGID_OBJ_XACRO_MACRO_NAME)
        urdf_xacro.add_macro(RIGID_OBJ_XACRO_MACRO_NAME)

        return urdf_xacro

    
    def get_link_name_(self, node):
        """
        Generate link name for the given node

        Noted that link name should be unique in a xacro file

        @param node: (igraph.Vertex) node object
        @return (string) the link name
        """
        link_name = ""

        if node["type"] == "ConceptNode":
            link_name = "{}_{}_link".format(node["label"], node["id"])
        elif node["type"] == "ObjectNode":
            link_name = "{}_{}_link".format(node["label"], node["id"])
        else:
            raise Exception("[ERROR] Does not support node type `{}`".format(node["type"]))

        return link_name


    def get_joint_name_(self, target, parent):
        """
        Get the joint name of the joint

        @param target: (igraph.Vertex) child node
        @param parent: (igraph.Vertex) parent node
        @return (string) the joint name
        """
        assert(parent is not None)

        joint_name = "{}_{}_{}_{}_joint".format(
            parent["label"], parent["id"], target["label"], target["id"]
        )

        return joint_name


    def get_xacro_bg_dir_(self, node):
        """
        Get the XACRO directory of the background pointcloud of the node

        The directory format should followed the ROS convention, e.g,
            package://tooluse_viz/asset/mesh/hammer_t9.stl
        
        @param node (igraph.Vertex) node object that represents background
        @return (string) the directory of the background pointcloud
        """
        bg_mesh_dir = "{}/{}/{}/{}.obj".format(
            XACRO_PKG_SCENE_PREFIX, self.output_dst_, SCENE_BACKGROUND_MESH_FOLDER, node["id"]
        )
        return bg_mesh_dir


    def get_scene_bg_dir_(self, node):
        """
        Get the directory of the background pointcloud in the generated scene folder

        @param node (igraph.Vertex) node object that represents background
        @return (string) the directory of the background pointcloud
        """
        bg_dir = "{}/{}/{}.obj".format(
            self.output_dst_, SCENE_BACKGROUND_MESH_FOLDER, node["id"]
        )
        return bg_dir


    def get_xacro_rigid_dir_(self, node):
        """
        Get the XACRO directory of the 3D CAD model of the node

        The directory format should followed the ROS convention, e.g,
            package://tooluse_viz/asset/mesh/hammer_t9.stl
        
        @param node (igraph.Vertex) node object that represents rigid body
        @return (string) the directory of the CAD model
        """
        cad_id = node["cad_id"]

        if cad_id == "" or cad_id is None:
            print_warn("[WARNING] `cad_id` for `{}_{}` is empty".format(node["label"], node["id"]))
            return None
        
        mesh_dir = "{}/{}/{}/{}.stl".format(
            XACRO_PKG_SCENE_PREFIX, self.output_dst_, SCENE_RIGID_MESH_FOLDER, cad_id
        )

        return mesh_dir


    def get_scene_rigid_dir_(self, node):
        """
        Get the directory of the rigid mesh in the generated scene folder

        @param node (igraph.Vertex) node object that represents background
        @return (string) the directory of the background pointcloud
        """
        rigid_dir = "{}/{}/{}.stl".format(
            self.output_dst_, SCENE_RIGID_MESH_FOLDER, node["cad_id"]
        )
        return rigid_dir

    
    def get_xacro_interactive_dir_(self, node):
        """
        Get the XACRO directory of the interactive object of the node

        The interactive object is stored in a xacro file
        
        @param node (igraph.Vertex) node object
        @return (string) the directory of the interactive object xacro file
        """
        cad_id = node["cad_id"]

        xacro_dir = "{}/{}/{}/{}/{}.xacro".format(
            XACRO_INC_SCENE_PREFIX, self.output_dst_, SCENE_INTERACTIVE_MESH_FOLDER, cad_id, cad_id
        )

        return xacro_dir

    
    def get_scene_interactive_dir_(self, node):
        """
        Get the directory of the rigid mesh in the generated scene folder

        @param node (igraph.Vertex) node object that represents background
        @return (string) the directory of the background pointcloud
        """
        interactive_dir = "{}/{}/{}".format(
            self.output_dst_, SCENE_INTERACTIVE_MESH_FOLDER, node["cad_id"]
        )
        return interactive_dir


    def record_parent_link_relation_(self, target, parent, obj_type):
        """
        Record the parent-child relations in the scene

        Relation are recorded follows (parent_link_name, child_link_name) format

        @param target: (igraph.Vertex) child node
        @param parent: (igraph.Vertex) parent node
        @param obj_type (ObjType): type of the object
        """
        if obj_type in [ObjType.Rigid, ObjType.Background, ObjType.ConceptNode]:
            parent_link_name = "" if (parent is None) else self.get_link_name_(parent)
            child_link_name = self.get_link_name_(target)
            self.parent_child_pairs_.append( (parent_link_name, child_link_name) )
        else:
            print_err("[ERROR] Unknown object type: `{}`".format(obj_type))
            raise


    def record_link2mesh_(self, node, obj_type):
        """
        Record the link name to mesh name mapping

        When a new object is added to the scene, record the mapping between
        link name and mesh name

        @param node (igraph.Vertex): new added object node
        @param obj_type (ObjType): type of the object
        """
        link_name = self.get_link_name_(node)

        if link_name in self.link_to_mesh_:
            print_warn("[WARNING] Link `{}` already exists in the link_to_mesh mapping, overwrite is performed.".format(link_name))
        
        if obj_type != ObjType.ConceptNode:
            scale = 1.0 if node["scale"] is None else node["scale"]

        if obj_type == ObjType.Background:
            self.link_to_mesh_[link_name] = {}
            self.link_to_mesh_[link_name]["type"] = obj_type
            self.link_to_mesh_[link_name]["mesh_dir"] = self.get_scene_bg_dir_(node)
            self.link_to_mesh_[link_name]["scale"] = scale
        elif obj_type == ObjType.Rigid:
            self.link_to_mesh_[link_name] = {}
            self.link_to_mesh_[link_name]["type"] = obj_type
            self.link_to_mesh_[link_name]["mesh_dir"] = self.get_scene_rigid_dir_(node)
            self.link_to_mesh_[link_name]["scale"] = scale
        elif obj_type == ObjType.ConceptNode:
            self.link_to_mesh_[link_name] = {}
            self.link_to_mesh_[link_name]["type"] = obj_type
            self.link_to_mesh_[link_name]["mesh_dir"] = None
            self.link_to_mesh_[link_name]["scale"] = None
        else:
            print_err("[ERROR] Unknown object type: `{}`".format(obj_type))
            raise
        

    def align_transform_(self, node):
        """
        Align the transformation between rigid interactive object and the 
        original interactive object.
        
        Note that, the origin and orientation of rigid interactive object
        could be different from the original one.
        
        @param node (igraph.Vertex) node object that represents object
        @return (string) the directory of the CAD model
        """
        scale = node["scale"]

        rpy = safe_quat2euler(node["orientation"])
        xyz = node["position"]

        return xyz, rpy, scale


    def add_gazebo_world_joint_(self):
        """
        Add a gazebo default joint between link `world` and DEFAULT_SCENE_ROOT

        In Gazebo simulation, they require a fixed link named `world`, then we need to
        add a dummy fixed joint between the `world` link and the DEFAULT_SCENE_ROOT

        The DEFAULT_SCENE_ROOT is given by the SLAM front-end.
        """
        self.rigid_obj_xacro_.add_link("world", False, RIGID_OBJ_XACRO_MACRO_NAME)

        # add the dummy joint
        # here we set rpy to [1.57, 0, 0] since by default, the gravity direction
        # of the output scene is alone -y direction, we need a rotation to make the
        # gravity direction to -z.
        self.rigid_obj_xacro_.add_joint(
            joint_name="world_fixed_joint",
            joint_type="fixed",
            child_link=DEFAULT_SCENE_ROOT,
            parent_link="world",
            xyz=[0, 0, 0],
            rpy=[1.57, 0, 0],
            add_gazebo_tag=True,
            ns=RIGID_OBJ_XACRO_MACRO_NAME
        )



