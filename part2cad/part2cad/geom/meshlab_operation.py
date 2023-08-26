import os
import pymeshlab


def export_pc_to_mesh_ply(trimesh_pcd, file_out):
    tmp_file_dir = "tmp.ply"

    # convert to absolute path
    trimesh_pcd.export(tmp_file_dir)

    abs_file_in = os.path.abspath(tmp_file_dir)
    
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(abs_file_in)

    print("[MeshLab] Processing ply to obj, waiting...")

    ms.compute_normal_for_point_clouds(
        k=10,
        smoothiter=0,
        # viewpos=[2, 2, 1.5],
        flipflag=False
    )

    ms.generate_surface_reconstruction_ball_pivoting()

    ms.meshing_invert_face_orientation()
    
    ms.save_current_mesh(file_out, binary=False)

    os.system("rm {}".format(tmp_file_dir))

    if os.path.exists(file_out):
        print("[INFO] Convert .ply to .obj at: `{}`".format(file_out))
    else:
        print("[ERROR] Fail to convert .ply to .obj: `{}`".format(file_out))
        raise