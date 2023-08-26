from sklearn.cluster import DBSCAN
import trimesh
import numpy as np


def get_instance_mask(points):
    clusters = DBSCAN(eps=0.1, min_samples=3).fit_predict(points)
    return clusters

    
def parse_seg_object_pointclouds(data):
    """Parse point npy cloud data to PartPointCloud object

    Args:
        data (np.ndarray (n_points, n_attr)):  segmented object parts where
            each row represents a 3D point with labels
                x, y, z, nx, ny, nz, obj_id, part_id_old, our_part_id

    Returns:
        list of PartPointCloud: a list of PartPointCloud
    """
    points = data[:, :3]
    part_labels = data[:, 8].astype(int)
    obj_id = data[0, 6]

    unique_part_ids = np.unique(part_labels)
    part_pcs = []

    for pid in unique_part_ids:
        part_points = points[part_labels == pid]
        # segment each instances of the part (e.g., a table has four legs)
        ins_mask = get_instance_mask(part_points)
        unique_ins_label = np.unique(ins_mask)
        for uid in unique_ins_label:
            part_pcs.append(
                PartPointCloud(part_points[ins_mask == uid], obj_id, pid)
            )

    return part_pcs


class PartPointCloud(object):

    def __init__(self, points, obj_id, part_id):
        self.points_ = points.copy()
        self.obj_id_ = obj_id
        self.part_id_ = part_id

        self.n_points_ = self.points_.shape[0]

    @property
    def obj_id(self):
        return self.obj_id_

    @property
    def points(self):
        return self.points_

    @property
    def part_id(self):
        return self.part_id_

    @property
    def n_points(self):
        return self.n_points_

    def get_obb_extents(self):
        try:
            to_origin_tf, extents = trimesh.bounds.oriented_bounds(self.points)
        except:
            print("PointCloud::get_obb_extents(): Failed to find obb extents")
            extents = None
            
        return extents
    
    def to_trimesh_pc(self, color=[0, 0, 0, 255]):
        return trimesh.PointCloud(self.points, colors=[color for _ in range(self.n_points)])