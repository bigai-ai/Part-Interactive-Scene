import numpy as np
import trimesh
import random

from part2cad.visualization import create_palette


class ScenePointCloud(object):

    def __init__(self, bg_points, obj_points_list):
        if bg_points is not None:
            self.bg_pcd_ = self.to_color_pcd_(bg_points)
        else:
            self.bg_pcd_ = None
        self.obj_pcds_ = [self.to_pcd_(points) for points in obj_points_list]

        if obj_points_list[0].shape[1] == 9:
            self.obj_partid_list_ = [
                points[:, 8].astype("uint8") for points in obj_points_list
            ]
        else:
            print("No part id is found in the point cloud")
            self.obj_partid_list_ = None


    def show(self, viz_part=True, angles=None, distance=None):
        scene_pcd = self.colored_parts() if viz_part else self.colored_objects()

        scene = trimesh.Scene([scene_pcd])
        scene.set_camera(angles=angles, distance=distance)
        scene.show()


    def save(self, output_dir, viz_part=True):
        scene = self.colored_parts() if viz_part else self.colored_objects()
        scene.export(output_dir)


    def get_object_pcds(self):
        return [pcd.copy() for pcd in self.obj_pcds_]


    def colored_objects(self):
        palette = create_palette([i for i in range(len(self.obj_pcds_))])

        for i, pcd in enumerate(self.obj_pcds_):
            c = palette[i]
            pcd.colors = [c for _ in range(pcd.shape[0])]

        if self.bg_pcd_ is not None:
            points = np.vstack([self.bg_pcd_.vertices] + [pcd.vertices for pcd in self.obj_pcds_])
            colors = np.vstack([self.bg_pcd_.colors] + [pcd.colors for pcd in self.obj_pcds_])
        else:
            points = np.vstack([pcd.vertices for pcd in self.obj_pcds_])
            colors = np.vstack([pcd.colors for pcd in self.obj_pcds_])

        return trimesh.PointCloud(points, colors=colors)


    def colored_parts(self):
        if self.obj_partid_list_ is None:
            raise Exception("This scene does not support part-based visualization")
        
        offset = 0
        all_part_idx = []

        for part_ids in self.obj_partid_list_:
            tmp_idx = dict()
            part_idx = []
            for pid in part_ids:
                if pid not in tmp_idx:
                    tmp_idx[pid] = offset
                    offset += 1
                part_idx.append(tmp_idx[pid])
            all_part_idx.append(part_idx)
        
        unique_idx = [i for i in range(offset)]
        random.seed(1)
        random.shuffle(unique_idx)
        palette = create_palette(unique_idx)

        for i, pcd in enumerate(self.obj_pcds_):
            pcd.colors = [palette[pid] for pid in all_part_idx[i]]

        # removed_part = [2, 9, 21]
        removed_part = []
        
        points = np.vstack([pcd.vertices for pcd in self.obj_pcds_])
        colors = np.vstack([pcd.colors for pcd in self.obj_pcds_])

        all_part_idx = np.array([i for part_idx in all_part_idx for i in part_idx])

        for i in removed_part:
            points = points[all_part_idx != i]
            colors = colors[all_part_idx != i]
            all_part_idx = all_part_idx[all_part_idx != i]

        if self.bg_pcd_ is not None:
            points = np.vstack([self.bg_pcd_.vertices, points])
            colors = np.vstack([self.bg_pcd_.colors, colors])
        else:
            points = np.vstack([points])
            colors = np.vstack([colors])

        return trimesh.PointCloud(points, colors=colors)
        

    def to_pcd_(self, points):
        """To Trimesh.PointCloud

        Args:
            points ((n_points, 3) np.ndarray): a list of points, where
                each point is represented by <x, y, z> 
        """
        return trimesh.PointCloud(points[:, :3])


    def to_color_pcd_(self, points):
        """To colored Trimesh.PointCloud

        Args:
            points ((n_points, 6) np.ndarray): a list of points, where each point
                is represented by <x, y, z, r, g, b>
        """
        colors_rgba = np.ones((len(points), 4), dtype="uint8") * 255
        colors_rgba[:, 0:3] = (points[:, 3:6] * 255).astype("uint8")

        pcd = trimesh.PointCloud(points[:, :3], colors=colors_rgba)
        return pcd