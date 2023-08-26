import numpy as np
import trimesh

from transforms3d.affines import compose
from shapely.geometry import Polygon

from part2cad.geom.geom_transform import normalize_vec



def visualize_obb(mesh):
    obb_vertices = mesh.bounding_box_oriented.vertices
    size = 0.01

    vm = [
        trimesh.creation.icosphere(subdivisions=2, radius=size, color=[255, 0, 0, 255]),
        trimesh.creation.icosphere(subdivisions=2, radius=size, color=[0, 255, 0, 255]),
        trimesh.creation.icosphere(subdivisions=2, radius=size, color=[0, 0, 255, 255]),
        trimesh.creation.icosphere(subdivisions=2, radius=size, color=[255, 255, 255, 255]),
        trimesh.creation.box(extents=[size, size, size]),
        trimesh.creation.box(extents=[size, size, size]),
        trimesh.creation.box(extents=[size, size, size]),
        trimesh.creation.box(extents=[size, size, size])
    ]

    vm[4].visual.vertex_colors = [255, 0, 0, 255]
    vm[5].visual.vertex_colors = [0, 255, 0, 255]
    vm[6].visual.vertex_colors = [0, 0, 255, 255]
    vm[7].visual.vertex_colors = [255, 255, 255, 255]

    for i, v in enumerate(obb_vertices):
        vm[i].apply_transform(compose(v, np.eye(3), np.ones(3)))
    
    scene = trimesh.Scene(vm)
    scene.add_geometry(mesh)

    scene.show()


def calc_geom_attributes(mesh, tf=None):
    if tf is not None:
        mesh = mesh.copy()
        mesh.apply_transform(tf)
    
    obb = np.array(mesh.bounding_box_oriented.vertices)
    origin = np.average(obb, axis=0)

    axes = np.array([
        normalize_vec(obb[7] - obb[3]), 
        normalize_vec(obb[2] - obb[3]),
        normalize_vec(obb[1] - obb[3])
    ])

    # each axis direction has two planes, (origin, normal)
    planes = np.array([
        [[obb[3], axes[0]], [obb[7], axes[0]]],
        [[obb[3], axes[1]], [obb[2], axes[1]]],
        [[obb[3], axes[2]], [obb[1], axes[2]]]
    ])

    # each rect on the plane has four corners
    corners = np.array([
        [
            [obb[3], obb[2], obb[0], obb[1]],
            [obb[7], obb[6], obb[4], obb[5]]
        ],
        [
            [obb[3], obb[7], obb[5], obb[1]],
            [obb[2], obb[0], obb[4], obb[6]]
        ],
        [
            [obb[3], obb[2], obb[6], obb[7]],
            [obb[1], obb[0], obb[4], obb[5]]
        ]
    ])

    return origin, axes, planes, corners


def calc_dist_3d_rect(rect_a, plane_b):
    """Distance between two approximately paralleled rectangles

    Args:
        rect_a (list of 4 vec3): four corners of the rectangle B
        plane_b (list of 2 vec3): origin and unit normal of plane B
    """
    dist = np.average(np.abs(np.dot(rect_a - plane_b[0], plane_b[1])))
    return dist



def calc_area_3d_rect(rect):
    """Calculate the area of a rectangle in 3D

    Args:
        rect (list of 4 vec3): four corners (clockwise/counter-clockwise)
            of the rectangle
    """
    va = rect[1] - rect[0]
    vb = rect[3] - rect[0]
    
    # vector va and vb should be perpendicular
    assert(np.isclose(np.dot(va, vb), 0))

    return np.sqrt(np.sum(va ** 2)) * np.sqrt(np.sum(vb ** 2))



def calc_intersection_3d_rect(rect, polygon):
    ux = normalize_vec(rect[1] - rect[0])
    uy = normalize_vec(rect[3] - rect[0])

    # re-compute the coordinate in terms of ux, uy
    coord = lambda p: (np.dot(ux, p), np.dot(uy, p))
    u_rect = Polygon([coord(p) for p in rect])
    u_polygon = Polygon([coord(p) for p in polygon])

    return u_rect.intersection(u_polygon).area


def calc_contact_ratio_3d_rect(rect_a, plane_a, rect_b, plane_b):
    # project rect_a to plane_b
    dist = np.dot(rect_a - plane_b[0], plane_b[1])
    # projected rect_a on plane_b
    rect_ab = rect_a - np.array([d * plane_b[1] for d in dist])

    intersected_area = calc_intersection_3d_rect(rect_b, rect_ab)

    area_a = calc_area_3d_rect(rect_a)
    area_b = calc_area_3d_rect(rect_b)

    # a support b: intersected_area / area_b
    # b support a: intersected_area / area_a
    return intersected_area / area_b, intersected_area / area_a


def calc_mesh_iou(mesh_a, mesh_b):
    try:
        intersected_mesh = mesh_a.intersection(mesh_b)
        intersected_volume = intersected_mesh.volume
        # a_support_b, b_support_a
        return intersected_volume / mesh_b.volume, intersected_volume / mesh_a.volume
    except:
        return 0, 0


def find_near_obb_axis(ma, mb, angle_threshold=0.98):
    """Find nearly aligned (along the same line) axis of two mesh
       in terms of their oriented bounding box.

    Args:
        ma (trimesh.Trimesh): trimesh mesh object
        mb (trimesh.Trimesh): trimesh mesh object
        angle_threshold (float, optional): cos(theta) of the misaligned angle.
            Defaults to 0.99.

    Returns:
        (list of paired axis): a list of paired axis, each pair contains two
            element, (axis_a, axis_b)
    """
    _, axes_a, _, _ = calc_geom_attributes(ma)
    _, axes_b, _, _ = calc_geom_attributes(mb)

    matched_axis = []

    for axi in range(3):
        for bxj in range(3):
            # angle too large, thus cos(angle) too small
            if(abs(np.dot(axes_a[axi], axes_b[bxj])) < angle_threshold):
                continue

            # matched_axis.append( (axes_a[axi], axes_b[bxj]) )
            matched_axis.append( (axi, bxj) )
            break
    
    return axes_a, axes_b, np.array(matched_axis)


def calc_contact_score(ma, tfa, mb, tfb, angle_threshold=0.99, dist_threshold=0.03):
    """Calculate the contact heuristic score

    Args:
        ma (Trimesh): mesh A
        tfa (4x4 matrix): transformation matrix of mesh A
        mb (Trimesh): mesh B
        tfb (4x4 matrix): transformation matrix of mesh B
        threshold (float): distance threshold for computing the contact area
    """
    ma = ma.copy().apply_transform(tfa)
    mb = mb.copy().apply_transform(tfb)

    origin_a, axes_a, planes_a, corners_a = calc_geom_attributes(ma)
    origin_b, axes_b, planes_b, corners_b = calc_geom_attributes(mb)
    
    a2b, b2a = [], []
    for axi in range(3):
        for bxj in range(3):
            # angle too large, thus cos(angle) too small
            if(abs(np.dot(axes_a[axi], axes_b[bxj])) < angle_threshold):
                continue
            
            plane_dists = [
                (api, bpj, calc_dist_3d_rect(corners_a[axi][api], planes_b[bxj][bpj]))
                    for api in range(2) for bpj in range(2)
            ]
            plane_dists.sort(key=lambda x: x[2])
            
            # minimum distance too large, skip to next
            if plane_dists[0][2] > dist_threshold:
                continue
            
            api, bpj, _ = plane_dists[0]
            
            a_support_b, b_support_a = calc_contact_ratio_3d_rect(
                corners_a[axi][api], planes_a[axi][api],
                corners_b[bxj][bpj], planes_b[bxj][bpj]
            )

            a2b.append(a_support_b)
            b2a.append(b_support_a)

            break

    if len(a2b) != 0:
        return max(a2b), max(b2a)

    a_support_b, b_support_a = calc_mesh_iou(ma, mb)
    if not np.isclose(a_support_b, 0) or not np.isclose(b_support_a, 0):
        return a_support_b, b_support_a

    # use negative of distance as the score
    dist_score = -np.linalg.norm(origin_a - origin_b)

    return dist_score, dist_score


def get_obb_axis_extents(mesh, tf=None):
    print(mesh.extents)
    if tf is not None:
        mesh = mesh.copy()
        mesh.apply_transform(tf)
    
    obb = np.array(mesh.bounding_box_oriented.vertices)

    axes = np.array([
        normalize_vec(obb[7] - obb[3]), 
        normalize_vec(obb[2] - obb[3]),
        normalize_vec(obb[1] - obb[3])
    ])

    obb_proj = np.dot(obb, axes.T)
    extents = np.max(obb_proj, axis=0) - np.min(obb_proj, axis=0)

    return axes, extents