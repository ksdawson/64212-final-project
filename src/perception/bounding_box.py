import numpy as np
from pydrake.all import (
    PointCloud
)

######################################################################
# Functions to compute a bounding box around a point cloud
######################################################################

def orient_cloud(point_cloud, main_axis):
    # Ensure main axis normalized
    a = main_axis / np.linalg.norm(main_axis)

    # Build rotation that aligns main_axis to world z-axis
    z = np.array([0., 0., 1.])
    if np.allclose(a, z):
        R = np.eye(3)
    elif np.allclose(a, -z):
        # 180 degree rotation around x-axis works
        R = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    else:
        v = np.cross(a, z)
        s = np.linalg.norm(v)
        c = np.dot(a, z)
        vx = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ])
        R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s*s))

    # Rotate all points into bounding-box frame
    pts = point_cloud.xyzs().T
    pts_rot = pts @ R.T
    pts_rot = pts_rot.T

    # Create a new point cloud
    pc = PointCloud(pts_rot.shape[1], fields=point_cloud.fields())
    pc.mutable_xyzs()[:] = pts_rot
    if point_cloud.has_rgbs():
        pc.mutable_rgbs()[:] = point_cloud.rgbs()[:]
    return pc

def get_cloud_bounding_box(point_cloud, sort_box=True):
    pts = point_cloud.xyzs()
    x, y, z = pts[0], pts[1], pts[2]
    min_x, max_x = np.min(x), np.max(x)
    min_y, max_y = np.min(y), np.max(y)
    min_z, max_z = np.min(z), np.max(z)
    w, l, h = max_x - min_x, max_y - min_y, max_z - min_z
    box = np.array([w, l, h])
    if sort_box:
        box.sort() # accounts for that a cloud can be rotated
    return box

def get_cloud_oriented_bounding_box(point_cloud, main_axis):
    # Orient cloud if needed
    z = main_axis[2]
    if abs(z - 1) > 0.02:
        oriented_point_cloud = orient_cloud(point_cloud, main_axis)
    else:
        oriented_point_cloud = point_cloud

    # Compute bounding box
    return get_cloud_bounding_box(oriented_point_cloud, False) # don't need to account for rotated cloud now

######################################################################
# Functions to compute bounding box similarity between point clouds
######################################################################

def cloud_bounding_box_similarity(scene_point_cloud, model_point_cloud):
    # Get bounding boxes for scene and model point cloud
    scene_bb = get_cloud_bounding_box(scene_point_cloud)
    model_bb = get_cloud_bounding_box(model_point_cloud)

    # Compute the similarity between the boxes vector distance
    dist = np.linalg.norm(scene_bb - model_bb)
    return dist

def cloud_oriented_bounding_box_similarity(scene_point_cloud, model_point_cloud,
                                           scene_main_axis, model_main_axis,
                                           scene_oriented=False, model_oriented=True):
    # Get bounding boxes for scene and model point cloud
    if scene_oriented:
        scene_obb = get_cloud_bounding_box(scene_point_cloud)
    else:
        scene_obb = get_cloud_oriented_bounding_box(scene_point_cloud, scene_main_axis)
    if model_oriented:
        model_obb = get_cloud_bounding_box(model_point_cloud)
    else:
        model_obb = get_cloud_oriented_bounding_box(model_point_cloud, model_main_axis)

    # Compute the similarity between the boxes vector distance
    dist = np.linalg.norm(scene_obb - model_obb)
    return dist