import numpy as np

def get_cloud_bounding_box(point_cloud):
    pts = point_cloud.xyzs()
    x, y, z = pts[0], pts[1], pts[2]
    min_x, max_x = np.min(x), np.max(x)
    min_y, max_y = np.min(y), np.max(y)
    min_z, max_z = np.min(z), np.max(z)
    w, l, h = max_x - min_x, max_y - min_y, max_z - min_z
    box = np.array([w, l, h])
    box.sort() # accounts for that a cloud can be rotated
    return box

def cloud_bounding_box_similarity(scene_point_cloud, model_point_cloud):
    # Get bounding boxes for scene and model point cloud
    scene_bb = get_cloud_bounding_box(scene_point_cloud)
    model_bb = get_cloud_bounding_box(model_point_cloud)

    # Compute the similarity between the boxes vector distance
    dist = np.linalg.norm(scene_bb - model_bb)
    return dist

def cloud_centroid_similarity(scene_point_cloud, model_point_cloud):
    # Scene centroid
    scene_pts = scene_point_cloud.xyzs()
    scene_centroid = np.mean(scene_pts, axis=1)
    # Model centroid
    scene_pts = scene_point_cloud.xyzs()
    scene_centroid = np.mean(scene_pts, axis=1)