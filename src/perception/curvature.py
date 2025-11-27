import numpy as np
from perception.bounding_box import orient_cloud

def compute_rz_profile(point_cloud, num_slices=100):
    cloud = point_cloud.xyzs().T
    xy = cloud[:, :2] - np.mean(cloud[:, :2], axis=0)
    z = cloud[:, 2]
    r = np.linalg.norm(xy, axis=1)

    z_min, z_max = np.min(z), np.max(z)
    bins = np.linspace(z_min, z_max, num_slices+1)
    r_profile = []
    for i in range(num_slices):
        mask = (z >= bins[i]) & (z < bins[i+1])
        if np.sum(mask) > 0:
            r_profile.append(np.mean(r[mask]))
        else:
            r_profile.append(0.0)
    return np.array(r_profile)

def cloud_oriented_rz_similarity(scene_point_cloud, model_point_cloud,
                                scene_main_axis, model_main_axis,
                                scene_oriented=False, model_oriented=True):
    # Orient clouds if needed
    if not scene_oriented:
        z = scene_main_axis[2]
        if abs(z - 1) > 0.02:
            scene_point_cloud = orient_cloud(scene_point_cloud, scene_main_axis)
    if not model_oriented:
        z = model_main_axis[2]
        if abs(z - 1) > 0.02:
            model_point_cloud = orient_cloud(model_point_cloud, model_main_axis)

    # Compute rz profiles
    scene_rz = compute_rz_profile(scene_point_cloud)
    model_rz = compute_rz_profile(model_point_cloud)

    # Compute the similarity between the boxes vector distance
    dist = np.linalg.norm(scene_rz - model_rz)
    return dist