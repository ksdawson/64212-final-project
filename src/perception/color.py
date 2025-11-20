import numpy as np

######################################################################
# Color classification of point clouds
######################################################################

def classify_by_color(point_clouds, color_map):
    # Classify point clouds by color
    classifications = []
    for cloud in point_clouds:
        # Average the color of the point cloud to get the object color
        colors = cloud.rgbs()
        rgb = np.mean(colors, axis=1)

        # Classify it by determining which color its closest to using vector distance
        min_dist, color_name = np.inf, None
        for name, color_vector in color_map:
            if rgb.ndim == 1:
                dist = np.linalg.norm(rgb - color_vector)
            else:
                dist = np.linalg.norm(rgb - color_vector, axis=1)
            if dist <= min_dist:
                min_dist = dist
                color_name = name
        classifications.append(color_name)
    return classifications