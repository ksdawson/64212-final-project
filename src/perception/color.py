import numpy as np
from scipy.optimize import linear_sum_assignment

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

def classify_by_color_constrained(point_clouds, color_map):
    # Build cost matrix (scene_count x model_count)
    cost = np.zeros((len(point_clouds), len(color_map)))

    # Partially fill in the cost matrix using the unique scene, color pairs
    for i, cloud in enumerate(point_clouds):
        # Average the color of the point cloud to get the object color
        colors = cloud.rgbs()
        rgb = np.mean(colors, axis=1)

        # Calculate cost using vector distance
        for j, (name, color_vector) in enumerate(color_map):
            if rgb.ndim == 1:
                dist = np.linalg.norm(rgb - color_vector)
            else:
                dist = np.linalg.norm(rgb - color_vector, axis=1)
            cost[i, j] = dist
    
    # Expand cost matrix
    expanded_columns = []
    count = len(point_clouds) // len(color_map) # split equally
    for j, _ in enumerate(color_map):
        for _ in range(count):
            expanded_columns.append(cost[:, j])
    cost = np.column_stack(expanded_columns)

    # Hungarian algorithm for optimal assignment
    row_ind, col_ind = linear_sum_assignment(cost)

    # Build mapping of piece-instance to pose
    mapping = [None] * len(point_clouds)
    for row, col in zip(row_ind, col_ind):
        k = col // count
        mapping[row] = color_map[k][0]

    return mapping