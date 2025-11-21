import numpy as np
from sklearn.cluster import DBSCAN

def get_cloud_main_axis_pca(point_cloud):
    pts = point_cloud.xyzs().T # Nx3

    # Center the cloud
    pts_centered = pts - np.mean(pts, axis=0)

    # Covariance matrix
    cov = np.cov(pts_centered, rowvar=False)

    # PCA: eigen decomposition
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # The eigenvector with largest eigenvalue is the main axis (tallest direction)
    idx = np.argmax(eigenvalues)
    main_axis = eigenvectors[:, idx]

    return main_axis


def get_piece_cloud_main_axis(piece_cloud):
    # Get main axis
    main_axis = get_cloud_main_axis_pca(piece_cloud)

    # Calculate distance between main axis and x,y,z axes
    pos_main_axis = np.abs(main_axis)
    x_axis, y_axis, z_axis = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
    x_dist = np.linalg.norm(pos_main_axis - x_axis)
    y_dist = np.linalg.norm(pos_main_axis - y_axis)
    z_dist = np.linalg.norm(pos_main_axis - z_axis)

    # Determine if it's upright based on which axis it is closer to
    upright = z_dist <= x_dist and z_dist <= y_dist

    # Determine direction of main axis
    z = main_axis[2]
    if upright:
        # If it's closer to being upright base is likely at the bottom of the main axis
        if z < 0:
            # Flip
            main_axis = -main_axis
    else:
        # If it's on its side likely base is pointing up
        if z > 0:
            # Flip
            main_axis = -main_axis
    return main_axis


######################################################################
# Functions to determine direction of main axis using circle fitting on the perp plane
######################################################################

def project_cloud_onto_main_axis_perp_plane(point_cloud, main_axis):
    pts = point_cloud.xyzs().T # Nx3

    # Center the cloud
    centroid = np.mean(pts, axis=0)
    pts_centered = pts - centroid

    # Compute two perpendicular vectors to define the plane
    # Pick an arbitrary vector not parallel to main_axis
    arbitrary = np.array([1, 0, 0]) if abs(main_axis[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(main_axis, arbitrary)
    u /= np.linalg.norm(u)
    v = np.cross(main_axis, u)
    v /= np.linalg.norm(v)

    # Project points onto the plane
    proj_coords_u = np.dot(pts_centered, u)
    proj_coords_v = np.dot(pts_centered, v)
    heights = np.dot(pts_centered, main_axis) # height along main_axis

    return proj_coords_u, proj_coords_v, heights

def fit_circle_2d(x, y):
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    b = x**2 + y**2
    c, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    x0, y0, c0 = c
    r = np.sqrt(c0 + x0**2 + y0**2)
    # compute mean squared error
    dists = np.sqrt((x - x0)**2 + (y - y0)**2)
    mse = np.mean((dists - r)**2)
    return np.array([x0, y0]), r, mse

def get_cloud_main_axis(point_cloud):
    # Get cloud main axis
    main_axis = get_cloud_main_axis_pca(point_cloud)

    # Project cloud onto the plane perpendicular to the main axis
    x, y, z = project_cloud_onto_main_axis_perp_plane(point_cloud, main_axis)

    # DBSCAN params
    z_range = np.max(z) - np.min(z)
    eps = z_range / 100
    min_samples = len(z) / 100
    z_reshaped = z.reshape(-1, 1) # needed since z is a 1D array

    # Cluster the z values to define slices along the main axis
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(z_reshaped)
    labels = db.labels_
    unique_labels = [l for l in np.unique(labels) if l != -1] # skip noise
    
    # Group x and y by cluster
    clusters = {}
    for label in unique_labels:
        mask = labels == label
        cluster_x = x[mask]
        cluster_y = y[mask]
        cluster_z = z[mask]
        clusters[label] = {
            'x': cluster_x,
            'y': cluster_y,
            'z': cluster_z
        }

    # Fit a circle to each cluster
    for cluster in clusters:
        x, y = clusters[cluster]['x'], clusters[cluster]['y']
        x0, y0, r, mse = fit_circle_2d(x, y)
        print(x0, y0, r, mse)