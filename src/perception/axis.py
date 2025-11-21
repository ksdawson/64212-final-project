import numpy as np

######################################################################
# Compute main axis of piece cloud with vector pointing in the
# direction of the top of the piece
######################################################################

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