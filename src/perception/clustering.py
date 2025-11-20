import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from pydrake.all import (
    Concatenate, PointCloud
)

######################################################################
# Distance-based clustering
######################################################################

def cluster_point_cloud_kmeans(point_cloud, num_clusters=32):
    # Convert point cloud to xyz matrix
    points = point_cloud.xyzs() # 3xN
    points_T = points.T # Nx3

    # Run KMeans
    kmeans = KMeans(n_clusters=num_clusters, random_state=42)
    labels = kmeans.fit_predict(points_T) # labels[i] = cluster index for point i

    # Create point clouds per cluster
    cluster_pointclouds = []
    for i in range(num_clusters):
        # Filter points
        mask = (labels == i)
        cluster_points = points[:, mask] # 3 x N_i
        # Create a new point cloud
        pc = PointCloud(cluster_points.shape[1], fields=point_cloud.fields())
        pc.mutable_xyzs()[:] = cluster_points
        # Add the colors
        if point_cloud.has_rgbs():
            colors = point_cloud.rgbs()[:, mask]
            pc.mutable_rgbs()[:] = colors
        cluster_pointclouds.append(pc)

    return cluster_pointclouds

######################################################################
# Density-based clustering
######################################################################

def cluster_point_cloud_dbscan(point_cloud, eps=0.02, min_samples=32):
    # Convert point cloud to xyz matrix
    points = point_cloud.xyzs() # 3xN
    pts = points.T # Nx3

    # Run DBSCAN
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(pts)
    labels = db.labels_  # labels[i] = cluster index OR -1 for noise

    unique_labels = [l for l in np.unique(labels) if l != -1] # skip noise
    cluster_pointclouds = []

    for label in unique_labels:
        mask = (labels == label)
        cluster_points = points[:, mask] # 3xNi

        pc = PointCloud(cluster_points.shape[1], fields=point_cloud.fields())
        pc.mutable_xyzs()[:] = cluster_points

        if point_cloud.has_rgbs():
            pc.mutable_rgbs()[:] = point_cloud.rgbs()[:, mask]

        cluster_pointclouds.append(pc)

    return cluster_pointclouds

######################################################################
# Hybrid clustering
######################################################################

def compute_centroids(clusters):
    centroids = []
    for pc in clusters:
        pts = pc.xyzs() # 3xN
        centroid = np.mean(pts, axis=1)
        centroids.append(centroid)
    return np.vstack(centroids) # Kx3

def cluster_point_cloud_hybrid(point_cloud, num_clusters=32):
    # Cluster by density using DBSCAN
    db_clusters = cluster_point_cloud_dbscan(point_cloud, min_samples=num_clusters) # returns >= num_clusters
    if len(db_clusters) < num_clusters:
        # Fallback is KMeans
        return cluster_point_cloud_kmeans(point_cloud, num_clusters=num_clusters)

    # Compute centroids of DBSCAN clusters
    centroids = compute_centroids(db_clusters) # Kx3

    # Cluster centroids using KMeans to reduce them to num_clusters
    kmeans = KMeans(n_clusters=num_clusters, random_state=42)
    centroid_labels = kmeans.fit_predict(centroids)

    # Merge DBSCAN clusters whose centroids fall in the same KMeans cluster
    final_clusters = []
    for i in range(num_clusters):
        # Collect DBSCAN clusters belonging to centroid cluster i
        pcs_to_merge = [
            db_clusters[idx]
            for idx, label in enumerate(centroid_labels)
            if label == i
        ]
        if len(pcs_to_merge) == 0:
            continue

        # Merge the clouds
        merged_pc = Concatenate(pcs_to_merge)
        final_clusters.append(merged_pc)

    return final_clusters