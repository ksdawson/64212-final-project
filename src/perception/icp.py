import os
import numpy as np
import trimesh
from sklearn.cluster import KMeans
from pydrake.all import (
    Concatenate, PointCloud, RigidTransform, RollPitchYaw
)
from manipulation.icp import IterativeClosestPoint

def get_model_point_cloud(filepath, scale, N_SAMPLE_POINTS = 1500):
    # Load object as mesh
    mesh = trimesh.load(file_obj=filepath, file_type="obj", force='mesh')

    # Sample a subset of points from the mesh for faster ICP
    points = mesh.sample(N_SAMPLE_POINTS)
    points /= scale
    points = np.array(points).T

    # Create a PointCloud
    cloud = PointCloud(points.shape[1])
    cloud.mutable_xyzs()[:] = points

    return cloud

def get_model_point_clouds():
    current_directory = os.getcwd()
    clouds = {}

    # Get table
    table_path = f'{current_directory}/assets/furniture/table1/model.obj'
    table_cloud = get_model_point_cloud(table_path, 100)
    clouds['table'] = table_cloud

    # Get chessboard
    chessboard_path = f'{current_directory}/assets/chess/chessboard/model.obj'
    chessboard_cloud = get_model_point_cloud(chessboard_path, 100)
    clouds['chessboard'] = chessboard_cloud

    # Get pieces
    colors = ['dark', 'light']
    pieces = ['pawn', 'king', 'queen', 'bishop', 'knight', 'rook']
    clouds['pieces'] = {}
    pawn_scaling_factor = 5000
    back_pieces_scaling_factor = 6500
    for color in colors:
        clouds['pieces'][color] = {}
        for piece in pieces:
            name = f'{color}_{piece}'
            piece_path = f'{current_directory}/assets/chess/pieces/individual_pieces/{name}/model.obj'
            scaling_factor = pawn_scaling_factor if piece == 'pawn' else back_pieces_scaling_factor
            piece_cloud = get_model_point_cloud(piece_path, scaling_factor)
            clouds['pieces'][color][piece] = piece_cloud

    return clouds

def get_scene_point_cloud(diagram, context):
    # Get the clouds from the depth cameras
    camera0_point_cloud = diagram.GetOutputPort("camera0_point_cloud").Eval(context)
    camera1_point_cloud = diagram.GetOutputPort("camera1_point_cloud").Eval(context)
    camera2_point_cloud = diagram.GetOutputPort("camera2_point_cloud").Eval(context)

    # Concatenate the point clouds
    pcd = [
        camera0_point_cloud,
        camera1_point_cloud,
        camera2_point_cloud
    ]
    merged_pcd = Concatenate(pcd)

    return merged_pcd

# Let's make the floor, table, chessboard, and IIWAs unmovable so known
# (the API will have a function returning their poses?)
# Only chess pieces change locations so ICP should crop out everything but those

def ReverseCrop(pc, lower, upper):
    "Remove all points within a bounding box by cropping the 6 outer regions."

    lower = np.array(lower)
    upper = np.array(upper)

    # Everything below/above each face
    # Region order doesn't matter
    crops = []

    # (-∞, lower_x)
    crops.append(pc.Crop(lower_xyz=[-np.inf, -np.inf, -np.inf],
                         upper_xyz=[lower[0], np.inf, np.inf]))
    # (upper_x, ∞)
    crops.append(pc.Crop(lower_xyz=[upper[0], -np.inf, -np.inf],
                         upper_xyz=[np.inf, np.inf, np.inf]))

    # (lower_x .. upper_x) ∧ (-∞ .. lower_y)
    crops.append(pc.Crop(lower_xyz=[lower[0], -np.inf, -np.inf],
                         upper_xyz=[upper[0], lower[1], np.inf]))
    # (lower_x .. upper_x) ∧ (upper_y .. ∞)
    crops.append(pc.Crop(lower_xyz=[lower[0], upper[1], -np.inf],
                         upper_xyz=[upper[0], np.inf, np.inf]))

    # (lower_x .. upper_x) ∧ (lower_y .. upper_y) ∧ (-∞ .. lower_z)
    crops.append(pc.Crop(lower_xyz=[lower[0], lower[1], -np.inf],
                         upper_xyz=[upper[0], upper[1], lower[2]]))
    # (lower_x .. upper_x) ∧ (lower_y .. upper_y) ∧ (upper_z .. ∞)
    crops.append(pc.Crop(lower_xyz=[lower[0], lower[1], upper[2]],
                         upper_xyz=[upper[0], upper[1], np.inf]))

    # Combine
    return Concatenate(crops)

def segment_scene_point_cloud(scene_point_cloud):
    # Object dimensions
    floor_z = 0.01
    table_z = 0.4746 + 0.012721
    chessboard_z = 0.0154
    piece_z = 494.27 / 6500 # Tallest piece standing up is the King

    # Object positions
    chessboard_x = (-0.4325/2 - 0.001, 0.4325/2 + 0.001) # These epsilon values work but are kind of imprecise
    chessboard_y = (-0.4325/2 - 0.002, 0.4325/2 + 0.002)
    table_x = (-1.4769999999999999/2, 1.4769999999999999/2)
    table_y = (-0.9492/2, 0.9492/2)

    # Crop out the floor by cropping below z=0.01
    scene_point_cloud = scene_point_cloud.Crop(lower_xyz=[-np.inf, -np.inf, floor_z], upper_xyz=[np.inf, np.inf, np.inf])

    # The pieces can only be on top of the table, chessboard, or on the ground.
    # (1) Piece on ground
    floor_region = scene_point_cloud.Crop(
        lower_xyz=[-np.inf, -np.inf, floor_z],
        upper_xyz=[np.inf, np.inf, floor_z + piece_z]
    )
    # (2) Piece on table/chessboard
    table_region = scene_point_cloud.Crop(
        lower_xyz=[table_x[0], table_y[0], floor_z + table_z],
        upper_xyz=[table_x[1], table_y[1], floor_z + table_z + chessboard_z + piece_z]
    )
    scene_point_cloud = Concatenate([floor_region, table_region])
    # Crop out chessboard
    scene_point_cloud = ReverseCrop(
        scene_point_cloud,
        [chessboard_x[0], chessboard_y[0], floor_z + table_z],
        [chessboard_x[1], chessboard_y[1], floor_z + table_z + chessboard_z]
    )
    # Crop out regions on table above table height + piece height
    scene_point_cloud = ReverseCrop(scene_point_cloud, [table_x[0], table_y[0], floor_z + table_z + piece_z], [table_x[1], chessboard_y[0], np.inf])
    scene_point_cloud = ReverseCrop(scene_point_cloud, [table_x[0], chessboard_y[1], floor_z + table_z + piece_z], [table_x[1], table_y[1], np.inf])
    scene_point_cloud = ReverseCrop(scene_point_cloud, [table_x[0], table_y[0], floor_z + table_z + piece_z], [chessboard_x[0], table_y[1], np.inf])
    scene_point_cloud = ReverseCrop(scene_point_cloud, [chessboard_x[1], table_y[0], floor_z + table_z + piece_z], [table_x[1], table_y[1], np.inf])

    # Crop out IIWA bases
    # Source: https://github.com/RobotLocomotion/models/blob/master/iiwa_description/sdf/iiwa7_with_box_collision.sdf
    iiwa_box_origin = [-0.004563, 0, 0.07875]
    iiwa_box_xyz = [0.216874, 0.207874, 0.1575]
    # iiwa 1
    p_WI_1 = np.array([0, -0.75, 0.01])
    p_IB_1 = np.array(iiwa_box_origin)
    R_WB_1 = RollPitchYaw(0, 0, np.deg2rad(180)).ToRotationMatrix()
    p_WB_1 = p_WI_1 + R_WB_1 @ p_IB_1
    scene_point_cloud = ReverseCrop(
        scene_point_cloud,
        [p_WB_1[0] - iiwa_box_xyz[0]/2, p_WB_1[1] - iiwa_box_xyz[1]/2, p_WB_1[2] - iiwa_box_xyz[2]/2],
        [p_WB_1[0] + iiwa_box_xyz[0]/2, p_WB_1[1] + iiwa_box_xyz[1]/2, p_WB_1[2] + iiwa_box_xyz[2]/2]
    )
    # iiwa 2
    p_WI_2 = np.array([0, 0.75, 0.01])
    p_IB_2 = np.array(iiwa_box_origin)
    R_WB_2 = RollPitchYaw(0, 0, 0).ToRotationMatrix()
    p_WB_2 = p_WI_2 + R_WB_2 @ p_IB_2
    scene_point_cloud = ReverseCrop(
        scene_point_cloud,
        [p_WB_2[0] - iiwa_box_xyz[0]/2, p_WB_2[1] - iiwa_box_xyz[1]/2, p_WB_2[2] - iiwa_box_xyz[2]/2],
        [p_WB_2[0] + iiwa_box_xyz[0]/2, p_WB_2[1] + iiwa_box_xyz[1]/2, p_WB_2[2] + iiwa_box_xyz[2]/2]
    )

    # Crop out table leg bases
    eps_x, eps_y = 0.005, 0.0035
    leg1 = [-0.711963, 0.409149 - eps_y, floor_z], [-0.65607 + eps_x, 0.448578 + eps_y, floor_z + table_z] # left front
    leg2 = [-0.711963, -0.446239 - eps_y, floor_z], [-0.65607 + eps_x, -0.406809 + eps_y, floor_z + table_z] # left back
    leg3 = [0.656216 - eps_x, 0.409149 - eps_y, floor_z], [0.712109, 0.448578 + eps_y, floor_z + table_z] # right front
    leg4 = [0.656216 - eps_x, -0.446239 - eps_y, floor_z], [0.712109, -0.406809 + eps_y, floor_z + table_z] # right back
    scene_point_cloud = ReverseCrop(scene_point_cloud, leg1[0], leg1[1])
    scene_point_cloud = ReverseCrop(scene_point_cloud, leg2[0], leg2[1])
    scene_point_cloud = ReverseCrop(scene_point_cloud, leg3[0], leg3[1])
    scene_point_cloud = ReverseCrop(scene_point_cloud, leg4[0], leg4[1])

    # Downsample to speed up ICP
    scene_point_cloud = scene_point_cloud.VoxelizedDownSample(voxel_size=0.005)

    return scene_point_cloud

def cluster_point_cloud(point_cloud, num_clusters=32):
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

def classify_by_color(piece_clouds):
    # Color of dark and light pieces
    DARK_RGB = np.array([0.16078431 * 255, 0.04313725 * 255, 0.00392157 * 255])
    LIGHT_RGB = np.array([0.64705882 * 255, 0.56470588 * 255, 0.43529412 * 255])

    # Classify piece clouds by color
    classifications = []
    for cloud in piece_clouds:
        # Average the color of the point cloud to get the piece color
        colors = cloud.rgbs()
        rgb = np.mean(colors, axis=1)

        # Classify it by determining which color its closer to
        if rgb.ndim == 1:
            dist_dark = np.linalg.norm(rgb - DARK_RGB)
            dist_light = np.linalg.norm(rgb - LIGHT_RGB)
            classification = 'dark' if dist_dark < dist_light else 'light'
        else:
            dist_dark = np.linalg.norm(rgb - DARK_RGB, axis=1)
            dist_light = np.linalg.norm(rgb - LIGHT_RGB, axis=1)
            classification = np.where(dist_dark < dist_light, 'dark', 'light')
        classifications.append(classification)
    return classifications

def compute_icp_error(X_Ohat, p_Om, p_Ws, chat):
    p_Om_transformed = X_Ohat @ p_Om[:, chat]
    errors = p_Ws - p_Om_transformed
    rms_error = np.sqrt(np.mean(np.sum(errors**2, axis=0)))
    return rms_error

def match_scene_to_model_cloud(scene_point_clouds, model_point_clouds, MAX_ITERATIONS = 25):
    initial_guess = RigidTransform()
    pairings = []
    for scene_point_cloud in scene_point_clouds:
        scene = np.array(scene_point_cloud.xyzs())
        error = np.inf
        model_name = None
        for name, model_point_cloud in model_point_clouds.items():
            model = np.array(model_point_cloud.xyzs())
            X_Ohat, chat = IterativeClosestPoint(
                p_Om=model,
                p_Ws=scene,
                X_Ohat=initial_guess,
                max_iterations=MAX_ITERATIONS
            )
            icp_error = compute_icp_error(X_Ohat, model, scene, chat)
            if icp_error < error:
                model_name = name
                error = icp_error
        pairings.append((model_name, error))
    return pairings

def run_icp(scene_point_cloud, model_point_clouds, MAX_ITERATIONS = 25):
    scene = np.array(scene_point_cloud.xyzs())
    colors = ['dark', 'light']
    pieces = ['pawn', 'king', 'queen', 'bishop', 'knight', 'rook']
    initial_guess = RigidTransform()
    poses = {}
    for color in colors:
        poses[color] = {}
        for piece in pieces:
            model_point_cloud = model_point_clouds['pieces'][color][piece]
            model = np.array(model_point_cloud.xyzs())
            X_Ohat, chat = IterativeClosestPoint(
                p_Om=model,
                p_Ws=scene,
                X_Ohat=initial_guess,
                max_iterations=MAX_ITERATIONS
            )
            poses[color][piece] = X_Ohat
    return poses

if __name__ == '__main__':
    get_model_point_clouds()