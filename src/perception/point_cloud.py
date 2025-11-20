import os
import numpy as np
import trimesh
from pydrake.all import (
    Concatenate, PointCloud
)

######################################################################
# Functions to load point clouds
######################################################################

def get_model_point_cloud(filepath, scale, n_sample_points = 1500):
    # Load object as mesh
    mesh = trimesh.load(file_obj=filepath, file_type='obj', force='mesh')

    # Sample a subset of points from the mesh for faster ICP
    points = mesh.sample(n_sample_points)
    points /= scale # scale to size of scene
    points = np.array(points).T

    # Create a PointCloud
    cloud = PointCloud(points.shape[1])
    cloud.mutable_xyzs()[:] = points

    return cloud

def get_model_point_clouds(n_sample_points = 1500):
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
            piece_cloud = get_model_point_cloud(piece_path, scaling_factor, n_sample_points=n_sample_points)
            clouds['pieces'][color][piece] = piece_cloud

    return clouds

def get_scene_point_cloud(diagram, context):
    # Get the clouds from the depth cameras
    camera0_point_cloud = diagram.GetOutputPort('camera0_point_cloud').Eval(context)
    camera1_point_cloud = diagram.GetOutputPort('camera1_point_cloud').Eval(context)
    camera2_point_cloud = diagram.GetOutputPort('camera2_point_cloud').Eval(context)

    # Concatenate the point clouds
    pcd = [
        camera0_point_cloud,
        camera1_point_cloud,
        camera2_point_cloud
    ]
    merged_pcd = Concatenate(pcd)

    return merged_pcd

######################################################################
# Cloud utils
######################################################################

def ReverseCrop(pc, lower, upper):
    'Remove all points within a bounding box by cropping the 6 outer regions.'

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

if __name__ == '__main__':
    get_model_point_clouds()