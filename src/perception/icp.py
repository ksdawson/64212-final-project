import os
import numpy as np
import trimesh
from pydrake.all import (
    Concatenate,
    PointCloud
)

def get_model_point_cloud(filepath, N_SAMPLE_POINTS = 1500):
    # Load object as mesh
    mesh = trimesh.load(file_obj=filepath, file_type="obj", force='mesh')

    # Sample a subset of points from the mesh for faster ICP
    points = mesh.sample(N_SAMPLE_POINTS)
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
    table_cloud = get_model_point_cloud(table_path)
    clouds['table'] = table_cloud

    # Get chessboard
    chessboard_path = f'{current_directory}/assets/chess/chessboard/model.obj'
    chessboard_cloud = get_model_point_cloud(chessboard_path)
    clouds['chessboard'] = chessboard_cloud

    # Get pieces
    colors = ['dark', 'light']
    pieces = ['pawn', 'king', 'queen', 'bishop', 'knight', 'rook']
    clouds['pieces'] = {}
    for color in colors:
        clouds['pieces'][color] = {}
        for piece in pieces:
            name = f'{color}_{piece}'
            piece_path = f'{current_directory}/assets/chess/pieces/individual_pieces/{name}/model.obj'
            piece_cloud = get_model_point_cloud(piece_path)
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

def finalize_scene_point_cloud(scene_point_cloud):
    # Should crop out the floor, table, chessboard, and IIWAs

    # Crop out the floor
    scene_point_cloud = scene_point_cloud.Crop(lower_xyz=[-100, -100, 0.01], upper_xyz=[100, 100, 100])

    # Downsample to speed up ICP
    scene_point_cloud = scene_point_cloud.VoxelizedDownSample(voxel_size=0.005)

    return scene_point_cloud

if __name__ == '__main__':
    get_model_point_clouds()