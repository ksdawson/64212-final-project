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
    scene_point_cloud = scene_point_cloud.Crop(lower_xyz=[-100, -100, floor_z], upper_xyz=[100, 100, 100])

    # The pieces can only be on top of the table, chessboard, or on the ground.
    # (1) Piece on ground
    floor_region = scene_point_cloud.Crop(
        lower_xyz=[-100, -100, floor_z],
        upper_xyz=[100, 100, floor_z + piece_z]
    )

    # (2) Piece on table
    table_region1 = scene_point_cloud.Crop(
        lower_xyz=[table_x[0], table_y[0], floor_z + table_z],
        upper_xyz=[chessboard_x[0], table_y[1], floor_z + table_z + piece_z]
    )
    table_region2 = scene_point_cloud.Crop(
        lower_xyz=[chessboard_x[1], table_y[0], floor_z + table_z],
        upper_xyz=[table_x[1], table_y[1], floor_z + table_z + piece_z]
    )
    table_region3 = scene_point_cloud.Crop(
        lower_xyz=[table_x[0], table_y[0], floor_z + table_z],
        upper_xyz=[table_x[1], chessboard_y[0], floor_z + table_z + piece_z]
    )
    table_region4 = scene_point_cloud.Crop(
        lower_xyz=[table_x[0], chessboard_y[1], floor_z + table_z],
        upper_xyz=[table_x[1], table_y[1], floor_z + table_z + piece_z]
    )
    table_region = Concatenate([table_region1, table_region2, table_region3, table_region4])

    # (3) Piece on chessboard
    chessboard_region = scene_point_cloud.Crop(
        lower_xyz=[chessboard_x[0], chessboard_y[0], floor_z + table_z + chessboard_z],
        upper_xyz=[chessboard_x[1], chessboard_y[1], floor_z + table_z + chessboard_z + piece_z]
    )

    # TODO: Crop out IIWA bases and bases of table legs

    # Combine the regions
    scene_point_cloud = Concatenate([floor_region, table_region, chessboard_region])

    # Downsample to speed up ICP
    scene_point_cloud = scene_point_cloud.VoxelizedDownSample(voxel_size=0.005)

    return scene_point_cloud

if __name__ == '__main__':
    get_model_point_clouds()