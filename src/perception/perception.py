import numpy as np
from scipy.optimize import linear_sum_assignment
from pydrake.all import (
    Concatenate, RollPitchYaw, RigidTransform
)
from perception.point_cloud import ReverseCrop
from perception.color import classify_by_color
from perception.icp import run_icp
from perception.bounding_box import cloud_bounding_box_similarity, cloud_oriented_bounding_box_similarity
from perception.axis import get_piece_cloud_main_axis

######################################################################
# Segmentation stage
######################################################################

def segment_scene_point_cloud(scene_point_cloud, voxel_size=0.005):
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
    scene_point_cloud = scene_point_cloud.VoxelizedDownSample(voxel_size=voxel_size)

    return scene_point_cloud

######################################################################
# Color classification stage
######################################################################

def classify_piece_colors(piece_clouds):
    # Color of dark and light pieces
    DARK_RGB = np.array([0.16078431 * 255, 0.04313725 * 255, 0.00392157 * 255])
    LIGHT_RGB = np.array([0.64705882 * 255, 0.56470588 * 255, 0.43529412 * 255])
    return classify_by_color(piece_clouds, [('dark', DARK_RGB), ('light', LIGHT_RGB)])

######################################################################
# Scene to model pairing stage
######################################################################
PIECE_COUNTS = {'pawn': 8, 'rook': 2, 'knight': 2, 'bishop': 2, 'queen': 1, 'king': 1}

def match_scene_to_model_cloud_icp(scene_point_clouds, model_point_clouds, max_iters = 100):
    # Pieces
    model_names = list(model_point_clouds.keys())

    # Checks to ensure inputs are correct
    assert len(scene_point_clouds) == 16, 'Should be 16 scene point clouds'
    assert len(model_point_clouds) == 6, 'Should be 6 model point clouds'
    assert set(model_names) == set(PIECE_COUNTS.keys()), 'Incorrect model names'

    # Build cost matrix (scene_count x model_count)
    cost = np.zeros((len(scene_point_clouds), len(model_point_clouds)))
    poses = [[None for _ in model_names] for _ in scene_point_clouds]

    # Partially fill in the cost matrix using the unique scene, model pairs
    for i, scene_pc in enumerate(scene_point_clouds):
        # Use the scene centroid as the initial guess
        pts = scene_pc.xyzs()
        centroid = np.mean(pts, axis=1)
        initial_pose = RigidTransform()
        initial_pose.set_translation(centroid)
        for j, name in enumerate(model_names):
            model_pc = model_point_clouds[name]
            X_Ohat, chat, icp_error, icp_error_alt = run_icp(scene_pc, model_pc, max_iters=max_iters, initial_guess=initial_pose)
            cost[i, j] = icp_error
            poses[i][j] = X_Ohat # store pose for (scene i, model j)
    
    # Expand cost matrix using PIECE_COUNTS
    expanded_model_names = []
    expanded_cost = []
    expanded_poses = []
    for j, name in enumerate(model_names):
        count = PIECE_COUNTS[name]
        for k in range(count):
            expanded_model_names.append(f'{name}{k+1}')
            expanded_cost.append(cost[:, j]) # reuse exact same column
            expanded_poses.append([poses[i][j] for i in range(len(scene_point_clouds))]) # and poses repeated
    expanded_cost = np.column_stack(expanded_cost)

    # Hungarian algorithm for optimal assignment
    row_ind, col_ind = linear_sum_assignment(expanded_cost)

    # Build mapping of piece-instance to pose
    result = {}
    for i, j in zip(row_ind, col_ind):
        piece_name = expanded_model_names[j]
        X_Ohat = expanded_poses[j][i] # pose for this pairing
        result[piece_name] = X_Ohat
    return result

def match_scene_to_model_cloud_bb(scene_point_clouds, model_point_clouds, obb=True):
    # Pieces
    model_names = list(model_point_clouds.keys())

    # Checks to ensure inputs are correct
    assert len(scene_point_clouds) == 16, 'Should be 16 scene point clouds'
    assert len(model_point_clouds) == 6, 'Should be 6 model point clouds'
    assert set(model_names) == set(PIECE_COUNTS.keys()), 'Incorrect model names'

    # Build cost matrix (scene_count x model_count)
    cost = np.zeros((len(scene_point_clouds), len(model_point_clouds)))

    # Partially fill in the cost matrix using the unique scene, model pairs
    for i, scene_pc in enumerate(scene_point_clouds):
        if obb:
            scene_main_axis = get_piece_cloud_main_axis(scene_pc)
        for j, name in enumerate(model_names):
            model_pc = model_point_clouds[name]
            if obb:
                bb_similarity = cloud_oriented_bounding_box_similarity(scene_pc, model_pc, scene_main_axis, None)
            else:
                bb_similarity = cloud_bounding_box_similarity(scene_pc, model_pc)
            cost[i, j] = bb_similarity

    # Expand cost matrix using PIECE_COUNTS
    expanded_model_names = []
    expanded_cost = []
    expanded_poses = []
    for j, name in enumerate(model_names):
        count = PIECE_COUNTS[name]
        for k in range(count):
            expanded_model_names.append(f'{name}{k+1}')
            expanded_cost.append(cost[:, j]) # reuse exact same column
    expanded_cost = np.column_stack(expanded_cost)

    # Hungarian algorithm for optimal assignment
    row_ind, col_ind = linear_sum_assignment(expanded_cost)

    # Build mapping of piece-instance to pose
    mapping = [None] * len(scene_point_clouds)
    for i, j in zip(row_ind, col_ind):
        piece_name = expanded_model_names[j]
        mapping[i] = piece_name
    return mapping