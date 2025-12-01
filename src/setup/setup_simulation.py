import os
import pickle
from game.utils import Game
from setup.utils import get_xy_offsets

# Base string containing the IIWAs, table, and chessboard
# Table math: set at 0.01 -> 0.01 (floor height) - -0.012721 (chess obj base) = 0.022721
#             set at -0.1 -> -0.1 - -0.012721 = -0.087279
# Chessboard math:
# set at 0.474623 (table height) + 0.01 (table base) = 0.484623
# 0.484623 - -0.015968 (chessboard obj base) = 0.500591
# move down 0.1 -> 0.500591 - 0.11 = 0.390591 (actually 0.478391 idk why)
base_scenario_string = '''
directives:
    - add_model:
        name: camera0
        file: package://manipulation/camera_box.sdf
    - add_frame:
        name: camera0_origin
        X_PF:
            base_frame: world
            rotation: !Rpy {{ deg: [-120.0, 0.0, 90.0] }}
            translation: [2, 0, 1]
    - add_weld:
        parent: camera0_origin
        child: camera0::base
    - add_model:
        name: camera1
        file: package://manipulation/camera_box.sdf
    - add_frame:
        name: camera1_origin
        X_PF:
            base_frame: world
            rotation: !Rpy {{ deg: [-120.0, 0.0, 270.0] }}
            translation: [-2, 0, 1]
    - add_weld:
        parent: camera1_origin
        child: camera1::base
    - add_model:
        name: camera2
        file: package://manipulation/camera_box.sdf
    - add_frame:
        name: camera2_origin
        X_PF:
            base_frame: world
            rotation: !Rpy {{ deg: [0.0, 180.0, 0.0] }}
            translation: [0, 0, 1.5]
    - add_weld:
        parent: camera2_origin
        child: camera2::base
    - add_model:
        name: floor
        file: file://{FLOOR_PATH}
    - add_weld:
        parent: world
        child: floor::link
    - add_model:
        name: iiwa1
        file: package://drake_models/iiwa_description/sdf/iiwa7_with_box_collision.sdf
        default_joint_positions:
            iiwa_joint_1: [{IIWA1_J1}]
            iiwa_joint_2: [{IIWA1_J2}]
            iiwa_joint_3: [{IIWA1_J3}]
            iiwa_joint_4: [{IIWA1_J4}]
            iiwa_joint_5: [{IIWA1_J5}]
            iiwa_joint_6: [{IIWA1_J6}]
            iiwa_joint_7: [{IIWA1_J7}]
    - add_weld:
        parent: world
        child: iiwa1::iiwa_link_0
        X_PC:
            translation: [0, {IIWA1_BASE_DIST}, 0.25]
            rotation: !Rpy {{ deg: [0, 0, 0] }}
    - add_model:
        name: wsg1
        file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
    - add_weld:
        parent: iiwa1::iiwa_link_7
        child: wsg1::body
        X_PC:
            translation: [0, 0, 0.09]
            rotation: !Rpy {{ deg: [90, 0, 90] }}
    - add_model:
        name: iiwa2
        file: package://drake_models/iiwa_description/sdf/iiwa7_with_box_collision.sdf
        default_joint_positions:
            iiwa_joint_1: [{IIWA2_J1}]
            iiwa_joint_2: [{IIWA2_J2}]
            iiwa_joint_3: [{IIWA2_J3}]
            iiwa_joint_4: [{IIWA2_J4}]
            iiwa_joint_5: [{IIWA2_J5}]
            iiwa_joint_6: [{IIWA2_J6}]
            iiwa_joint_7: [{IIWA2_J7}]
    - add_weld:
        parent: world
        child: iiwa2::iiwa_link_0
        X_PC:
            translation: [0, {IIWA2_BASE_DIST}, 0.25]
            rotation: !Rpy {{ deg: [0, 0, 0] }}
    - add_model:
        name: wsg2
        file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
    - add_weld:
        parent: iiwa2::iiwa_link_7
        child: wsg2::body
        X_PC:
            translation: [0, 0, 0.09]
            rotation: !Rpy {{ deg: [90, 0, 90] }}
    - add_model:
        name: table
        file: file://{TABLE_PATH}
    - add_weld:
        parent: world
        child: table::link
        X_PC:
            translation: [0.0, 0.0, -0.087279]
    - add_model:
        name: chessboard
        file: file://{CHESSBOARD_PATH}
    - add_weld:
        parent: table::link
        child: chessboard::link
        X_PC:
            translation: [0.0, 0.0, 0.478391]
    {PIECES}
cameras:
    camera0:
        name: camera0
        depth: True
        X_PB:
            base_frame: camera0::base
    camera1:
        name: camera1
        depth: True
        X_PB:
            base_frame: camera1::base
    camera2:
        name: camera2
        depth: True
        X_PB:
            base_frame: camera2::base
model_drivers:
    iiwa1: !IiwaDriver
        control_mode: position_only
        hand_model_name: wsg1
    iiwa2: !IiwaDriver
        control_mode: position_only
        hand_model_name: wsg2
    wsg1: !SchunkWsgDriver {{}}
    wsg2: !SchunkWsgDriver {{}}
    default: !ZeroForceDriver {{}}
visualization:
    publish_contacts: false
    publish_proximity: false
'''
# String format for chess pieces
# Piece height: 0.527262 - 0.1 = 0.427262
piece_scenario_str = '''- add_model:
        name: {NAME}
        file: file://{PATH}
        default_free_body_pose:
            link:
                translation: [{X}, {Y}, 0.390591]
                rotation: !Rpy {{ deg: [90, 0, 0] }}'''

def get_pieces_poses():
    game = Game()
    piece_poses = {'dark': {}, 'light': {}}

    # Get offsets
    offsets = get_xy_offsets()

    # Pawns
    piece_poses['dark']['pawn'] = {}
    piece_poses['light']['pawn'] = {}
    for file_idx in range(8):
        file = chr(ord('a') + file_idx)
        # Dark pawns (rank 7)
        pos = game.square_to_pose(f'{file}7').translation()[:2]
        piece_poses['dark']['pawn'][file_idx] = (float(pos[0]) + offsets['dark']['pawn'][0], float(pos[1]) + offsets['dark']['pawn'][1])
        # Light pawns (rank 2)
        pos = game.square_to_pose(f'{file}2').translation()[:2]
        piece_poses['light']['pawn'][file_idx] = (float(pos[0]) + offsets['light']['pawn'][0], float(pos[1]) + offsets['light']['pawn'][1])

    # Kings
    for color, square in [('dark', 'e8'), ('light', 'e1')]:
        pos = game.square_to_pose(square).translation()[:2]
        piece_poses[color]['king'] = {0: (float(pos[0]) + offsets[color]['king'][0], float(pos[1]) + offsets[color]['king'][1])}

    # Queens
    for color, square in [('dark', 'd8'), ('light', 'd1')]:
        pos = game.square_to_pose(square).translation()[:2]
        piece_poses[color]['queen'] = {0: (float(pos[0]) + offsets[color]['queen'][0], float(pos[1]) + offsets[color]['queen'][1])}

    # Bishops
    for color, squares in [('dark',  ['c8', 'f8']), ('light', ['c1', 'f1'])]:
        piece_poses[color]['bishop'] = {}
        for idx, sq in enumerate(squares):
            pos = game.square_to_pose(sq).translation()[:2]
            piece_poses[color]['bishop'][idx] = (float(pos[0]) + offsets[color]['bishop'][0], float(pos[1]) + offsets[color]['bishop'][1])

    # Knights
    for color, squares in [('dark',  ['b8', 'g8']), ('light', ['b1', 'g1'])]:
        piece_poses[color]['knight'] = {}
        for idx, sq in enumerate(squares):
            pos = game.square_to_pose(sq).translation()[:2]
            piece_poses[color]['knight'][idx] = (float(pos[0]) + offsets[color]['knight'][0], float(pos[1]) + offsets[color]['knight'][1])

    # Rooks
    for color, squares in [('dark',  ['a8', 'h8']), ('light', ['a1', 'h1'])]:
        piece_poses[color]['rook'] = {}
        for idx, sq in enumerate(squares):
            pos = game.square_to_pose(sq).translation()[:2]
            piece_poses[color]['rook'][idx] = (float(pos[0]) + offsets[color]['rook'][0], float(pos[1]) + offsets[color]['rook'][1])

    return piece_poses

def create_scenario():
    # Get paths to assets
    current_directory = os.getcwd()
    chess_assets_directory = 'assets/chess'
    furniture_assets_directory = 'assets/furniture'
    room_assets_directory = 'assets/room'

    # Get room
    floor_path = f'{current_directory}/{room_assets_directory}/floor.sdf'

    # Get table
    table_path = f'{current_directory}/{furniture_assets_directory}/table1/model.sdf'

    # Get chessboard
    chessboard_path = f'{current_directory}/{chess_assets_directory}/chessboard/model.sdf'

    # Get pieces
    piece_poses = get_pieces_poses()
    piece_strs = []
    for color in piece_poses:
        for piece in piece_poses[color]:
            name = f'{color}_{piece}'
            piece_path = f'{current_directory}/{chess_assets_directory}/pieces/individual_pieces/{name}/model.sdf'
            for p in piece_poses[color][piece]:
                name_copy = f'{name}_{p}'
                x, y = piece_poses[color][piece][p]
                model_str = piece_scenario_str.format(NAME=name_copy, PATH=piece_path, X=x, Y=y)
                piece_strs.append(model_str)
    
    # Get iiwa starting configurations
    file_path = 'starting_configuration.pkl'
    with open(file_path, 'rb') as file:
        # Load the data from the pickle file
        data = pickle.load(file)
    iiwa1_config, iiwa2_config = data[1], data[2]

    # Create scenario
    scenario_string = base_scenario_string.format(
        FLOOR_PATH=floor_path,
        TABLE_PATH=table_path,
        CHESSBOARD_PATH=chessboard_path,
        PIECES='\n    '.join(piece_strs),
        IIWA1_BASE_DIST=iiwa1_config['base_dist'], IIWA2_BASE_DIST=iiwa2_config['base_dist'],
        IIWA1_J1=iiwa1_config['j1'], IIWA1_J2=iiwa1_config['j2'], IIWA1_J3=iiwa1_config['j3'], IIWA1_J4=iiwa1_config['j4'], IIWA1_J5=iiwa1_config['j5'], IIWA1_J6=iiwa1_config['j6'], IIWA1_J7=iiwa1_config['j7'],
        IIWA2_J1=iiwa2_config['j1'], IIWA2_J2=iiwa2_config['j2'], IIWA2_J3=iiwa2_config['j3'], IIWA2_J4=iiwa2_config['j4'], IIWA2_J5=iiwa2_config['j5'], IIWA2_J6=iiwa2_config['j6'], IIWA2_J7=iiwa2_config['j7']
    )

    # Output to file
    out_file = f'{current_directory}/scenario.yaml'
    with open(out_file, 'w') as f:
        f.write(scenario_string)

    return scenario_string

def get_scenario():
    current_directory = os.getcwd()
    file_path = f'{current_directory}/scenario.yaml'
    with open(file_path, 'r') as f:
        content = f.read()
        return content

if __name__ == '__main__':
    create_scenario()