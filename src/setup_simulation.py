import os
from pydrake.all import (
    DiagramBuilder, StartMeshcat, Simulator
)
from manipulation.station import LoadScenario, MakeHardwareStation

# Base string containing the IIWAs, table, and chessboard
base_scenario_string = '''
directives:
    - add_model:
        name: floor
        file: file://{FLOOR_PATH}
    - add_weld:
        parent: world
        child: floor::link
    - add_model:
        name: iiwa1
        file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
        default_joint_positions:
            iiwa_joint_1: [-1.57]
            iiwa_joint_2: [0.1]
            iiwa_joint_3: [0]
            iiwa_joint_4: [-1.2]
            iiwa_joint_5: [0]
            iiwa_joint_6: [ 1.6]
            iiwa_joint_7: [0]
    - add_weld:
        parent: world
        child: iiwa1::iiwa_link_0
        X_PC:
            translation: [0, -0.75, 0.01]
            rotation: !Rpy {{ deg: [0, 0, 180] }}
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
        file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
        default_joint_positions:
            iiwa_joint_1: [-1.57]
            iiwa_joint_2: [0.1]
            iiwa_joint_3: [0]
            iiwa_joint_4: [-1.2]
            iiwa_joint_5: [0]
            iiwa_joint_6: [ 1.6]
            iiwa_joint_7: [0]
    - add_weld:
        parent: world
        child: iiwa2::iiwa_link_0
        X_PC:
            translation: [0, 0.75, 0.01]
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
            translation: [0.0, 0.0, 0.022721]
    - add_model:
        name: chessboard
        file: file://{CHESSBOARD_PATH}
    - add_weld:
        parent: table::link
        child: chessboard::link
        X_PC:
            translation: [0.0, 0.0, 0.4846]
    {PIECES}
visualization:
    publish_contacts: true
    publish_proximity: true
'''
# String format for chess pieces
piece_scenario_str = '''- add_model:
        name: {NAME}
        file: file://{PATH}
        default_free_body_pose:
            link:
                translation: [{X}, {Y}, 0.527262]
                rotation: !Rpy {{ deg: [90, 0, 0] }}'''

def get_pieces_poses():
    # Piece poses
    e7_x, e7_y = -0.155, -0.4225
    sq_size = 0.047
    piece_poses = {}

    # Add pawns
    piece_poses['dark'] = {}
    piece_poses['dark']['pawn'] = {}
    for p in range(-3, 5):
        p7_x, p7_y = e7_x + p * sq_size, e7_y
        piece_poses['dark']['pawn'][p+3] = p7_x, p7_y
    piece_poses['light'] = {}
    piece_poses['light']['pawn'] = {}
    for p in range(-3, 5):
        p2_x, p2_y = e7_x + p * sq_size, e7_y + 2.125 * sq_size
        piece_poses['light']['pawn'][p+3] = p2_x, p2_y

    # Add kings
    piece_poses['dark']['king'] = {0: (e7_x + 1.4 * sq_size, e7_y + 0.5 * sq_size)}
    piece_poses['light']['king'] = {0: (e7_x + 1.4 * sq_size, e7_y + 5.25 * sq_size)}

    # Add queens
    piece_poses['dark']['queen'] = {0: (e7_x + 3.35 * sq_size, e7_y + 0.5 * sq_size)}
    piece_poses['light']['queen'] = {0: (e7_x + 3.35 * sq_size, e7_y + 5.25 * sq_size)}

    # Add bishops
    piece_poses['dark']['bishop'] = {
        0: (e7_x + 5.25 * sq_size, e7_y + 0.5 * sq_size),
        1: (e7_x + 2.25 * sq_size, e7_y + 0.5 * sq_size)
    }
    piece_poses['light']['bishop'] = {
        0: (e7_x + 5.25 * sq_size, e7_y + 5.25 * sq_size),
        1: (e7_x + 2.25 * sq_size, e7_y + 5.25 * sq_size)
    }

    # Add knights
    piece_poses['dark']['knight'] = {
        0: (e7_x + 7.125 * sq_size, e7_y + 0.5 * sq_size),
        1: (e7_x + 2.125 * sq_size, e7_y + 0.5 * sq_size),
    }
    piece_poses['light']['knight'] = {
        0: (e7_x + 7.125 * sq_size, e7_y + 5.25 * sq_size),
        1: (e7_x + 2.125 * sq_size, e7_y + 5.25 * sq_size),
    }

    # Add rooks
    piece_poses['dark']['rook'] = {
        0: (e7_x + 9 * sq_size, e7_y + 0.5 * sq_size),
        1: (e7_x + 2 * sq_size, e7_y + 0.5 * sq_size)
    }
    piece_poses['light']['rook'] = {
        0: (e7_x + 9 * sq_size, e7_y + 5.25 * sq_size),
        1: (e7_x + 2 * sq_size, e7_y + 5.25 * sq_size)
    }

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
    
    # Create scenario
    scenario_string = base_scenario_string.format(
        FLOOR_PATH=floor_path,
        TABLE_PATH=table_path,
        CHESSBOARD_PATH=chessboard_path,
        PIECES='\n    '.join(piece_strs),
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

def setup_simulation():
    # Setup meshcat for visualization
    meshcat = StartMeshcat()
    print('Click the link above to open Meshcat in your browser!')

    # Get scenario
    scenario_string = get_scenario()

    # Load the scenario and build the simulation station
    scenario = LoadScenario(data=scenario_string)
    station = MakeHardwareStation(scenario, meshcat=meshcat)

    # Build a simple Drake Diagram containing the station
    builder = DiagramBuilder()
    builder.AddSystem(station)
    diagram = builder.Build()

    # Create and run a simulator
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(0.1)

    return diagram, simulator

if __name__ == '__main__':
    create_scenario()