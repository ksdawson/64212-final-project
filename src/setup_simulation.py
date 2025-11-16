import os
from pydrake.all import (
    DiagramBuilder, StartMeshcat, Simulator
)
from manipulation.station import LoadScenario, MakeHardwareStation

# Base string containing the IIWAs, table, and chessboard
base_scenario_string = '''
directives:
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
            translation: [0, -0.75, 0]
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
            translation: [0, 0.75, 0]
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
            translation: [0.0, 0.0, 0.012721]
    - add_model:
        name: chessboard
        file: file://{CHESSBOARD_PATH}
    - add_weld:
        parent: table::link
        child: chessboard::link
        X_PC:
            translation: [0.0, 0.0, 0.4746]
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
                translation: [-0.25, -0.6, 0.517262]
                rotation: !Rpy {{ deg: [90, 0, 0] }}'''

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
    colors = ['dark', 'light']
    pieces = ['pawn', 'king', 'queen', 'bishop', 'knight', 'rook']
    piece_nums = [8, 1, 1, 2, 2, 2]
    piece_strs = []
    for color in colors:
        for piece, piece_num in zip(pieces, piece_nums):
            name = f'{color}_{piece}'
            piece_path = f'{current_directory}/{chess_assets_directory}/pieces/individual_pieces/{name}/model.sdf'
            for p in range(piece_num):
                name_copy = f'{name}_{p}'
                model_str = piece_scenario_str.format(NAME=name_copy, PATH=piece_path)
                piece_strs.append(model_str)
    
    # Create scenario
    scenario_string = base_scenario_string.format(
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