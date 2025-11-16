import os
from pydrake.all import (
    DiagramBuilder, StartMeshcat, Simulator, MeshcatVisualizer
)
from manipulation.station import LoadScenario, MakeHardwareStation

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
    dark_pawn_path = f'{current_directory}/{chess_assets_directory}/pieces/individual_pieces/dark_pawn/model.sdf'

    # - add_weld:
    # parent: table::link
    # child: chessboard::link
    # X_PC:
    #     translation: [0.0, 0.0, -0.05]
    #     rotation: !Rpy {{ deg: [0, 0, -90] }}

    # - add_model:
    # name: floor
    # file: file://{floor_path}

    # Create scenario
    scenario_string = f'''directives:
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
            rotation: !Rpy {{ deg: [90, 0, 90]}}
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
            rotation: !Rpy {{ deg: [90, 0, 90]}}
    - add_model:
        name: table
        file: file://{table_path}
    - add_weld:
        parent: world
        child: table::link
        X_PC:
            translation: [0.0, 0.0, 0.012721]
    - add_model:
        name: chessboard
        file: file://{chessboard_path}
    - add_weld:
        parent: table::link
        child: chessboard::link
        X_PC:
            translation: [0.0, 0.0, 0.4746]
    - add_model:
        name: dark_pawn
        file: file://{dark_pawn_path}
        default_free_body_pose:
            link:
                translation: [0, 0, 0]
                rotation: !Rpy {{ deg: [90, 0, 0] }}
visualization:
    publish_contacts: true
    publish_proximity: true
'''

    return scenario_string

def setup_simulation():
    # Setup meshcat for visualization
    meshcat = StartMeshcat()
    print('Click the link above to open Meshcat in your browser!')

    # Get scenario
    scenario_string = create_scenario()

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