import os
from pydrake.all import (
    DiagramBuilder, StartMeshcat, Simulator, MeshcatVisualizer
)
from manipulation.station import LoadScenario, MakeHardwareStation

def create_scenario():
    # Get paths to assets
    current_directory = os.getcwd()
    chess_assets_directory = 'src/assets/chess'
    furniture_assets_directory = 'src/assets/furniture'

    # Get table
    table_path = f'{current_directory}/{furniture_assets_directory}/table1/model.sdf'

    # Get chessboard
    chessboard_path = f'{current_directory}/{chess_assets_directory}/chessboard/model.sdf'

    # Get pieces
    dark_pawn_path = f'{current_directory}/{chess_assets_directory}/pieces/individual_pieces/dark_pawn/model.sdf'

    # Create scenario
    scenario_string = f'''directives:
    - add_model:
        name: dark_pawn
        file: file://{dark_pawn_path}
        default_free_body_pose:
            link:
                translation: [-0.35, 0, 0]
                rotation: !Rpy {{ deg: [0, 0, 0] }}
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