import numpy as np
import pickle
import time
from pydrake.all import (
    DiagramBuilder, RollPitchYaw, RotationMatrix, RigidTransform
)
from manipulation.station import LoadScenario, MakeHardwareStation
from motion.kinematics import kinematic_traj_op_per_pose, kinematic_traj_op
from game.utils import Game

BASE_SCENARIO_STR = '''
directives:
    - add_model:
        name: floor
        file: file:///workspaces/code/src/assets/room/floor.sdf
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
        file: file:///workspaces/code/src/assets/furniture/table1/model.sdf
    - add_weld:
        parent: world
        child: table::link
        X_PC:
            translation: [0.0, 0.0, -0.087279]
    - add_model:
        name: chessboard
        file: file:///workspaces/code/src/assets/chess/chessboard/model.sdf
    - add_weld:
        parent: table::link
        child: chessboard::link
        X_PC:
            translation: [0.0, 0.0, 0.478391]
'''

def setup_station(iiwa1_config, iiwa2_config):
    # Create scenario
    scenario_string = BASE_SCENARIO_STR.format(
        IIWA1_BASE_DIST=iiwa1_config['base_dist'], IIWA2_BASE_DIST=iiwa2_config['base_dist'],
        IIWA1_J1=iiwa1_config['j1'], IIWA1_J2=iiwa1_config['j2'], IIWA1_J3=iiwa1_config['j3'], IIWA1_J4=iiwa1_config['j4'], IIWA1_J5=iiwa1_config['j5'], IIWA1_J6=iiwa1_config['j6'], IIWA1_J7=iiwa1_config['j7'],
        IIWA2_J1=iiwa2_config['j1'], IIWA2_J2=iiwa2_config['j2'], IIWA2_J3=iiwa2_config['j3'], IIWA2_J4=iiwa2_config['j4'], IIWA2_J5=iiwa2_config['j5'], IIWA2_J6=iiwa2_config['j6'], IIWA2_J7=iiwa2_config['j7']
    )

    # Load the scenario and build the simulation station
    scenario = LoadScenario(data=scenario_string)
    station = MakeHardwareStation(scenario)

    # Build a Drake Diagram containing the station
    builder = DiagramBuilder()
    builder.AddSystem(station)
    diagram = builder.Build()

    return diagram, station

def get_squares():
    for file_idx in range(ord('a'), ord('h') + 1):
        file = chr(file_idx)
        for rank_idx in range(1, 8 + 1):
            # Construct the square
            rank = str(rank_idx)
            sq = file + rank
            yield sq

def create_traj_db(iiwa1_config, iiwa2_config):
    # Offline prebuilds a trajectory database for performing every chess move from a starting configuration
    
    # Setup
    diagram, station = setup_station(iiwa1_config, iiwa2_config)
    context = diagram.CreateDefaultContext()
    plant = station.plant()
    plant_context = diagram.GetSubsystemContext(plant, context)

    # Create a game
    game = Game()
    piece_heights = {
        'king': 0.07604153846153845,
        'queen': 0.06604307692307691,
        'bishop': 0.05498769230769231,
        'knight': 0.05065692307692307,
        'rook': 0.039976923076923083,
        'pawn': 0.041956
    }

    # Compute all 3 trajectories for each square for each iiwa
    moves_db = {}
    for iiwa_instance in range(1, 3):
        moves_db[iiwa_instance] = {}
        X_WG_home = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName('body', plant.GetModelInstanceByName(f'wsg{iiwa_instance}')))
        iiwa_model = plant.GetModelInstanceByName(f'iiwa{iiwa_instance}')
        original_q = plant.GetPositions(plant_context, iiwa_model)
        for src_sq in get_squares():
            # Get the base pose
            X_WG_pick_base = game.square_to_pose(src_sq)

            # Construct the poses
            rpy_down = RotationMatrix(RollPitchYaw(-np.pi/2, 0, 0)) # gripper pointing down
            pick_base_xyz = X_WG_pick_base.translation()
            X_WG_postpick = RigidTransform(rpy_down, [pick_base_xyz[0], pick_base_xyz[1], pick_base_xyz[2] + 0.1 + 2 * piece_heights['king']])
            X_WG_prepick = RigidTransform(rpy_down, [pick_base_xyz[0], pick_base_xyz[1], pick_base_xyz[2] + 0.1 + piece_heights['king']])
            X_WG_pick = RigidTransform(rpy_down, [pick_base_xyz[0], pick_base_xyz[1], pick_base_xyz[2] + 0.1]) # refine for best grasping

            # Trajectories
            moves_db[iiwa_instance][src_sq] = {'to_post_pick': None, 'from_post_pick': None, 'to_pick': None, 'from_pick': None, 'to_place': None}

            # Post-pick
            try:
                # Only get the knots bc can't pickle trajectories
                plant.SetPositions(plant_context, iiwa_model, original_q) # set to home
                to_post_pick_knots = kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, [X_WG_postpick], knots_only=True)
                moves_db[iiwa_instance][src_sq]['to_post_pick'] = to_post_pick_knots
            except Exception as e:
                print('KTO failed: ', e)
                continue # need to for the from
            try:
                plant.SetPositions(plant_context, iiwa_model, to_post_pick_knots[-1]) # set to post-pick
                from_post_pick_knots = kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, [X_WG_home], knots_only=True)
                moves_db[iiwa_instance][src_sq]['from_post_pick'] = from_post_pick_knots
            except Exception as e:
                print('KTO failed: ', e)

            # Pick
            moves_db[iiwa_instance][src_sq]['to_pick'] = {}
            moves_db[iiwa_instance][src_sq]['from_pick'] = {}
            for piece, height in piece_heights.items():
                try:
                    plant.SetPositions(plant_context, iiwa_model, to_post_pick_knots[-1]) # set to post-pick

                    # Move to piece height
                    pick_xyz = X_WG_pick.translation()
                    X_WG_piece_pick = RigidTransform(X_WG_pick.rotation(), [pick_xyz[0], pick_xyz[1], pick_xyz[2] + height - 0.005])

                    to_pick_knots = kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, [X_WG_prepick, X_WG_piece_pick], knots_only=True)
                    moves_db[iiwa_instance][src_sq]['to_pick'][piece] = to_pick_knots
                except Exception as e:
                    print('KTO failed: ', e) # need to for the from
                    continue
                try:
                    plant.SetPositions(plant_context, iiwa_model, to_pick_knots[-1]) # set to pick
                    from_pick_knots = kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, [X_WG_prepick, X_WG_postpick], knots_only=True)
                    moves_db[iiwa_instance][src_sq]['from_pick'][piece] = from_pick_knots
                except Exception as e:
                    print('KTO failed: ', e)

            # Post-pick to post-place
            moves_db[iiwa_instance][src_sq]['to_place'] = {}
            for dest_sq in get_squares():
                # Skip if the curr sq
                if dest_sq == src_sq:
                    continue

                # Construct the poses
                X_WG_place_base = game.square_to_pose(dest_sq)
                place_base_xyz = X_WG_place_base.translation()
                X_WG_postplace = RigidTransform(rpy_down, [place_base_xyz[0], place_base_xyz[1], place_base_xyz[2] + 0.1 + 2 * piece_heights['king']])

                # Build the trajectory
                try:
                    plant.SetPositions(plant_context, iiwa_model, to_post_pick_knots[-1]) # set to post-pick
                    to_post_place_knots = kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, [X_WG_postplace], knots_only=True)
                    moves_db[iiwa_instance][src_sq]['to_place'][dest_sq] = to_post_place_knots
                except Exception as e:
                    print('KTO failed: ', e)

    # Pickle the database
    file_path = 'traj_db.pkl'
    with open(file_path, 'wb') as f:
        pickle.dump(moves_db, f)

def main():
    # Get iiwa starting configurations from Bayesian Optimization
    file_path = 'starting_configuration.pkl'
    with open(file_path, 'rb') as file:
        # Load the data from the pickle file
        data = pickle.load(file)
    iiwa1_config, iiwa2_config = data[1], data[2]

    # Create traj db
    start = time.time()
    create_traj_db(iiwa1_config, iiwa2_config)
    end = time.time()
    duration = round(end - start, 3)
    print(f'Traj db construction finished in {duration} s')

if __name__ == '__main__':
    main()