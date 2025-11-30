import numpy as np
import pickle
import time
from pydrake.all import (
    DiagramBuilder, RollPitchYaw, RotationMatrix, RigidTransform
)
from manipulation.station import LoadScenario, MakeHardwareStation
from motion.kinematics import kinematic_traj_op_per_pose
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
            translation: [0, {IIWA1_BASE_DIST}, 0.23]
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
            translation: [0, {IIWA2_BASE_DIST}, 0.23]
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

def create_traj_db(iiwa1_config, iiwa2_config):
    # Offline prebuilds a trajectory database for performing every chess move from a starting configuration
    
    # Setup
    diagram, station = setup_station(iiwa1_config, iiwa2_config)
    context = diagram.CreateDefaultContext()
    plant = station.plant()
    plant_context = diagram.GetSubsystemContext(plant, context)

    # Create a game
    game = Game()

    # Compute all 3 trajectories for each square for each iiwa
    moves_db = {}
    for iiwa_instance in range(1, 3):
        moves_db[iiwa_instance] = {}
        for file_idx in range(ord('a'), ord('h') + 1):
            file = chr(file_idx)
            for rank_idx in range(1, 8 + 1):
                # Construct the square
                rank = str(rank_idx)
                sq = file + rank

                # Get the base pose
                X_WG_base = game.square_to_pose(sq)

                # Construct the poses
                rpy_down = RotationMatrix(RollPitchYaw(-np.pi/2, 0, 0)) # gripper pointing down
                base_xyz = X_WG_base.translation()
                X_WG_postpick = RigidTransform(rpy_down, [base_xyz[0], base_xyz[1], base_xyz[2] + 0.1 + 2*0.076])
                X_WG_pick = RigidTransform(rpy_down, [base_xyz[0], base_xyz[1], base_xyz[2] + 0.1 + 0.025])

                # Trajectories
                try:
                    # Only get the knots bc can't pickle trajectories
                    knots = kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, [X_WG_postpick, X_WG_pick], knots_only=True)
                    moves_db[iiwa_instance][sq] = knots
                except Exception as e:
                    print('KTO failed')

    # Pickle the database
    file_path = 'traj_db.pkl'
    with open(file_path, 'wb') as f:
        pickle.dump(moves_db, f)

def main():
    # Configs from Bayesian Optimization
    iiwa1_config = {'base_dist': -0.55, 'j1': -1.7237, 'j2': -0.1377, 'j3': 2.0056, 'j4': -1.8408, 'j5': -2.8261, 'j6': -0.7136, 'j7': -1.7702}
    iiwa2_config = {'base_dist': 0.5808, 'j1': -1.5772, 'j2': 0.8144, 'j3': -0.7459, 'j4': -1.7612, 'j5': 1.4501, 'j6': 1.6473, 'j7': 1.6579}
    
    # Create traj db
    start = time.time()
    create_traj_db(iiwa1_config, iiwa2_config)
    end = time.time()
    duration = round(end - start, 3)
    print(f'Traj db construction finished in {duration} s')

if __name__ == '__main__':
    main()