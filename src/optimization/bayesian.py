import numpy as np
import time
from bayes_opt import BayesianOptimization
from bayes_opt.util import UtilityFunction
from pydrake.all import (
    DiagramBuilder, RollPitchYaw, RotationMatrix, RigidTransform
)
from manipulation.station import LoadScenario, MakeHardwareStation
from motion.kinematics import inverse_kinematics
from game.utils import Game

# Global vars
BASE_SCENARIO_STR = '''
directives:
    - add_model:
        name: iiwa1
        file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
        default_joint_positions:
            iiwa_joint_1: [{J1}]
            iiwa_joint_2: [{J2}]
            iiwa_joint_3: [{J3}]
            iiwa_joint_4: [{J4}]
            iiwa_joint_5: [{J5}]
            iiwa_joint_6: [{J6}]
            iiwa_joint_7: [{J7}]
    - add_weld:
        parent: world
        child: iiwa1::iiwa_link_0
        X_PC:
            translation: [0, {BASE_DIST}, 0.01]
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
model_drivers:
    iiwa1: !IiwaDriver
        control_mode: position_only
        hand_model_name: wsg1
    wsg1: !SchunkWsgDriver {{}}
    default: !ZeroForceDriver {{}}
'''
PLANT = None
PLANT_CONTEXT = None

def setup_station(base_dist, j1, j2, j3, j4, j5, j6, j7):
    # Create scenario
    scenario_string = BASE_SCENARIO_STR.format(
        BASE_DIST=base_dist,
        J1=j1, J2=j2, J3=j3, J4=j4, J5=j5, J6=j6, J7=j7
    )

    # Load the scenario and build the simulation station
    scenario = LoadScenario(data=scenario_string)
    station = MakeHardwareStation(scenario)

    # Build a Drake Diagram containing the station
    builder = DiagramBuilder()
    builder.AddSystem(station)
    diagram = builder.Build()

    # Get the plant and context
    context = diagram.CreateDefaultContext()
    plant = station.plant()
    plant_context = diagram.GetSubsystemContext(plant, context)

    # Set global vars
    global PLANT, PLANT_CONTEXT
    PLANT = plant
    PLANT_CONTEXT = plant_context

    return diagram, station

def black_box_function(base_dist, j1, j2, j3, j4, j5, j6, j7):
    # Set the plant positions
    q = np.zeros(PLANT.num_positions())
    q[:7] = [j1, j2, j3, j4, j5, j6, j7]
    PLANT.SetPositions(PLANT_CONTEXT, q)
    q_nominal = q[:7]

    # Create a game
    game = Game()

    # Run IK to each square
    score = 0 # each success gets +1
    for file_idx in range(ord('a'), ord('h') + 1):
        file = chr(file_idx)
        for rank_idx in range(1, 8 + 1):
            # Construct the square
            rank = str(rank_idx)
            sq = file + rank

            # Get the base pose
            X_WG_pick = game.square_to_pose(sq)

            # Construct the pre-pick pose
            rpy_down = RotationMatrix(RollPitchYaw(-np.pi/2, 0, 0)) # gripper pointing down
            pick_xyz = X_WG_pick.translation()
            # X_WG_prepick = RigidTransform(rpy_down, [pick_xyz[0], pick_xyz[1] - base_dist, pick_xyz[2] + 0.175]) # offset to gripper origin is 0.1, max piece height is 0.075

            X_offset = RigidTransform([0, -base_dist, 0])
            X_WG_prepick = X_offset @ RigidTransform(rpy_down, pick_xyz + np.array([0, 0, 0.175])) # offset to gripper origin is 0.1, max piece height is 0.075

            # Run IK
            try:
                result = inverse_kinematics(PLANT, PLANT_CONTEXT, X_WG_prepick, q_nominal=q_nominal)
                score += 1
            except:
                continue

    # Normalize score
    score /= 64

    return score

def bayesian_optimization(n_iter=50):
    # Bounded region of parameter space
    pbounds = {
        'base_dist': (-1.0, -0.6),
        'j1': (-2.96706, 2.96706),
        'j2': (-2.0944, 2.0944),
        'j3': (-2.96706, 2.96706),
        'j4': (-2.0944, 2.0944),
        'j5': (-2.96706, 2.96706),
        'j6': (-2.0944, 2.0944),
        'j7': (-3.05433, 3.05433)
    }

    # Create optimizer
    optimizer = BayesianOptimization(
        f=black_box_function,
        pbounds=pbounds,
        random_state=42,
        verbose=0 # minimize printing to speed up
    )

    # Improve GP numerical stability
    optimizer.set_gp_params(
        alpha=1e-4, # jitter for noisy function
        normalize_y=True # normalize outputs
    )

    # Run BO
    utility = UtilityFunction(kind='ei', xi=0.01) # better for expensive noisy functions
    optimizer.maximize(
        init_points=15,
        n_iter=n_iter,
        acquisition_function=utility
    )

    # Get result
    result = optimizer.max
    return result

if __name__ == '__main__':
    # Get the bounds of the iiwa
    diagram, station = setup_station(0, 0, 0, 0, 0, 0, 0, 0)
    # plant = station.plant()
    # lower = plant.GetPositionLowerLimits()
    # upper = plant.GetPositionUpperLimits()
    # print(lower, upper)
    
    # Run bayesian optimization to find best starting configuration
    start = time.time()
    result = bayesian_optimization(n_iter=3)
    end = time.time()

    # Print results
    duration = round(end - start, 3)
    print(f'Bayesian optimization finished in {duration} s')
    score = round(result['target'] * 64)
    print(f'Score: {score}')
    params = {k:round(float(v),4) for k,v in result['params'].items()}
    print('Params: ', params)