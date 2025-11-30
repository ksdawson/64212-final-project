import numpy as np
from pydrake.all import (
    JacobianWrtVariable
)
from optimization.bayesian import setup_station

def manipulability_index(plant, context, q):
    # Uses Yoshikawa's manipuability index which is based on the volume of the manipuability ellipsoid

    # Add gripper joints if needed
    if plant.num_positions() > len(q):
        q = np.hstack((q, np.zeros(plant.num_positions() - len(q))))

    # Set the configuration
    plant.SetPositions(context, q)

    # Compute spatial velocity Jacobian at the end-effector
    ee_frame = plant.GetFrameByName('body')
    J = plant.CalcJacobianSpatialVelocity(
        context,
        with_respect_to=JacobianWrtVariable.kQDot,
        frame_B=ee_frame,
        p_BoBp_B=np.zeros((3,1)),
        frame_A=plant.world_frame(),
        frame_E=plant.world_frame()
    )

    # Yoshikawa manipulability: w = sqrt(det(J*J^T))
    JJT = J @ J.T
    manipulability_index = np.sqrt(np.linalg.det(JJT))

    return manipulability_index

def test(config):
    # Setup
    q = np.array([config['j1'], config['j2'], config['j3'], config['j4'], config['j5'], config['j6'], config['j7']])
    diagram, station = setup_station(config['base_dist'], q[0], q[1], q[2], q[3], q[4], q[5], q[6])
    context = diagram.CreateDefaultContext()
    plant = station.plant()
    plant_context = diagram.GetSubsystemContext(plant, context)
    
    # Compute index
    idx = manipulability_index(plant, plant_context, q)
    return idx

if __name__ == '__main__':
    test1 = {'base_dist': -0.5916, 'j1': -1.4877, 'j2': -0.3754, 'j3': 1.5165, 'j4': -1.136, 'j5': -2.5103, 'j6': -0.8807, 'j7': -2.0695}
    test2 = {'base_dist': -0.5727, 'j1': -1.7201, 'j2': -1.4555, 'j3': -1.8716, 'j4': 1.1987, 'j5': 1.0198, 'j6': -1.139, 'j7': -0.2204}
    test3 = {'base_dist': -0.55, 'j1': -1.7237, 'j2': -0.1377, 'j3': 2.0056, 'j4': -1.8408, 'j5': -2.8261, 'j6': -0.7136, 'j7': -1.7702}
    idx1 = test(test1)
    idx2 = test(test2)
    idx3 = test(test3)
    print(idx1, idx2, idx3)