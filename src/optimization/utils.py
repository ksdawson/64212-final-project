import numpy as np
from pydrake.all import (
    JacobianWrtVariable
)
from motion.kinematics import inverse_kinematics_axis

def is_near_singularity(plant, context, q, threshold=1e-3):
    # Add gripper joints if needed
    if plant.num_positions() > len(q):
        q = np.hstack((q, np.zeros(plant.num_positions() - len(q))))

    # Set the robot configuration
    plant.SetPositions(context, q)
    
    # Calculate Jacobian
    ee_frame = plant.GetFrameByName('body')
    J = plant.CalcJacobianSpatialVelocity(
        context,
        with_respect_to=JacobianWrtVariable.kQDot,
        frame_B=ee_frame,
        p_BoBp_B=np.zeros((3,1)),
        frame_A=plant.world_frame(),
        frame_E=plant.world_frame()
    )
    
    # Compute singular values
    s = np.linalg.svd(J, compute_uv=False)
    min_singular_value = np.min(s)
    
    return min_singular_value < threshold, min_singular_value

def find_home_configuration(plant, context, n_candidates=10, pos_tol=0.001, rot_tol=0.01):
    # Get target
    X_target = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName('body'))
    q_nominal = plant.GetPositions(context)

    # Candidate generation functions
    def gen_perturbed_candidates(plant, q_nominal):
        n_q = plant.num_positions()
        return q_nominal + np.random.randn(n_q) * 0.02  # 0.02 rad perturbation
    def gen_random_candidates(plant):
        lower = plant.GetPositionLowerLimits()
        upper = plant.GetPositionUpperLimits()
        q_nominal = np.random.uniform(low=lower, high=upper)
        return q_nominal

    # Generate candidates
    q_candidates = [q_nominal]
    for _ in range(n_candidates):
        # q_nominal = gen_perturbed_candidates(plant, q_nominal)
        q_nominal = gen_random_candidates(plant)
        try:
            # result = inverse_kinematics(plant, context, X_target, q_nominal=q_nominal, pos_tol=pos_tol, rot_tol=rot_tol)
            result = inverse_kinematics_axis(plant, context, X_target, [0, 1, 0], [0, 0, -1], q_nominal=q_nominal, pos_tol=pos_tol, rot_tol=rot_tol)
            q_candidates.append(result)
        except:
            continue
    
    # Score them based on how far from singularity they are
    scores = []
    for q in q_candidates:
        _, min_sv = is_near_singularity(plant, context, q[:7])
        scores.append(min_sv)

    # Choose the best one
    best_idx = np.argmax(scores)
    q_home = q_candidates[best_idx]

    # Get relative improvement over base configuration
    relative_perf = scores[best_idx] / scores[0]

    return q_home, relative_perf