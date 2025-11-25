import numpy as np
from pydrake.all import (
    RotationMatrix,
    Solve
)
from pydrake.multibody import inverse_kinematics

def kinematic_traj_op(plant, plant_context, pose_lst):
    # Get frames
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName('body')
    context = plant.CreateDefaultContext()

    # Nominal joint angles for joint-centering
    q_nominal = plant.GetPositions(plant_context)

    # Solve IK OP for each pose in the trajectory
    # Feed sol pose into next IK OP as nominal sol
    q_knots = [q_nominal] # Include start
    for pose in pose_lst:
        # Setup IK OP
        ik = inverse_kinematics.InverseKinematics(plant, context)
        q_variables = ik.q()
        prog = ik.prog()

        # Set prog cost and initial guess
        prog.AddQuadraticErrorCost(np.identity(len(q_variables)), q_nominal, q_variables)
        prog.SetInitialGuess(q_variables, q_nominal)

        # Set IK constraints
        ik.AddOrientationConstraint(
            gripper_frame,
            RotationMatrix(),
            world_frame,
            pose.rotation(),
            0.01*np.pi
        )
        ik.AddPositionConstraint(
            gripper_frame,
            [0, 0, 0],
            world_frame,
            pose.translation() - np.array([0.001, 0.001, 0.001]),
            pose.translation() + np.array([0.001, 0.001, 0.001])
        )

        # Solve the IK OP
        result = Solve(prog)
        assert result.is_success(), 'No solution found'
        q_knots.append(result.GetSolution(q_variables))

        # Set q nominal for the next IK
        q_nominal = result.GetSolution(q_variables)
    return np.array(q_knots)