import numpy as np
from pydrake.all import (
    RotationMatrix,
    Solve
)
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.trajectories import PiecewisePolynomial

def trajectory(q_knots, t = 5):
    # Spline: piecewise polynomial function
    # Knot: points where piecewise polynomial curves join together
    # This creates a time-parameterized spline in configuration space of our trajectory

    # x = f(t)
    x_lst = q_knots.T
    t_lst = np.linspace(0, t, len(q_knots)) # equal time spacing

    # Build a 1D cubic spline
    if len(q_knots) >= 3:
        q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, x_lst)
    elif len(q_knots) == 2:
        q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, x_lst, zero_end_point_derivatives=True)
    else:
        raise Exception('Need at least 2 knots for trajectory')

    return q_traj

def inverse_kinematics(iiwa_instance, plant, plant_context, pose, q_nominal=None, pos_tol=0.001, rot_tol=0.01):
    # Get frames
    world_frame = plant.world_frame()
    gripper_model = plant.GetModelInstanceByName(f'wsg{iiwa_instance}')
    gripper_frame = plant.GetFrameByName('body', gripper_model)

    # Setup IK
    ik = InverseKinematics(plant, plant_context)
    q_variables = ik.q()
    prog = ik.prog()

    # Set prog cost and initial guess
    if q_nominal is not None:
        prog.AddQuadraticErrorCost(np.identity(len(q_variables)), q_nominal, q_variables)
        prog.SetInitialGuess(q_variables, q_nominal)

    # Set IK constraints
    ik.AddOrientationConstraint(
        gripper_frame,
        RotationMatrix(),
        world_frame,
        pose.rotation(),
        rot_tol*np.pi
    )
    ik.AddPositionConstraint(
        gripper_frame,
        [0, 0, 0],
        world_frame,
        pose.translation() - np.full(3, pos_tol),
        pose.translation() + np.full(3, pos_tol)
    )
    ik.AddMinimumDistanceLowerBoundConstraint(0.001, 0.01)

    # Solve the IK OP
    result = Solve(prog)
    assert result.is_success(), 'KTO solve failed'
    sol = result.GetSolution(q_variables)

    return sol

def inverse_kinematics_axis(iiwa_instance, plant, plant_context, pose, gripper_axis, world_axis, q_nominal=None, pos_tol=0.001, rot_tol=0.01):
    # Get frames
    world_frame = plant.world_frame()
    gripper_model = plant.GetModelInstanceByName(f'wsg{iiwa_instance}')
    gripper_frame = plant.GetFrameByName('body', gripper_model)

    # Setup IK
    ik = InverseKinematics(plant, plant_context)
    q_variables = ik.q()
    prog = ik.prog()

    # Set prog cost and initial guess
    if q_nominal is not None:
        prog.AddQuadraticErrorCost(np.identity(len(q_variables)), q_nominal, q_variables)
        prog.SetInitialGuess(q_variables, q_nominal)

    # Set IK constraints
    ik.AddAngleBetweenVectorsConstraint(
        gripper_frame,
        gripper_axis,
        world_frame,
        world_axis,
        0.0, # min angle (0 = perfectly aligned)
        rot_tol*np.pi # max angle
    )
    ik.AddPositionConstraint(
        gripper_frame,
        [0, 0, 0],
        world_frame,
        pose.translation() - np.full(3, pos_tol),
        pose.translation() + np.full(3, pos_tol)
    )
    ik.AddMinimumDistanceLowerBoundConstraint(0.001, 0.01)

    # Solve the IK OP
    result = Solve(prog)
    assert result.is_success(), 'KTO solve failed'
    return result.GetSolution(q_variables)

def kinematic_traj_op_per_pose(iiwa_instance, plant, plant_context, pose_lst, orientation_config=None, traj_t=5.0, knots_only=False):
    # Nominal joint angles for joint-centering
    q_nominal = plant.GetPositions(plant_context)

    # Solve IK OP for each pose in the trajectory
    # Feed sol pose into next IK OP as nominal sol
    q_knots = [q_nominal] # Include start
    for pose in pose_lst:
        # Run IK
        if orientation_config is None or orientation_config['type'] == 'full':
            result = inverse_kinematics(iiwa_instance, plant, plant_context, pose, q_nominal=q_nominal)
        elif orientation_config['type'] == 'axis':
            result = inverse_kinematics_axis(iiwa_instance, plant, plant_context, pose, orientation_config['gripper_axis'], orientation_config['world_axis'], q_nominal=q_nominal)
        else:
            raise Exception('Orientation type is full or axis alignment')
        q_knots.append(result)

        # Set q nominal for the next IK
        q_nominal = result
    
    # Construct the traj
    q_knots = np.array(q_knots)

    # Exclude gripper joints and other iiwa
    if q_knots.shape[1] > 9:
        # two iiwas
        q_knots_iiwa = q_knots[:, 0:7] if iiwa_instance == 1 else q_knots[:, 9:16]
    else:
        # one iiwa
        q_knots_iiwa = q_knots[:, 0:7]
    if knots_only:
        return q_knots_iiwa

    # Build trajectory from joint positions
    q_traj = trajectory(q_knots_iiwa, t=traj_t)
    return q_traj

def kinematic_traj_op(iiwa_instance, plant, plant_context, poses, times, traj_t=5.0, pos_tol=0.001, rot_tol=0.01, knots_only=False):
    # Create a KTO
    num_q = plant.num_positions()
    trajopt = KinematicTrajectoryOptimization(num_q, 15)
    prog = trajopt.get_mutable_prog()

    # Create position/orientation constraints at the poses
    q_nominal = plant.GetPositions(plant_context)
    for s, pose in zip(times, poses):
        # Solve IK for joint configuration at time s
        q = inverse_kinematics(iiwa_instance, plant, plant_context, pose, q_nominal=q_nominal, pos_tol=pos_tol, rot_tol=rot_tol)

        # Add constraints as path position constraints
        trajopt.AddPathPositionConstraint(lb=q, ub=q, s=s)

        # Set q nominal for the next IK
        q_nominal = q

    # Add position and velocity bounds to ensure the traj sol respects the hardware's limits
    pos_lower, pos_upper = plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
    vel_lower, vel_upper = plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()
    if q_nominal.shape[0] > 9:
        # two iiwas
        pos_lower[7:9], pos_upper[7:9], vel_lower[7:9], vel_upper[7:9] = 0.0, 0.0, 0.0, 0.0 # exclude gripper
        pos_lower[16:], pos_upper[16:], vel_lower[16:], vel_upper[16:] = 0.0, 0.0, 0.0, 0.0
    else:
        # one iiwa
        pos_lower[7:], pos_upper[7:], vel_lower[7:], vel_upper[7:] = 0.0, 0.0, 0.0, 0.0
    trajopt.AddPositionBounds(pos_lower, pos_upper)
    trajopt.AddVelocityBounds(vel_lower, vel_upper)

    # Add a path velocity constraint at the beginning and end of the traj
    # This ensures we begin the traj stationary and end it stationary
    trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), s=0.0)
    trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), s=1.0)

    # Constrain the duration to last between 0.5 and 5 seconds
    trajopt.AddDurationConstraint(0.5, 5)

    # Add a cost on the duration and path length with magnitude 1.0
    trajopt.AddDurationCost(1.0)
    trajopt.AddPathLengthCost(1.0)

    # Add a cost that encourages the sol to be close to the Iiwa's starting joint state
    # at beginning and end of the traj -- "normalizes" sol behavior
    q0 = plant.GetPositions(plant_context)
    prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 0])
    prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, -1])

    # Solve the OP
    result = Solve(prog)
    assert result.is_success(), 'KTO solve failed'
    traj = trajopt.ReconstructTrajectory(result)

    # Construct the traj
    q_knots = np.array(traj.control_points())
    q_knots = np.squeeze(q_knots, axis=2)

    # Exclude gripper joints and other iiwa
    if q_knots.shape[1] > 9:
        # two iiwas
        q_knots_iiwa = q_knots[:, 0:7] if iiwa_instance == 1 else q_knots[:, 9:16]
    else:
        # one iiwa
        q_knots_iiwa = q_knots[:, 0:7]
    if knots_only:
        return q_knots_iiwa

    # Build trajectory from joint positions
    q_traj = trajectory(q_knots_iiwa, t=traj_t)
    return q_traj