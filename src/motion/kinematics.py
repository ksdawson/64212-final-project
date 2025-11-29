import numpy as np
from pydrake.all import (
    RotationMatrix,
    Solve,
    PositionConstraint,
    OrientationConstraint,
    AngleBetweenVectorsConstraint
)
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.trajectories import PiecewisePolynomial

def reverse_traj(traj):
    # Original time span
    t_start = traj.start_time()
    t_end = traj.end_time()
    
    # Sample the trajectory at the original knot times
    t_knots = traj.get_segment_times()
    q_knots = np.column_stack([traj.value(t) for t in t_knots])
    
    # Reverse the knot order
    q_knots_rev = q_knots[:, ::-1]
    
    # Create new times that go forward but map reversed poses
    t_new = np.linspace(t_start, t_end, len(t_knots))
    
    # Build new trajectory
    traj_rev = PiecewisePolynomial.CubicShapePreserving(t_new, q_knots_rev)
    return traj_rev

def trajectory(q_knots, t = 5):
    # Spline: piecewise polynomial function
    # Knot: points where piecewise polynomial curves join together
    # This creates a time-parameterized spline in configuration space of our trajectory

    # x = f(t)
    x_lst = q_knots.T
    t_lst = np.linspace(0, t, len(q_knots)) # pose samples over t seconds

    # Build a 1D cubic spline
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, x_lst)

    return q_traj

def inverse_kinematics(plant, plant_context, pose, q_nominal=None, pos_tol=0.001, rot_tol=0.01):
    # Get frames
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName('body')

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

    # Solve the IK OP
    result = Solve(prog)
    assert result.is_success(), 'KTO solve failed'
    return result.GetSolution(q_variables)

def inverse_kinematics_axis(plant, plant_context, pose, gripper_axis, world_axis, q_nominal=None, pos_tol=0.001, rot_tol=0.01):
    # Get frames
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName('body')

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

    # Solve the IK OP
    result = Solve(prog)
    assert result.is_success(), 'KTO solve failed'
    return result.GetSolution(q_variables)

def kinematic_traj_op_per_pose(plant, plant_context, pose_lst, orientation_config, traj_t):
    # Nominal joint angles for joint-centering
    q_nominal = plant.GetPositions(plant_context)

    # Solve IK OP for each pose in the trajectory
    # Feed sol pose into next IK OP as nominal sol
    q_knots = [q_nominal] # Include start
    for pose in pose_lst:
        # Run IK
        if orientation_config is None or orientation_config['type'] == 'full':
            result = inverse_kinematics(plant, plant_context, pose, q_nominal=q_nominal)
        elif orientation_config['type'] == 'axis':
            result = inverse_kinematics_axis(plant, plant_context, pose, orientation_config['gripper_axis'], orientation_config['world_axis'], q_nominal=q_nominal)
        else:
            raise Exception('Orientation type is full or axis alignment')
        q_knots.append(result)

        # Set q nominal for the next IK
        q_nominal = result
    
    # Construct the traj
    q_knots = np.array(q_knots)

    # Exclude gripper joints
    q_knots_iiwa = q_knots[:, 0:7]

    # Build trajectory from joint positions
    q_traj = trajectory(q_knots_iiwa, t=traj_t)
    return q_traj

def kinematic_traj_op(plant, plant_context, poses, times, traj_t, pos_tol=0.001, rot_tol=0.01):
    # Create a KTO
    num_q = plant.num_positions()
    trajopt = KinematicTrajectoryOptimization(num_q, 15)
    prog = trajopt.get_mutable_prog()

    # Get frames
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName('body')

    # Create position/orientation constraints at the poses
    for s, pose in zip(times, poses):
        pos_const = PositionConstraint(
            plant,
            world_frame,
            pose.translation() - np.full(3, pos_tol),
            pose.translation() + np.full(3, pos_tol),
            gripper_frame,
            [0, 0, 0], # gripper origin
            plant_context
        )
        orient_const = OrientationConstraint(
            plant,
            world_frame,
            RotationMatrix(),
            gripper_frame,
            pose.rotation(),
            rot_tol*np.pi,
            plant_context
        )
        # orient_const = AngleBetweenVectorsConstraint(
        #     plant,
        #     gripper_frame,
        #     [0, 1, 0],
        #     world_frame,
        #     [0, 0, -1],
        #     0.0,
        #     rot_tol*np.pi,
        #     plant_context
        # )

        # Add constraints as path position constraints
        trajopt.AddPathPositionConstraint(pos_const, s=s)
        trajopt.AddPathPositionConstraint(orient_const, s=s)

    # Add position and velocity bounds to ensure the traj sol respects the hardware's limits
    pos_lower, pos_upper = plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
    vel_lower, vel_upper = plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()
    pos_lower[7:], pos_upper[7:], vel_lower[7:], vel_upper[7:] = 0.0, 0.0, 0.0, 0.0 # exclude gripper
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

    # Exclude gripper joints
    q_knots_iiwa = q_knots[:, 0:7]

    # Build trajectory from joint positions
    q_traj = trajectory(q_knots_iiwa, t=traj_t)
    return q_traj