import numpy as np
from pydrake.all import LeafSystem, BasicVector
from pydrake.trajectories import PiecewisePolynomial
from motion.kinematics import kinematic_traj_op

def trajectory(q_knots, t = 10):
    # Spline: piecewise polynomial function
    # Knot: points where piecewise polynomial curves join together
    # This creates a time-parameterized spline in configuration space of our trajectory

    # x = f(t)
    x_lst = q_knots.T
    t_lst = np.linspace(0, t, len(q_knots)) # pose samples over t seconds

    # Build a 1D cubic spline
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, x_lst)

    return q_traj

class TrajectoryController(LeafSystem):
    def __init__(self, plant, num_joints=7):
        super().__init__()
        self._traj = None
        self._num_joints = num_joints
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()

        # Output: desired joint positions
        self.DeclareVectorOutputPort(
            'q_desired', 
            BasicVector(self._num_joints), 
            self.CalcOutput
        )

    def SetTrajectory(self, traj):
        self._traj = traj

    def NextTrajctory(self, poses=None):
        # If no poses then hold at pose
        if poses is None:
            self.SetTrajectory(None)
        assert len(poses) >= 2, 'Need at least 2 poses'

        # Get iiwa joint positions interpolated along the trajectory
        q_knots = kinematic_traj_op(self._plant, self._plant_context, poses)

        # Exclude gripper joints
        q_knots_iiwa = q_knots[:, 0:7]

        # Build trajectory from joint positions
        q_traj = trajectory(q_knots_iiwa)

        # Command the traj to the controller
        self.SetTrajectory(q_traj)

    def CalcOutput(self, context, output):
        if self._traj is None:
            # No trajectory then hold zeros
            output.SetFromVector([0.0] * self._num_joints)
            return

        # Clamp to end time (i.e. hold final configuration)
        t = context.get_time()
        t_eval = min(t, self._traj.end_time())

        # Command joint positions at this time step
        q = self._traj.value(t_eval).reshape(-1)
        output.SetFromVector(q)