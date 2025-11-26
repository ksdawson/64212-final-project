from pydrake.all import LeafSystem, BasicVector
from motion.kinematics import kinematic_traj_op_per_pose, kinematic_traj_op

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

    def NextTrajectory(self, goal=None, poses=None):
        # If no goal/poses then hold at pose
        if goal is None and poses is None:
            self.SetTrajectory(None)

        # Get iiwa joint positions interpolated along the trajectory
        if goal:
            q_traj = kinematic_traj_op(self._plant, self._plant_context, goal)
        elif poses:
            assert len(poses) >= 2, 'Need at least 2 poses'
            q_traj = kinematic_traj_op_per_pose(self._plant, self._plant_context, poses)
        else:
            raise Exception('Neigher goal or poses specified')

        # Command the traj to the controller
        self.SetTrajectory(q_traj)

    def CalcOutput(self, context, output):
        if self._traj is None:
            # No trajectory so hold at current pos
            plant_q = self._plant.GetPositions(self._plant_context)
            iiwa_q = plant_q[0:7]
            output.SetFromVector(iiwa_q)
            return

        # Clamp to end time (i.e. hold final configuration)
        t = context.get_time()
        t_eval = min(t, self._traj.end_time())

        # Command joint positions at this time step
        q = self._traj.value(t_eval).reshape(-1)
        output.SetFromVector(q)