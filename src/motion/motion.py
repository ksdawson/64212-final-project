import numpy as np
from pydrake.all import LeafSystem, BasicVector
from motion.kinematics import kinematic_traj_op_per_pose, kinematic_traj_op

class TrajectoryController(LeafSystem):
    def __init__(self, plant):
        super().__init__()
        self._traj = None
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()

        # Input: current joint positions
        self.q_port = self.DeclareVectorInputPort('q_actual', BasicVector(7))

        # Output: desired joint positions
        self.DeclareVectorOutputPort('q_desired', BasicVector(7), self.CalcOutput)

    def get_current_pose(self):
        X_WStart = self._plant.EvalBodyPoseInWorld(self._plant_context, self._plant.GetBodyByName('body'))
        return X_WStart

    def SetTrajectory(self, traj):
        self._traj = traj

    def NextTrajectory(self, poses=None, times=None, orientation_config=None, traj_t=5.0):
        # If no poses then hold at pose
        if poses is None:
            self.SetTrajectory(None)

        # Get iiwa joint positions interpolated along the trajectory
        if poses:
            assert len(poses) >= 2, 'Need at least 2 poses'
            q_traj = kinematic_traj_op_per_pose(self._plant, self._plant_context, poses, orientation_config, traj_t)
            # q_traj = kinematic_traj_op(self._plant, self._plant_context, poses, times, traj_t)
        else:
            raise Exception('Poses unspecified')

        # Command the traj to the controller
        self.SetTrajectory(q_traj)

    def CalcOutput(self, context, output):
        # Get current positions
        iiwa_q = self.q_port.Eval(context)
        plant_q = np.hstack((iiwa_q, np.zeros(self._plant.num_positions() - len(iiwa_q)))) # pad gripper joints w/ 0

        # Update the internal plant context
        self._plant.SetPositions(self._plant_context, plant_q)

        # If no trajectory hold at current pos
        if self._traj is None:
            output.SetFromVector(iiwa_q)
            return

        # Clamp to end time (i.e. hold final configuration)
        t = context.get_time()
        t_eval = min(t, self._traj.end_time())

        # Command joint positions at this time step
        q = self._traj.value(t_eval).reshape(-1)
        output.SetFromVector(q)