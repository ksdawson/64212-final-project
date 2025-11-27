import numpy as np
from pydrake.all import LeafSystem, BasicVector

class GraspController(LeafSystem):
    def __init__(self, plant, num_joints=1):
        super().__init__()
        self._num_joints = num_joints
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._grip = None # distance of gripper opened (0 is closed)

        # Output: desired joint positions
        self.DeclareVectorOutputPort(
            'q_desired', 
            BasicVector(num_joints), 
            self.CalcOutput
        )

    def SetGripper(self, pos=None):
        self._grip = None if pos is None else np.array([[pos]]) # shape 1, 1

    def CalcOutput(self, context, output):
        if self._grip is None:
            # Hold at current position
            plant_q = self._plant.GetPositions(self._plant_context)
            wsg_q = plant_q[8:9]
            output.SetFromVector(wsg_q)
            return
        output.SetFromVector(self._grip)