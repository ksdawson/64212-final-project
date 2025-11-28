import numpy as np
from pydrake.all import LeafSystem, BasicVector

class GraspController(LeafSystem):
    def __init__(self, plant):
        super().__init__()
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._grip = None # distance of gripper opened (0 is closed)

        # Input: current joint positions
        self.q_port = self.DeclareVectorInputPort('q_actual', BasicVector(2))

        # Output: desired joint positions
        self.DeclareVectorOutputPort('q_desired', BasicVector(1), self.CalcOutput)

    def SetGripper(self, pos=None):
        self._grip = None if pos is None else np.array([[pos]]) # shape 1, 1

    def CalcOutput(self, context, output):
        # Get current positions
        wsg_q = self.q_port.Eval(context)
        plant_q = np.hstack((np.zeros(self._plant.num_positions() - len(wsg_q)), wsg_q)) # pad iiwa joints w/ 0

        # Update the internal plant context
        self._plant.SetPositions(self._plant_context, plant_q)

        # If no grip hold at current pos
        if self._grip is None:
            wsg_q = wsg_q[1:2]
            if wsg_q[0] != 0.0:
                # Hold at 0
                wsg_q[0] = 0.0
            output.SetFromVector(wsg_q)
            return
        
        # Output grip
        output.SetFromVector(self._grip)