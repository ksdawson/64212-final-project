from pydrake.all import LeafSystem, BasicVector

class GraspController(LeafSystem):
    def __init__(self, plant, num_joints=1):
        super().__init__()
        self._num_joints = num_joints
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()

        # Output: desired joint positions
        self.DeclareVectorOutputPort(
            'q_desired', 
            BasicVector(num_joints), 
            self.CalcOutput
        )

    def CalcOutput(self, context, output):
        plant_q = self._plant.GetPositions(self._plant_context)
        wsg_q = plant_q[8:9]
        output.SetFromVector(wsg_q)