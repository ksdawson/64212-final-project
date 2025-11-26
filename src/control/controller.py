from perception.perception import perception
from perception.point_cloud import get_oriented_piece_model_pcs

class Controller:
    def __init__(self, diagram, context,
                 iiwa1_traj_controller, iiwa2_traj_controller,
                 iiwa1_grasp_controller, iiwa2_grasp_controller):
        # Store simulation info
        self.diagram = diagram
        self.context = context
        self.iiwa1_traj_controller = iiwa1_traj_controller
        self.iiwa2_traj_controller = iiwa2_traj_controller
        self.iiwa1_grasp_controller = iiwa1_grasp_controller
        self.iiwa2_grasp_controller = iiwa2_grasp_controller

        # Get piece model point clouds
        self.oriented_model_piece_point_clouds = get_oriented_piece_model_pcs()

    def get_piece_poses(self):
        perception_result = perception(self.diagram, self.context, self.oriented_model_piece_point_clouds)
        piece_poses = perception_result['poses']
        return piece_poses
    
    def set_iiwa_next_traj(self, iiwa_index, goal=None, poses=None):
        if iiwa_index == 1:
            self.iiwa1_traj_controller.NextTrajectory(goal, poses)
        elif iiwa_index == 2:
            self.iiwa2_traj_controller.NextTrajectory(goal, poses)
        else:
            raise Exception('iiwa index must be 1 or 2')
        
    def control_loop(self):
        # Called once every simulation step
        # Returns the time to advance to
        
        # Run perception pipeline
        piece_poses = self.get_piece_poses()

        