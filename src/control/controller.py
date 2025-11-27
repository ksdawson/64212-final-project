import chess
import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw, RotationMatrix
from perception.perception import perception
from perception.point_cloud import get_oriented_piece_model_pcs
from game.utils import Game

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
        self.time = 0.1

        # Get piece model point clouds
        self.oriented_model_piece_point_clouds = get_oriented_piece_model_pcs()

        # Create game
        self.game = Game()

    def get_piece_poses(self):
        # Run perception pipeline
        perception_result = perception(self.diagram, self.context, self.oriented_model_piece_point_clouds)
        piece_poses = perception_result['poses']

        # Reformat result
        formatted_piece_poses = {}
        for color in piece_poses:
            for piece in piece_poses[color]:
                name = 'n' if piece.lower().startswith('knight') else piece[0]
                name = name.upper() if color == 'light' else name
                if name not in formatted_piece_poses:
                    formatted_piece_poses[name] = []
                formatted_piece_poses[name].append(piece_poses[color][piece])

        return formatted_piece_poses
    
    def set_iiwa_next_traj(self, iiwa_index, goal=None, poses=None):
        if iiwa_index == 1:
            self.iiwa1_traj_controller.NextTrajectory(goal, poses)
        elif iiwa_index == 2:
            self.iiwa2_traj_controller.NextTrajectory(goal, poses)
        else:
            raise Exception('iiwa index must be 1 or 2')
        
    def control_loop(self, simulator):
        # Called once every simulation step
        
        # Run perception pipeline
        piece_poses = self.get_piece_poses()

        # Check board state
        self.game.check_game_state(piece_poses)

        # Get a move
        move = self.game.get_move()

        # Handle move
        if self.game.is_capture_move(move):
            # TODO: Remove piece first
            # For now just end
            return
        
        # Get start pose
        from_square = move.from_square
        from_square_name = chess.square_name(from_square)
        piece = self.game.get_piece_at(from_square)
        # start_poses = piece_poses[piece]
        X_WG_pick = self.game.square_to_pose(from_square_name)

        # Get goal pose
        to_square = move.to_square
        to_square_name = chess.square_name(to_square)
        X_WG_place = self.game.square_to_pose(to_square_name)

        # Get which iiwa to move
        iiwa_instance = self.game.get_turn()
        iiwa_traj_controller = self.iiwa1_traj_controller if iiwa_instance == 1 else self.iiwa2_traj_controller
        iiwa_grasp_controller = self.iiwa1_grasp_controller if iiwa_instance == 1 else self.iiwa2_grasp_controller

        # Move to pick pose through a pre-pick pose
        rpy_down = RollPitchYaw(-np.pi/2, 0, 0)
        xyz = X_WG_pick.translation()
        X_WG_prepick = RigidTransform(RotationMatrix(rpy_down), [xyz[0], xyz[1], xyz[2] + 0.175]) # offset is 0.1, max piece height is 0.075

        # Grasp: pick piece up at midpoint for better stability
        X_WG_pick.set_rotation(RotationMatrix(rpy_down))
        X_WG_pick.set_translation([xyz[0], xyz[1], xyz[2] + 0.1 + 0.025]) # offset is 0.1, midpoint is 0.025
        # TODO: play w/ midpoint

        # Open gripper
        iiwa_grasp_controller.SetGripper(0.025)

        # Advance simulator
        self.time += 1.0
        simulator.AdvanceTo(self.time)

        # Move to pre-pick -> pick
        X_WStart = iiwa_traj_controller.get_current_pose()
        iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick, X_WG_pick], traj_t=5.0)

        # Advance simulator
        self.time += 5.0
        simulator.AdvanceTo(self.time)

        # Close gripper
        iiwa_grasp_controller.SetGripper(0.0) # max force

        # Advance simulator
        self.time += 1.0
        simulator.AdvanceTo(self.time)

        # Move to pre-pick -> place
        X_WStart = iiwa_traj_controller.get_current_pose()
        # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick, X_WG_place], traj_t=5.0)
        iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick], traj_t=5.0)

        # Advance simulator
        self.time += 5.0
        simulator.AdvanceTo(self.time)

        print(X_WG_prepick)
        print(X_WG_pick)
        print(X_WG_place)

        #
        X_WStart = iiwa_traj_controller.get_current_pose()
        iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_place], traj_t=5.0)
        self.time += 5.0
        simulator.AdvanceTo(self.time)