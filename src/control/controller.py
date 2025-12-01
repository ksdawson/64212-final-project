import chess
import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw, RotationMatrix
from perception.perception import perception
from perception.point_cloud import get_oriented_piece_model_pcs
from game.utils import Game
from utils import poses_equal
from control.utils import get_trajs_from_db

class Controller:
    def __init__(self, diagram, context, simulator,
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
        self.simulator = simulator

        # Store home pose
        self.iiwa1_X_WG_home = iiwa1_traj_controller.get_current_pose()
        self.iiwa2_X_WG_home = iiwa2_traj_controller.get_current_pose()

        # Get piece model point clouds
        self.oriented_model_piece_point_clouds = get_oriented_piece_model_pcs()

        # Create game
        self.game = Game()
        self.traj_db = get_trajs_from_db()

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
    
    def advance(self, t):
        # Advance simulator
        self.time += t
        self.simulator.AdvanceTo(self.time)

    def get_traj_controller(self, iiwa_instance):
        iiwa_traj_controller = self.iiwa1_traj_controller if iiwa_instance == 1 else self.iiwa2_traj_controller
        return iiwa_traj_controller
    
    def get_grasp_controller(self, iiwa_instance):
        iiwa_grasp_controller = self.iiwa1_grasp_controller if iiwa_instance == 1 else self.iiwa2_grasp_controller
        return iiwa_grasp_controller
    
    def move(self, iiwa_instance, poses=None, traj=None, traj_t=5.0):
        # Get which iiwa to move
        iiwa_traj_controller = self.get_traj_controller(iiwa_instance)

        # Advance trajectory
        if poses is not None:
            iiwa_traj_controller.NextTrajectory(poses, self.time, traj_t=traj_t)
        elif traj is not None:
            iiwa_traj_controller.SetTrajectory(traj, self.time)
        else:
            raise Exception('Must specify poses or trajectory')
        self.advance(traj_t)

    def grip(self, iiwa_instance, grip, grasp_t=0.1):
        # Get which iiwa to move
        iiwa_grasp_controller = self.get_grasp_controller(iiwa_instance)

        # Set gripper
        iiwa_grasp_controller.SetGripper(grip)
        self.advance(grasp_t)

    def open_gripper(self, iiwa_instance):
        self.grip(iiwa_instance, 0.025) # width of chess piece

    def close_gripper(self, iiwa_instance):
        self.grip(iiwa_instance, 0.0) # max force

    def chess_move(self, iiwa_instance, move):
        # Get trajectories
        pick_sq = chess.square_name(move.from_square)
        place_sq = chess.square_name(move.to_square)
        pick_traj = self.traj_db[iiwa_instance][pick_sq]
        place_traj = self.traj_db[iiwa_instance][place_sq]

        # open gripper
        self.open_gripper(iiwa_instance)
        print('Gripper opened')

        # home -> post-pick -> pre-pick -> pick
        self.move(iiwa_instance, traj=pick_traj['to_post_pick'])
        self.move(iiwa_instance, traj=pick_traj['to_pick'])
        print('Home to pick')

        # close gripper
        self.close_gripper(iiwa_instance)
        print('Gripper closed')

        # pick -> pre-pick -> post-pick
        self.move(iiwa_instance, traj=pick_traj['from_pick'])
        print('Pick to post pick')

        # post-pick -> post-place -> pre-place -> place
        self.move(iiwa_instance, traj=pick_traj['to_place'][place_sq])
        self.move(iiwa_instance, traj=place_traj['to_pick'])
        print('Post pick to place')

        # open gripper
        self.open_gripper(iiwa_instance)
        print('Gripper opened')

        # place -> pre-place -> post-place
        self.move(iiwa_instance, traj=place_traj['from_pick'])

        # close gripper
        self.close_gripper(iiwa_instance)
        print('Gripper closed')

        # post-place -> home
        self.move(iiwa_instance, traj=place_traj['from_post_pick'])
        
    def control_loop(self, simulator):
        # Called once every simulation step

        # Get a move
        move = self.game.get_move()

        # Get which iiwa to move
        iiwa_instance = self.game.get_turn()
        
        # Make move
        # self.chess_move(iiwa_instance, move)
        self.chess_move(iiwa_instance, move)
        
        # # Run perception pipeline
        # piece_poses = self.get_piece_poses()

        # # Check board state
        # self.game.check_game_state(piece_poses)

        # # Get a move
        # move = self.game.get_move()

        # # Handle move
        # if self.game.is_capture_move(move):
        #     # TODO: Remove piece first
        #     # For now just end
        #     return