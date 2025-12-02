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
        self.piece_widths = {
            # 'K': 0.02863,
            # 'Q': 0.02863,
            'K': 0.02740, # maybe go smaller
            'Q': 0.02740,
            'B': 0.02417,
            'N': 0.02665,
            'R': 0.02552,
            'P': 0.02726
        }

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

    def open_gripper(self, iiwa_instance, grip=0.025):
        self.grip(iiwa_instance, grip) # width of chess piece

    def close_gripper(self, iiwa_instance):
        self.grip(iiwa_instance, 0.0) # max force

    def chess_move(self, iiwa_instance, move):
        # Get trajectories
        piece = self.game.get_piece_at(move.from_square)
        grip = self.piece_widths[piece.upper()]
        pick_sq = chess.square_name(move.from_square)
        place_sq = chess.square_name(move.to_square)
        pick_traj = self.traj_db[iiwa_instance][pick_sq]
        place_traj = self.traj_db[iiwa_instance][place_sq]

        # Open gripper
        self.open_gripper(iiwa_instance, grip) # could add 1mm (0.001) for clearance

        # Move home -> post-pick -> pre-pick -> pick
        self.move(iiwa_instance, traj=pick_traj['to_post_pick'], traj_t=1.0)
        self.move(iiwa_instance, traj=pick_traj['to_pick'][piece.upper()], traj_t=0.5)

        # Wait to make pick more stable
        self.advance(0.1)

        # Close gripper
        self.close_gripper(iiwa_instance)

        # Move pick -> pre-pick -> post-pick
        self.move(iiwa_instance, traj=pick_traj['from_pick'][piece.upper()], traj_t=0.5)

        self.advance(0.1)

        # Move post-pick -> post-place -> pre-place -> place
        self.move(iiwa_instance, traj=pick_traj['to_place'][place_sq], traj_t=0.75)

        self.advance(0.1)

        self.move(iiwa_instance, traj=place_traj['to_pick'][piece.upper()], traj_t=0.5)

        self.advance(0.1)

        # Open gripper
        self.open_gripper(iiwa_instance, grip)

        # Move place -> pre-place -> post-place
        self.move(iiwa_instance, traj=place_traj['from_pick'][piece.upper()], traj_t=0.5)

        # Close gripper
        self.close_gripper(iiwa_instance)

        # Move post-place -> home
        self.move(iiwa_instance, traj=place_traj['from_post_pick'], traj_t=1.0)
        
    def chess_remove_piece(self, iiwa_instance, move):
        # Get trajectories
        piece = self.game.get_piece_at(move.to_square)
        grip = self.piece_widths[piece.upper()]
        place_sq = chess.square_name(move.to_square)
        place_traj = self.traj_db[iiwa_instance][place_sq]

        # Open gripper
        self.open_gripper(iiwa_instance, grip + 0.001)

        # Move home -> post-place -> pre-place -> place
        self.move(iiwa_instance, traj=place_traj['to_post_pick'], traj_t=1.0)
        self.move(iiwa_instance, traj=place_traj['to_pick'][piece.upper()], traj_t=0.5)

        self.advance(0.1)

        # Close gripper
        self.close_gripper(iiwa_instance)

        # Move place -> pre-place -> post-place
        self.move(iiwa_instance, traj=place_traj['from_pick'][piece.upper()], traj_t=0.5)

        self.advance(0.1)

        # Move post-place -> home
        self.move(iiwa_instance, traj=place_traj['from_post_pick'], traj_t=1.0)

        # Open gripper
        self.open_gripper(iiwa_instance, grip)

        # Close gripper
        self.close_gripper(iiwa_instance)

    def NEW_chess_remove_piece(self, iiwa_instance, move):
        # Get trajectories
        piece = self.game.get_piece_at(move.to_square)
        grip = self.piece_widths[piece.upper()]
        place_sq = chess.square_name(move.to_square)
        place_traj = self.traj_db[iiwa_instance][place_sq]

        # Open gripper
        self.open_gripper(iiwa_instance, grip + 0.001)

        # Move home -> post-place -> pre-place -> place
        self.move(iiwa_instance, traj=place_traj['to_post_pick'], traj_t=1.0)
        self.move(iiwa_instance, traj=place_traj['to_pick'][piece.upper()], traj_t=0.5)

        self.advance(0.1)

        # Close gripper
        self.close_gripper(iiwa_instance)

        # Move place -> pre-place -> post-place
        self.move(iiwa_instance, traj=place_traj['from_pick'][piece.upper()], traj_t=0.5)

        self.advance(0.1)

        # Move post-place -> capture
        capture_idx = self.game.get_num_captured(iiwa_instance)
        self.move(iiwa_instance, traj=place_traj['to_capture'][capture_idx], traj_t=0.5)

        self.advance(0.1)

        # Open gripper
        self.open_gripper(iiwa_instance, grip)

        # Move capture -> home
        self.move(iiwa_instance, traj=place_traj['from_capture'][capture_idx], traj_t=0.5)

        # Close gripper
        self.close_gripper(iiwa_instance)

    def chess_capture_move(self, iiwa_instance, move):
        # Remove captured piece first
        # For now just throw the piece away
        self.NEW_chess_remove_piece(iiwa_instance, move)

        # Then normal pick/place chess move
        self.chess_move(iiwa_instance, move)

    def chess_castling_move(self, iiwa_instance, move):
        # King squares
        king_from = move.from_square
        king_to = move.to_square

        # Determine rook squares
        if king_to > king_from: # kingside
            rook_from = king_to + 1 # rook starts 1 square to the right of king's destination
            rook_to = king_to - 1
        else: # queenside
            rook_from = king_to - 2 # rook starts 2 squares to the left of king's destination
            rook_to = king_to + 1

        # Construct king move
        king_move = chess.Move(king_from, king_to)
        self.chess_move(iiwa_instance, king_move)

        # Construct rook move
        rook_move = chess.Move(rook_from, rook_to)
        self.chess_move(iiwa_instance, rook_move)

    def control_loop(self, move):
        # Get which iiwa to move
        iiwa_instance = self.game.get_turn()

        # Make move in simulation
        if self.game.is_castling_move(move):
            self.chess_castling_move(iiwa_instance, move)
        elif self.game.is_capture_move(move):
            self.chess_capture_move(iiwa_instance, move)
        else:
            self.chess_move(iiwa_instance, move)

        # Make move in game
        self.game.make_move(move)
        
        # # Run perception pipeline
        # piece_poses = self.get_piece_poses()

        # # Check board state
        # self.game.check_game_state(piece_poses)
    
    def run_game(self, moves=None):
        move_idx = 0
        while True:
            # Get move
            if moves is None:
                move = self.game.get_move()
                if move is None:
                    # Game over
                    return
            else:
                if move_idx >= len(moves):
                    # Game over
                    return
                move = moves[move_idx]
                move_idx += 1

            # Game step
            self.control_loop(move)