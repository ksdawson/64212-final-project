import chess
import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw, RotationMatrix
from perception.perception import perception
from perception.point_cloud import get_oriented_piece_model_pcs
from motion.kinematics import reverse_traj
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
    
    def move(self, iiwa_instance, poses=None, times=None, traj=None, traj_t=5.0):
        # Get which iiwa to move
        iiwa_traj_controller = self.iiwa1_traj_controller if iiwa_instance == 1 else self.iiwa2_traj_controller

        # Advance trajectory
        if poses is not None:
            iiwa_traj_controller.NextTrajectory(poses=poses, times=times, traj_t=traj_t)
        elif traj is not None:
            iiwa_traj_controller.SetTrajectory(traj)
        else:
            raise Exception('Must specify poses or trajectory')

        # Advance simulator
        self.time += traj_t
        self.simulator.AdvanceTo(self.time)

    def grip(self, iiwa_instance, grip, traj_t=1.0):
        # Get which iiwa to move
        iiwa_grasp_controller = self.iiwa1_grasp_controller if iiwa_instance == 1 else self.iiwa2_grasp_controller

        # Set gripper
        iiwa_grasp_controller.SetGripper(grip)

        # Advance simulator
        self.time += traj_t
        self.simulator.AdvanceTo(self.time)

    def open_gripper(self, iiwa_instance):
        self.grip(iiwa_instance, 0.025) # width of chess piece

    def close_gripper(self, iiwa_instance):
        self.grip(iiwa_instance, 0.0) # max force

    def NEW_chess_move(self, iiwa_instance, move):
        pass
    
    def chess_move(self, iiwa_instance, move):
        # Get which iiwa to move
        iiwa_traj_controller = self.iiwa1_traj_controller if iiwa_instance == 1 else self.iiwa2_traj_controller
        iiwa_X_WG_home = self.iiwa1_X_WG_home if iiwa_instance == 1 else self.iiwa2_X_WG_home

        # Get end poses
        X_WG_start = iiwa_traj_controller.get_current_pose()
        X_WG_pick = self.game.square_to_pose(chess.square_name(move.from_square))
        X_WG_place = self.game.square_to_pose(chess.square_name(move.to_square))

        # Get intermediate poses
        rpy_down = RotationMatrix(RollPitchYaw(-np.pi/2, 0, 0)) # gripper pointing down
        pick_xyz = X_WG_pick.translation()
        X_WG_prepick = RigidTransform(rpy_down, [pick_xyz[0], pick_xyz[1], pick_xyz[2] + 0.1 + 0.076]) # offset to gripper origin is 0.1, max piece height is 0.076
        X_WG_postpick = RigidTransform(rpy_down, [pick_xyz[0], pick_xyz[1], pick_xyz[2] + 0.1 + 2*0.076])
        place_xyz = X_WG_place.translation()
        X_WG_preplace = RigidTransform(rpy_down, [place_xyz[0], place_xyz[1], place_xyz[2] + 0.1 + 0.076])

        # Adjust end poses
        X_WG_pick.set_rotation(rpy_down)
        X_WG_pick.set_translation([pick_xyz[0], pick_xyz[1], pick_xyz[2] + 0.125]) # midpoint is 0.025 (grasp at midpoint for better stability)
        X_WG_place.set_rotation(rpy_down)
        X_WG_place.set_translation([place_xyz[0], place_xyz[1], place_xyz[2] + 0.125])

        # Go home if not at home
        if not poses_equal(X_WG_start, iiwa_X_WG_home):
            print('Moving home')
            self.move(iiwa_instance, poses=[X_WG_start, iiwa_X_WG_home], times=[0.0, 1.0])
        print('Home')

        # Open gripper
        self.open_gripper(iiwa_instance)
        print('Gripper opened')

        # Go to pre-pick -> pick
        self.move(iiwa_instance, poses=[X_WG_prepick, X_WG_pick])
        print('Pre-pick to pick')

        # Close gripper
        self.close_gripper(iiwa_instance)
        print('Gripper closed')

        # Go to pre-pick -> home
        # self.move(iiwa_instance, traj=reverse_traj(iiwa_traj_controller._traj)) # just reverse the previous traj
        self.move(iiwa_instance, poses=[X_WG_postpick, X_WG_start])
        print('Post-pick to home')

        # Go to pre-place -> place
        self.move(iiwa_instance, poses=[X_WG_preplace, X_WG_place])
        print('Pre-place to place')

        # Open gripper
        self.open_gripper(iiwa_instance)
        print('Gripper opened')

        # Go to pre-place -> home
        self.move(iiwa_instance, traj=reverse_traj(iiwa_traj_controller._traj)) # just reverse the previous traj
        print('Pre-place to home')
        
    def control_loop(self, simulator):
        # Called once every simulation step

        # Get a move
        move = self.game.get_move()

        # Get which iiwa to move
        iiwa_instance = self.game.get_turn()
        
        # Make move
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
        
        # # Get start pose
        # from_square = move.from_square
        # from_square_name = chess.square_name(from_square)
        # piece = self.game.get_piece_at(from_square)
        # # start_poses = piece_poses[piece]
        # X_WG_pick = self.game.square_to_pose(from_square_name)

        # # Get goal pose
        # to_square = move.to_square
        # to_square_name = chess.square_name(to_square)
        # X_WG_place = self.game.square_to_pose(to_square_name)

        # # Get which iiwa to move
        # iiwa_instance = self.game.get_turn()
        # iiwa_traj_controller = self.iiwa1_traj_controller if iiwa_instance == 1 else self.iiwa2_traj_controller
        # iiwa_grasp_controller = self.iiwa1_grasp_controller if iiwa_instance == 1 else self.iiwa2_grasp_controller

        # # Move to pick pose through a pre-pick pose
        # rpy_down = RollPitchYaw(-np.pi/2, 0, 0)
        # xyz = X_WG_pick.translation()
        # X_WG_prepick = RigidTransform(RotationMatrix(rpy_down), [xyz[0], xyz[1], xyz[2] + 0.175]) # offset is 0.1, max piece height is 0.075

        # # Grasp: pick piece up at midpoint for better stability
        # X_WG_pick.set_rotation(RotationMatrix(rpy_down))
        # X_WG_pick.set_translation([xyz[0], xyz[1], xyz[2] + 0.1 + 0.025]) # offset is 0.1, midpoint is 0.025
        # # TODO: play w/ midpoint

        # # Open gripper
        # iiwa_grasp_controller.SetGripper(0.025)

        # # Advance simulator
        # self.time += 1.0
        # simulator.AdvanceTo(self.time)

        # # Axis-aligned orientation constraint
        # gripper_approach = [0, 1, 0] # gripper +y axis (approach axis)
        # world_axis = [0, 0, -1] # world downward direction (-z)
        # orientation_config = {'type': 'axis', 'gripper_axis': gripper_approach, 'world_axis': world_axis}

        # # Move to pre-pick -> pick
        # X_WStart = iiwa_traj_controller.get_current_pose()
        # # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick, X_WG_pick], traj_t=5.0)
        # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick, X_WG_pick], orientation_config=orientation_config, traj_t=5.0)

        # # Advance simulator
        # self.time += 5.0
        # simulator.AdvanceTo(self.time)

        # # Close gripper
        # iiwa_grasp_controller.SetGripper(0.0) # max force

        # # Advance simulator
        # self.time += 1.0
        # simulator.AdvanceTo(self.time)

        # # Move to pre-pick
        # X_WStart = iiwa_traj_controller.get_current_pose()
        # # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick], traj_t=5.0)
        # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_prepick], orientation_config=orientation_config, traj_t=5.0)

        # # Advance simulator
        # self.time += 5.0
        # simulator.AdvanceTo(self.time)

        # # Move to place pose through a pre-place pose
        # xyz = X_WG_place.translation()
        # X_WG_preplace = RigidTransform(RotationMatrix(rpy_down), [xyz[0], xyz[1], xyz[2] + 0.175]) # offset is 0.1, max piece height is 0.075
        # X_WG_place.set_rotation(RotationMatrix(rpy_down))
        # X_WG_place.set_translation([xyz[0], xyz[1], xyz[2] + 0.1 + 0.025]) # offset is 0.1, midpoint is 0.025

        # # Move to pre-place -> place
        # X_WStart = iiwa_traj_controller.get_current_pose()
        # # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_preplace, X_WG_place], traj_t=5.0)
        # iiwa_traj_controller.NextTrajectory(poses=[X_WStart, X_WG_preplace, X_WG_place], orientation_config=orientation_config, traj_t=5.0)

        # # Advance simulator
        # self.time += 5.0
        # simulator.AdvanceTo(self.time)

        # # Open gripper
        # iiwa_grasp_controller.SetGripper(0.025)

        # # Advance simulator
        # self.time += 1.0
        # simulator.AdvanceTo(self.time)