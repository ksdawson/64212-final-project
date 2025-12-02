import chess
import random
from pydrake.all import RigidTransform

# Utils
def square_to_index(square):
    file = square[0] # a–h
    rank = square[1] # 1–8
    file_idx = ord(file) - ord('a')
    rank_idx = int(rank) - 1
    return rank_idx, file_idx

def index_to_square(rank_idx, file_idx):
    file = chr(file_idx + ord('a'))
    rank = str(rank_idx + 1)
    return file + rank

class Game:
    def __init__(self):
        # Game state
        self.board = chess.Board()

        # Game in the simulation
        self.sq_size = 0.047
        self.board_xyz = (0, 0, 0.390591)
        self.board_size = self.sq_size * 8
        self.piece_size = 0.028626

    def get_turn(self):
        return 1 if self.board.turn == chess.WHITE else 2

    def is_capture_move(self, move):
        return self.board.is_capture(move)
    
    def is_castling_move(self, move):
        return self.board.is_castling(move)

    def get_piece_at(self, square):
        piece = self.board.piece_at(square)
        if piece:
            return piece.symbol()
        return None
    
    def get_num_captured(self, player):
        is_opp = (lambda x: x.islower()) if player == 1 else (lambda x: x.isupper())
        num_captured = 16
        for file_idx in range(ord('a'), ord('h') + 1):
            file = chr(file_idx)
            for rank_idx in range(1, 9):
                rank = str(rank_idx)
                sq = file + rank
                piece = self.get_piece_at(chess.parse_square(sq))
                if piece is not None and is_opp(piece):
                    num_captured -= 1
        return num_captured
    
    def get_move(self):
        # Returns a random legal move
        moves = list(self.board.legal_moves)
        return random.choice(moves) if moves else None
        
    def get_non_capture_move(self):
        # Returns a random legal non-capture move
        non_capture_moves = [move for move in self.board.legal_moves if not self.is_capture_move(move)]
        return random.choice(non_capture_moves) if non_capture_moves else None

    def make_move(self, move):
        if isinstance(move, str):
            move = self.board.parse_san(move)
        self.board.push(move)

    def pose_to_square(self, pose):
        # Get xyz
        pos = pose.translation()
        x, y, z = pos[0], pos[1], pos[2]

        # Check piece is on board
        half_size = self.board_size / 2
        assert self.board_xyz[0] - half_size <= x <= self.board_xyz[0] + half_size, 'Piece not on board in x-dim'
        assert self.board_xyz[1] - half_size <= y <= self.board_xyz[1] + half_size, 'Piece not on board in y-dim'
        assert z >= self.board_xyz[2], 'Piece below board'
        
        # Center bottom-left corner (+x, +y -> a1) at 0, 0
        x_offset = -x + half_size
        y_offset = -y + half_size

        # Compute file and rank indices (0–7)
        file_idx = int(x_offset / self.sq_size)
        rank_idx = int(y_offset / self.sq_size)

        # Clamp indices
        file_idx = min(max(file_idx, 0), 7)
        rank_idx = min(max(rank_idx, 0), 7)

        # Convert indices to square
        return index_to_square(rank_idx, file_idx)
    
    def square_to_pose(self, square):
        # Get abs location on board
        rank_idx, file_idx = square_to_index(square)

        # Get x, y
        half_size = self.board_size / 2
        x = half_size - (file_idx + 0.5) * self.sq_size
        y = half_size - (rank_idx + 0.5) * self.sq_size

        # Build pose
        z = self.board_xyz[2]
        pose = RigidTransform()
        pose.set_translation([x, y, z])
        return pose

    def check_game_state(self, piece_poses):
        # Convert poses to squares
        piece_squares = {piece: [self.pose_to_square(pose) for pose in poses] for piece, poses in piece_poses.items()}

        # Check game state is correct
        for piece, squares in piece_squares.items():
            for square in squares:
                actual_piece = str(self.board.piece_at(chess.parse_square(square)))
                # assert piece == actual_piece, f'Incorrect piece {piece} at {square} (actually {actual_piece})'
                if piece != actual_piece:
                    print(f'Incorrect piece {piece} at {square} (actually {actual_piece})')

if __name__ == '__main__':
    # Game
    game = Game()
    print(game.board)

    # Example piece
    pose = RigidTransform()
    pose.set_translation([0.04 * 4, 0.04 * 4, 0.6])

    pose1 = RigidTransform()
    pose1.set_translation([-0.04 * 3, 0.04 * 4, 0.6])

    # Test pose to square
    print(game.pose_to_square(pose))
    print(game.pose_to_square(pose1))

    # Test turn
    poses = {'R': [pose]}
    game.check_game_state(poses)

    # Test square to pose
    print(game.square_to_pose('a1'))
    print(game.square_to_pose('g1'))

    # Check convert functions
    sq = index_to_square(0, 0)
    print(sq)
    r, f = square_to_index('a1')
    print(r, f)