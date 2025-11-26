# Constants
FILES = ['a','b','c','d','e','f','g','h']
RANKS = ['1','2','3','4','5','6','7','8']

# Utils
def square_to_index(square):
    file = square[0] # a–h
    rank = square[1] # 1–8
    file_idx = ord(file) - ord('a')
    rank_idx = 8 - int(rank)
    return rank_idx, file_idx

# Starting board setup
def get_starting_board():
    def file_to_piece(file):
        if file in {'a','h'}: return 'R' # Rook
        if file in {'b','g'}: return 'N' # Knight
        if file in {'c','f'}: return 'B' # Bishop
        if file == 'd': return 'Q' # Queen
        if file == 'e': return 'K' # King
        return ''
    board = [['' for _ in range(8)] for _ in range(8)]
    for file in FILES:
        for rank in RANKS:
            if rank in {'1','2'}:
                piece = ('P' if rank == '2' else file_to_piece(file))
            elif rank in {'7','8'}:
                piece = ('P' if rank == '7' else file_to_piece(file))
                piece = piece.lower()
            else:
                piece = ''
            rank_idx, file_idx = square_to_index(file + rank)
            board[rank_idx][file_idx] = piece
    return board

class Game:
    def __init__(self):
        # Create board state
        self.board = get_starting_board()
        self.turn = 'w'
    
    def make_move(self, move):
        # Get move
        start, end = move[:2], move[2:]

        # Convert move to indices
        sr, sc = square_to_index(start)
        er, ec = square_to_index(end)

        # Get piece to move
        piece = self.board[sr][sc]
        if piece == '':
            raise ValueError(f'No piece on {start}')

        # Move piece
        self.board[er][ec] = piece
        self.board[sr][sc] = ''

        return piece
    
    def __str__(self):
        strs = ['\n  +------------------------+']
        for r in range(8):
            rank_label = 8 - r
            row_str = f'{rank_label} |'
            for c in range(8):
                piece = self.board[r][c]
                row_str += ' . ' if piece == '' else f' {piece} '
            row_str += '|'
            strs.append(row_str)
        strs.append('  +------------------------+')
        strs.append('    a  b  c  d  e  f  g  h\n')
        return '\n'.join(strs)

def get_game_square_poses():
    z = 0.527262
    sq_size = 0.047
    rank = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    file = ['1', '2', '3', '4', '5', '6', '7', '8']
    board = {}

if __name__ == '__main__':
    game = Game()
    print(game)