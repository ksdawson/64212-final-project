RANKS = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
FILES = ['1', '2', '3', '4', '5', '6', '7', '8']

def move_to_index(move):
    rank, file = move[0], move[1]
    rank_idx = ord(rank) - 97
    file_idx = 8 - int(file)
    return rank_idx, file_idx

class Game:
    def __init__(self):
        board = []
        for rank in RANKS:
            for file in FILES:
                pass

def get_game_square_poses():
    z = 0.527262
    sq_size = 0.047
    rank = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    file = ['1', '2', '3', '4', '5', '6', '7', '8']
    board = {}

if __name__ == '__main__':
    print(move_to_index('a8'))