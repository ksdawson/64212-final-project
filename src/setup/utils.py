import numpy as np
from pydrake.all import RollPitchYaw

def read_obj_vertices(path):
    verts = []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                verts.append([x, y, z])
    return np.asarray(verts)

def compute_xy_offset(obj_path, scale, rpy_deg=(90,0,0), method='bbox'):
    # Get mesh
    verts = read_obj_vertices(obj_path)
    if verts.size == 0:
        raise ValueError('OBJ has no vertices.')

    # Rotate mesh
    rpy_rad = np.radians(rpy_deg)
    R = RollPitchYaw(*rpy_rad).ToRotationMatrix().matrix()
    verts_rot = (R @ verts.T).T

    # Scale
    verts_rot /= scale

    # Get x,y
    xs = verts_rot[:, 0]
    ys = verts_rot[:, 1]

    # Compute offset
    if method == 'centroid':
        cx = xs.mean()
        cy = ys.mean()
    elif method == 'bbox':
        cx = 0.5 * (xs.min() + xs.max())
        cy = 0.5 * (ys.min() + ys.max())
    else:
        raise ValueError('method must be "bbox" or "centroid"')

    return float(-cx), float(-cy)

def get_xy_offsets():
    offsets = {'dark': {}, 'light': {}}

    # Get kings
    offsets['dark']['king'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/dark_king/model.obj', 6500)
    offsets['light']['king'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/light_king/model.obj', 6500)

    # Get queens
    offsets['dark']['queen'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/dark_queen/model.obj', 6500)
    offsets['light']['queen'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/light_queen/model.obj', 6500)

    # Get bishops
    offsets['dark']['bishop'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/dark_bishop/model.obj', 6500)
    offsets['light']['bishop'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/light_bishop/model.obj', 6500)

    # Get knights
    offsets['dark']['knight'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/dark_knight/model.obj', 6500)
    offsets['light']['knight'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/light_knight/model.obj', 6500)

    # Get rooks
    offsets['dark']['rook'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/dark_rook/model.obj', 6500)
    offsets['light']['rook'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/light_rook/model.obj', 6500)

    # Get pawns
    offsets['dark']['pawn'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/dark_pawn/model.obj', 5000)
    offsets['light']['pawn'] = compute_xy_offset('/workspaces/code/src/assets/chess/pieces/individual_pieces/light_pawn/model.obj', 5000)
    
    return offsets

if __name__ == '__main__':
    offsets = get_xy_offsets()
    print(offsets)