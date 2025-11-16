import os
from assets.utils import *

# Source: https://free3d.com/3d-model/another-chess-60360.html
base_sdf_string = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{NAME}">
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>{MASS}</mass>
        <inertia>
          <ixx>{IXX}</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>{IYY}</iyy>
          <iyz>0.0</iyz>
          <izz>{IZZ}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>{X_OFFSET} {Y_OFFSET} {Z_OFFSET} 0 0 0</pose>
        <geometry>
          <box>
            <size>{X} {Y} {Z}</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <mesh>
            <uri>model.obj</uri>
            <scale>{SCALE_X} {SCALE_Y} {SCALE_Z}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>'''

def main():
    # Get paths to assets
    current_directory = os.getcwd()
    chess_assets_directory = 'assets/chess'

    # Chess info
    colors = ['dark', 'light']
    pieces = ['pawn', 'king', 'queen', 'bishop', 'knight', 'rook']
    piece_dimensions = {
        'pawn': {'X': 136.31, 'Y': 209.78, 'Z': 136.31},
        'rook':   {'X': 165.90, 'Y': 259.85, 'Z': 163.38},
        'knight': {'X': 173.22, 'Y': 329.27, 'Z': 185.68},
        'bishop': {'X': 157.10, 'Y': 357.42, 'Z': 157.09},
        'queen':  {'X': 186.09, 'Y': 429.28, 'Z': 186.10},
        'king':   {'X': 186.07, 'Y': 494.27, 'Z': 186.06},
    }
    piece_offsets = {
        'dark': {
            'pawn': {'X_OFFSET': 665.47, 'Y_OFFSET': 104.89, 'Z_OFFSET': -1539.60},
            'king': {'X_OFFSET': 416.84, 'Y_OFFSET': 247.14, 'Z_OFFSET': -1544.35},
            'queen': {'X_OFFSET': 139.83, 'Y_OFFSET': 214.64, 'Z_OFFSET': -1546.14},
            'bishop': {'X_OFFSET': -137.92, 'Y_OFFSET': 178.70, 'Z_OFFSET': -1547.53},
            'knight': {'X_OFFSET': -410.08, 'Y_OFFSET': 164.64, 'Z_OFFSET': -1546.35},
            'rook': {'X_OFFSET': -682.95, 'Y_OFFSET': 129.92, 'Z_OFFSET': -1547.69}
        },
        'light': {
            'pawn': {'X_OFFSET': 665.47, 'Y_OFFSET': 104.89, 'Z_OFFSET': -2203.89},
            'king': {'X_OFFSET': 416.84, 'Y_OFFSET': 247.14, 'Z_OFFSET': -2208.64},
            'queen': {'X_OFFSET': 139.83, 'Y_OFFSET': 214.64, 'Z_OFFSET': -2210.43},
            'bishop': {'X_OFFSET': -137.92, 'Y_OFFSET': 178.70, 'Z_OFFSET': -2211.82},
            'knight': {'X_OFFSET': -410.08, 'Y_OFFSET': 164.64, 'Z_OFFSET': -2210.64},
            'rook': {'X_OFFSET': -682.95, 'Y_OFFSET': 129.92, 'Z_OFFSET': -2211.98}
        }
    }
    scaling_factor = 4500 # Experiment with this

    # Generate SDFs
    for color in colors:
        for piece in pieces:
            # Calculate collision box height based on mesh height
            dim = piece_dimensions[piece]
            x, y, z = dim['X'], dim['Y'], dim['Z']
            x_offset, y_offset, z_offset = piece_offsets[color][piece]['X_OFFSET'], piece_offsets[color][piece]['Y_OFFSET'], piece_offsets[color][piece]['Z_OFFSET']
            x /= scaling_factor
            y /= scaling_factor
            z /= scaling_factor
            x_offset /= scaling_factor
            y_offset /= scaling_factor
            z_offset /= scaling_factor

            # Scale mesh to same size as collision box
            scale = 1/scaling_factor
            scale_x, scale_y, scale_z = scale, scale, scale

            # Calculate mass and inertia
            v = calculate_rectangular_prism_volume(x, y, z)
            d = 700 # kg/m^3 (wood)
            m = calculate_mass(v, d) # TODO: verify this is reasonable
            ixx, iyy, izz = calculate_rectangular_prism_inertia(x, y, z, m)

            # Get SDF file
            name = f'{color}_{piece}'
            sdf_string = base_sdf_string.format(NAME=name,
              MASS=m, IXX=ixx, IYY=iyy, IZZ=izz,
              X=x, Y=y, Z=z,
              X_OFFSET=x_offset, Y_OFFSET=y_offset, Z_OFFSET=z_offset,
              SCALE_X=scale_x, SCALE_Y=scale_y, SCALE_Z=scale_z
            )

            # Output to file
            out_file = f'{current_directory}/{chess_assets_directory}/pieces/individual_pieces/{name}/model.sdf'
            with open(out_file, 'w') as f:
                f.write(sdf_string)

if __name__ == '__main__':
    main()