import os
from assets.utils import *

# Source: https://free3d.com/3d-model/woodenchessboard-v1--783762.html
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
        <pose>0 0 {Z_OFFSET} 0 0 0</pose>
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

    # Info
    dim = {'X': 43.25, 'Y': 43.25, 'Z': 1.54, 'Z_OFFSET': -0.82}
    scaling_factor = 100 # Experiment with this

    # Calculate collision box height based on mesh height
    x, y, z, z_offset = dim['X'], dim['Y'], dim['Z'], dim['Z_OFFSET']
    x /= scaling_factor
    y /= scaling_factor
    z /= scaling_factor
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
    name = 'chessboard'
    sdf_string = base_sdf_string.format(NAME=name,
        MASS=m, IXX=ixx, IYY=iyy, IZZ=izz,
        X=x, Y=y, Z=z, Z_OFFSET=z_offset,
        SCALE_X=scale_x, SCALE_Y=scale_y, SCALE_Z=scale_z
    )

    # Output to file
    out_file = f'{current_directory}/{chess_assets_directory}/chessboard/model.sdf'
    with open(out_file, 'w') as f:
        f.write(sdf_string)

if __name__ == '__main__':
    main()