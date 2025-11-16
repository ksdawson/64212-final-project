import os
from assets.utils import *

# Source: https://free3d.com/3d-model/straight-leg-coffee-tablepine-v1--697100.html
# Source: https://free3d.com/3d-model/straight-leg-square-tablepine-v1--461712.html
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
    assets_directory = 'assets/furniture'

    # Info
    dimensions = {
        'table1': {'X': 147.70, 'Y': 94.92, 'Z': 47.46,
          'X_OFFSET': 0.0, 'Y_OFFSET': 0.0, 'Z_OFFSET': 22.46
        },
        'table2': {'X': 149.17, 'Y': 149.24, 'Z': 74.61,
          'X_OFFSET': 0.0134, 'Y_OFFSET': 0.0657, 'Z_OFFSET': -8.90
        }
    }
    scaling_factor = 100 # Experiment with this
    tables = ['table1', 'table2']

    for table in tables:
        # Calculate collision box height based on mesh height
        dim = dimensions[table]
        x, y, z = dim['X'], dim['Y'], dim['Z']
        x_offset, y_offset, z_offset = dim['X_OFFSET'], dim['Y_OFFSET'], dim['Z_OFFSET']
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
        name = table
        sdf_string = base_sdf_string.format(NAME=name,
            MASS=m, IXX=ixx, IYY=iyy, IZZ=izz,
            X=x, Y=y, Z=z,
            X_OFFSET=x_offset, Y_OFFSET=y_offset, Z_OFFSET=z_offset,
            SCALE_X=scale_x, SCALE_Y=scale_y, SCALE_Z=scale_z
        )

        # Output to file
        out_file = f'{current_directory}/{assets_directory}/{name}/model.sdf'
        with open(out_file, 'w') as f:
            f.write(sdf_string)

if __name__ == '__main__':
    main()