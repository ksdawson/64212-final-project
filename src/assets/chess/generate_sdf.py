import os

base_sdf_string = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="NAME">
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.000015</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.000015</iyy>
          <iyz>0.0</iyz>
          <izz>0.0000108</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.03 0.03 0.06</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <mesh>
            <uri>model.obj</uri>
            <scale>0.00044 0.00044 0.00044</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>'''

def main():
    # Get paths to assets
    current_directory = os.getcwd()
    chess_assets_directory = 'src/assets/chess'

    # Generate SDFs
    pieces = ['pawn', 'king', 'queen', 'bishop', 'knight', 'rook']
    colors = ['dark', 'light']
    for color in colors:
        for piece in pieces:
            # Get SDF file
            name = f'{color}_{piece}'
            sdf_string = base_sdf_string.replace('NAME', name) # TODO: fix collision box height

            # Output to file
            out_file = f'{current_directory}/{chess_assets_directory}/individual_pieces/{name}/model.sdf'
            with open(out_file, 'w') as f:
                f.write(sdf_string)

if __name__ == '__main__':
    main()