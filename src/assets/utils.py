def calculate_rectangular_prism_volume(x, y, z):
    return x * y * z

def calculate_mass(volume, density):
    return volume * density

def calculate_rectangular_prism_inertia(x, y, z, m):
    ixx = (1/12) * m * (y**2 + z**2)
    iyy = (1/12) * m * (x**2 + z**2)
    izz = (1/12) * m * (x**2 + y**2)
    return ixx, iyy, izz

def calculate_bounding_box_from_file(filepath):
    # Calculate bounding box using min/max
    min_x, min_y, min_z = None, None, None
    max_x, max_y, max_z = None, None, None
    vertex_count = 0
    with open(filepath, 'r') as f:
        for line in f:
            # Check if the line starts with 'v ' for vertex definition
            if line.startswith('v '):
                # Split the line by spaces and convert coordinates to floats
                parts = line.strip().split()
                # Check for the expected 4 parts: 'v', X, Y, Z
                if len(parts) < 4:
                    # Use print() instead of sys.stderr for simplicity
                    print(f"Skipping incomplete vertex line: {line.strip()}")
                    continue
                # parts[0] is 'v', parts[1] is X, parts[2] is Y, parts[3] is Z
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                # Update the bounding box minimums
                if min_x is None or x < min_x: min_x = x
                if min_y is None or y < min_y: min_y = y
                if min_z is None or z < min_z: min_z = z
                # Update the bounding box maximums
                if max_x is None or x > max_x: max_x = x
                if max_y is None or y > max_y: max_y = y
                if max_z is None or z > max_z: max_z = z
                # Update vertex count
                vertex_count += 1
    if vertex_count == 0:
        return (None, None, None)
    # Calculate the center point
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    center_z = (min_z + max_z) / 2
    # Return
    min_coords = (min_x, min_y, min_z)
    max_coords = (max_x, max_y, max_z)
    center_coords = (center_x, center_y, center_z)
    return min_coords, max_coords, center_coords

def get_all_bounding_boxes():
    objects = ['./assets/chess/chessboard/model.obj', './assets/furniture/table1/model.obj', './assets/furniture/table2/model.obj']
    for o in objects:
        print(o)
        min_coords, max_coords, center_coords = calculate_bounding_box_from_file(o)
        print(min_coords, max_coords, center_coords, '\n')

if __name__ == '__main__':
    get_all_bounding_boxes()