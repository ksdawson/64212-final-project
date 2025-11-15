def calculate_rectangular_prism_volume(x, y, z):
    return x * y * z

def calculate_mass(volume, density):
    return volume * density

def calculate_rectangular_prism_inertia(x, y, z, m):
    ixx = (1/12) * m * (y**2 + z**2)
    iyy = (1/12) * m * (x**2 + z**2)
    izz = (1/12) * m * (x**2 + y**2)
    return ixx, iyy, izz