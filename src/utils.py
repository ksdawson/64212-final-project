from pydrake.all import (
    Rgba, RigidTransform, Box
)

def visualize_box(meshcat, lower, upper, name=None):
    size = [upper[i] - lower[i] for i in range(3)]
    center = [(upper[i] + lower[i])/2 for i in range(3)]
    name = name if name else f'Box={lower}-{upper}'

    # Create box
    meshcat.SetObject(
        f"leg{i+1}",
        Box(size[0], size[1], size[2]),
        rgba=Rgba(1, 0, 0, 0.5)  # optional transparency
    )
    # Position it at the center
    meshcat.SetTransform(name, RigidTransform(center))