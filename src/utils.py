from PIL import Image, ImageDraw, ImageFont
import numpy as np
from pydrake.all import (
    Rgba, RigidTransform, Box, PointCloud, Sphere
)

######################################################################
# Visualization functions
######################################################################

def visualize_box(meshcat, lower, upper, name=None):
    size = [upper[i] - lower[i] for i in range(3)]
    center = [(upper[i] + lower[i])/2 for i in range(3)]
    name = name if name else f'Box={lower}-{upper}'

    # Create box
    meshcat.SetObject(
        name,
        Box(size[0], size[1], size[2]),
        rgba=Rgba(1, 0, 0, 0.5)  # optional transparency
    )
    # Position it at the center
    meshcat.SetTransform(name, RigidTransform(center))

def visualize_text(meshcat, text, pose=RigidTransform(), name=None,
                   scale=0.1, point_size=0.001, color=Rgba(1, 0, 0)):
    # Create a text image
    W, H = 1024, 256
    img = Image.new('RGB', (W, H), color=(0, 0, 0))
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype('DejaVuSans.ttf', 120)
    draw.text((20, 20), text, fill=(255, 255, 255), font=font)
    img_np = np.array(img)

    # Extract pixels
    ys, xs = np.where(img_np[:, :, 0] > 40)

    # Convert to centered coordinates
    xs = xs.astype(float) - W/2
    ys = -(ys.astype(float) - H/2)

    # Scale
    xs *= scale / W
    ys *= scale / W
    zs = np.zeros_like(xs)

    # Convert to point cloud
    pts = np.vstack([xs, ys, zs])
    pc = PointCloud(pts.shape[1])
    pc.mutable_xyzs()[:] = pts

    # Visualize cloud
    if name is None:
        name = f'/text_cloud/{text}'
    meshcat.SetObject(name, pc, point_size=point_size, rgba=color)
    meshcat.SetTransform(name, pose)

def visualize_iiwa_reachability(meshcat, plant, N=3000):
    ctx = plant.CreateDefaultContext()
    ee_frame = plant.GetFrameByName('body')

    # Joint limits
    lower = []
    upper = []
    for i in range(7):
        lim = plant.GetJointByName(f'iiwa_joint_{i+1}')
        lower.append(lim.position_lower_limits()[0])
        upper.append(lim.position_upper_limits()[0])
    lower = np.array(lower)
    upper = np.array(upper)

    positions = []

    for i in range(N):
        q = lower + np.random.rand(len(lower)) * (upper - lower)
        q = np.hstack((q, np.array([0.,0.])))
        plant.SetPositions(ctx, q)

        X_WE = plant.EvalBodyPoseInWorld(ctx, ee_frame.body())
        positions.append(X_WE.translation())

    # Plot all points as tiny spheres
    for i, p in enumerate(positions):
        path = f'/reachability/{i}'
        meshcat.SetObject(path, Sphere(0.01))
        meshcat.SetTransform(path, RigidTransform(p))

    print(f'Reachability visualization complete. Limits: {lower}, {upper}')