from PIL import Image, ImageDraw, ImageFont
import numpy as np
from pydrake.all import (
    Rgba, RigidTransform, Box, PointCloud, Sphere, JacobianWrtVariable
)
from motion.kinematics import inverse_kinematics, inverse_kinematics_axis

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


######################################################################
# Drake functions
######################################################################

def poses_equal(pose1, pose2, pos_tol=1e-3, rot_tol=1e-3):
    # Check translation
    if np.linalg.norm(pose1.translation() - pose2.translation()) > pos_tol:
        return False

    # Check rotation via angle between rotation matrices
    R_diff = pose1.rotation().matrix().T @ pose2.rotation().matrix()
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
    if angle > rot_tol:
        return False

    return True

def is_near_singularity(plant, context, q, threshold=1e-3):
    # Add gripper joints if needed
    if plant.num_positions() > len(q):
        q = np.hstack((q, np.zeros(plant.num_positions() - len(q))))

    # Set the robot configuration
    plant.SetPositions(context, q)
    
    # Calculate Jacobian
    ee_frame = plant.GetFrameByName('body')
    J = plant.CalcJacobianSpatialVelocity(
        context,
        with_respect_to=JacobianWrtVariable.kQDot,
        frame_B=ee_frame,
        p_BoBp_B=np.zeros((3,1)),
        frame_A=plant.world_frame(),
        frame_E=plant.world_frame()
    )
    
    # Compute singular values
    s = np.linalg.svd(J, compute_uv=False)
    min_singular_value = np.min(s)
    
    return min_singular_value < threshold, min_singular_value

def find_home_configuration(plant, context, n_candidates=10, pos_tol=0.001, rot_tol=0.01):
    # Get target
    X_target = plant.EvalBodyPoseInWorld(context, plant.GetBodyByName('body'))
    q_nominal = plant.GetPositions(context)

    # Candidate generation functions
    def gen_perturbed_candidates(plant, q_nominal):
        n_q = plant.num_positions()
        return q_nominal + np.random.randn(n_q) * 0.02  # 0.02 rad perturbation
    def gen_random_candidates(plant):
        lower = plant.GetPositionLowerLimits()
        upper = plant.GetPositionUpperLimits()
        q_nominal = np.random.uniform(low=lower, high=upper)
        return q_nominal

    # Generate candidates
    q_candidates = [q_nominal]
    for _ in range(n_candidates):
        # q_nominal = gen_perturbed_candidates(plant, q_nominal)
        q_nominal = gen_random_candidates(plant)
        try:
            # result = inverse_kinematics(plant, context, X_target, q_nominal=q_nominal, pos_tol=pos_tol, rot_tol=rot_tol)
            result = inverse_kinematics_axis(plant, context, X_target, [0, 1, 0], [0, 0, -1], q_nominal=q_nominal, pos_tol=pos_tol, rot_tol=rot_tol)
            q_candidates.append(result)
        except:
            continue
    
    # Score them based on how far from singularity they are
    scores = []
    for q in q_candidates:
        _, min_sv = is_near_singularity(plant, context, q[:7])
        scores.append(min_sv)

    # Choose the best one
    best_idx = np.argmax(scores)
    q_home = q_candidates[best_idx]

    # Get relative improvement over base configuration
    relative_perf = scores[best_idx] / scores[0]

    return q_home, relative_perf