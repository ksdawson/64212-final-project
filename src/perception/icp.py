import numpy as np
from pydrake.all import (
    RigidTransform, RollPitchYaw
)
from manipulation.icp import IterativeClosestPoint

######################################################################
# Iterative Closest Point (ICP)
######################################################################

def compute_icp_error_alt(X_Ohat, X_PC):
    error = X_Ohat.inverse().multiply(X_PC)
    rpy = RollPitchYaw(error.rotation()).vector()
    xyz = error.translation()
    return rpy, xyz

def compute_icp_error(X_Ohat, p_Om, p_Ws, chat):
    p_Om_transformed = X_Ohat @ p_Om[:, chat]
    errors = p_Ws - p_Om_transformed
    rms_error = np.sqrt(np.mean(np.sum(errors**2, axis=0)))
    return rms_error

def run_icp(scene_point_cloud, model_point_cloud, max_iters = 25, initial_guess = RigidTransform()):
    scene = np.array(scene_point_cloud.xyzs())
    model = np.array(model_point_cloud.xyzs())
    X_Ohat, chat = IterativeClosestPoint(
        p_Om=model,
        p_Ws=scene,
        X_Ohat=initial_guess,
        max_iterations=max_iters
    )
    icp_error = compute_icp_error(X_Ohat, model, scene, chat)
    icp_error_alt = compute_icp_error_alt(X_Ohat, initial_guess)
    return X_Ohat, chat, icp_error, icp_error_alt