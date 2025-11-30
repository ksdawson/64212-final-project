def lock_joints(iiwa_instance, plant, plant_context):
    # Get other iiwa
    other_iiwa_instance = 1 if iiwa_instance == 2 else 2
    other_iiwa_model = plant.GetModelInstanceByName(f'iiwa{other_iiwa_instance}')
    other_gripper_model = plant.GetModelInstanceByName(f'wsg{other_iiwa_instance}')

    # Lock iiwa joints
    for i in range (1, 8):
        joint_name = f'iiwa_joint_{i}'
        joint = plant.GetJointByName(joint_name, other_iiwa_model)
        if not joint.is_locked(plant_context):
            joint.Lock(plant_context)

    # Lock gripper joints
    joint = plant.GetJointByName('left_finger_sliding_joint', other_gripper_model)
    if not joint.is_locked(plant_context):
        joint.Lock(plant_context)
    joint = plant.GetJointByName('right_finger_sliding_joint', other_gripper_model)
    if not joint.is_locked(plant_context):
        joint.Lock(plant_context)

def unlock_joints(iiwa_instance, plant, plant_context):
    # Get other iiwa
    other_iiwa_instance = 1 if iiwa_instance == 2 else 2
    other_iiwa_model = plant.GetModelInstanceByName(f'iiwa{other_iiwa_instance}')
    other_gripper_model = plant.GetModelInstanceByName(f'wsg{other_iiwa_instance}')

    # Unlock iiwa joints
    for i in range (1, 8):
        joint_name = f'iiwa_joint_{i}'
        joint = plant.GetJointByName(joint_name, other_iiwa_model)
        if joint.is_locked(plant_context):
            joint.Unlock(plant_context)

    # Unlock gripper joints
    joint = plant.GetJointByName('left_finger_sliding_joint', other_gripper_model)
    if joint.is_locked(plant_context):
        joint.Unlock(plant_context)
    joint = plant.GetJointByName('right_finger_sliding_joint', other_gripper_model)
    if joint.is_locked(plant_context):  
        joint.Unlock(plant_context)