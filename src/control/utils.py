import pickle
from motion.kinematics import trajectory, reverse_traj

def get_trajs_from_db():
    # Get the traj db
    file_path = 'traj_db.pkl'
    with open(file_path, 'rb') as file:
        # Load the data from the pickle file
        data = pickle.load(file)
    
    # Build the trajectories from the knots
    trajs = {}
    for iiwa_instance in data:
        trajs[iiwa_instance] = {}
        for move, knots in data[iiwa_instance].items():
            traj = trajectory(knots)
            opp_traj = reverse_traj(traj)
            trajs[iiwa_instance][move] = {'to': traj, 'from': opp_traj}

    return trajs