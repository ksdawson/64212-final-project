import pickle
from motion.kinematics import trajectory

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
            pick_knots, post_pick_knots = knots['pick'], knots['post_pick']
            traj = trajectory(pick_knots) if pick_knots is not None else None
            opp_traj = trajectory(post_pick_knots) if post_pick_knots is not None else None
            trajs[iiwa_instance][move] = {'pick': traj, 'post_pick': opp_traj}

    return trajs

if __name__ == '__main__':
    get_trajs_from_db()