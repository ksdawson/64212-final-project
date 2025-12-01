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
            trajs[iiwa_instance][move] = {}
            for traj_type, traj_knots in knots.items():
                if traj_type == 'to_place':
                    traj_t = 0.5
                    traj = {sq: trajectory(sq_knots, t=traj_t) for sq, sq_knots in traj_knots.items()} if traj_knots is not None else None
                else:
                    if traj_type in ('to_post_pick', 'from_post_pick'):
                        # Speed doesn't matter since not holding a piece or near pieces but should be reasonable
                        traj_t = 1.0
                    else:
                        traj_t = 0.5
                    traj = trajectory(traj_knots, t=traj_t) if traj_knots is not None else None
                trajs[iiwa_instance][move][traj_type] = traj

    return trajs

if __name__ == '__main__':
    get_trajs_from_db()