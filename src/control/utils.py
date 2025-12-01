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
            pick_knots, post_pick_knots, home_knots, place_knots = knots['pick'], knots['post_pick'], knots['home'], knots['place']
            pick_traj = trajectory(pick_knots) if pick_knots is not None else None
            postpick_traj = trajectory(post_pick_knots) if post_pick_knots is not None else None
            home_traj = trajectory(home_knots) if home_knots is not None else None
            place_traj = {sq: trajectory(sq_knots) for sq, sq_knots in place_knots.items()} if place_knots is not None else None
            trajs[iiwa_instance][move] = {'pick': pick_traj, 'post_pick': postpick_traj, 'home': home_traj, 'place': place_traj}

    return trajs

# def explore_trajs_from_db():
#     # Get the traj db
#     file_path = 'traj_db.pkl'
#     with open(file_path, 'rb') as file:
#         # Load the data from the pickle file
#         data = pickle.load(file)

#     # 
#     for iiwa_instance in data:
#         for move, knot_types in data[iiwa_instance].items():
#             for knot_type, knots in knot_types.items():


if __name__ == '__main__':
    get_trajs_from_db()