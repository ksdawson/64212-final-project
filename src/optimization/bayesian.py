from bayes_opt import BayesianOptimization

def black_box_function(x, y):
    return -x ** 2 - (y - 1) ** 2 + 1

def bayesian_optimization():
    # Bounded region of parameter space
    pbounds = {'x': (2, 4), 'y': (-3, 3)}

    # Create optimizer object
    optimizer = BayesianOptimization(
        f=black_box_function,
        pbounds=pbounds,
        random_state=1
    )

    # Run optimization
    optimizer.maximize(
        init_points=2,
        n_iter=3
    )

    print(optimizer.max)

if __name__ == '__main__':
    bayesian_optimization()