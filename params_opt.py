import numpy as np


def bayesopt_bounds(task: str, coords="cartesianas", variation="1button"):
    n = 0
    lb = 0
    ub = 0
    if task == 'avoid_obstacle':
        n = 6  # n dimensions
        if coords == 'cartesianas':
            lb = np.array([-0.3, 0.1, -0.15, -0.15, 0.2, -0.15])  # coords cart
            ub = np.array([0.3, 0.3, 0.4, 0.15, 0.5, 0.4])
        elif coords == 'esfericas':
            lb = np.array([0.1, 0.0, 0.0, 0.1, 0.0, 0.0])  # coords esf
            ub = np.array([0.5, np.pi, np.pi, 0.5, np.pi, np.pi])
    elif task == 'pick_and_place':
        n = 6  # n dimensions
        lb = np.array([-0.15, -0.15, -0.25, -0.1, -0.1, -0.25])
        ub = np.array([0.15, 0.15, -0.14, 0.1, 0.1, -0.14])
    elif task == 'push_button':
        n = 6  # n dimensions
        lb = np.array([0.0, -0.1, -0.15, -np.pi / 4, 0.0, -np.pi / 2])
        ub = np.array([0.1, 0.1, 0.0, np.pi / 4, np.pi / 3, np.pi / 2])
    elif task == 'slide_block':
        n = 5  # n dimensions
        if variation == '1block':
            lb = np.array([-0.2, 0.0, -0.25, 0.05, -np.pi / 4])
            ub = np.array([0.2, 0.1, 0, 0.4, np.pi / 4])
        elif variation == '2block':
            lb = np.array([-0.15, 0.0, -0.25, 0.2, -np.pi / 4])
            ub = np.array([0.15, 0.2, 0, 0.5, np.pi / 4])
    elif task == 'three_obstacles':
        n = 4  # n dimensions
        lb = np.array([0.1, 0.0, 0.1, -np.pi / 4])
        ub = np.array([0.6, 70 * np.pi / 180, 0.4, np.pi / 4])
    else:
        print('Nombre de la tarea no válido')
    return n, lb, ub


def difevol_bounds(task: str, coords="cartesianas", variation="1button"):
    bounds = []
    if task == 'avoid_obstacle':
        if coords == 'cartesianas':
            bounds = [(-0.3, 0.3), (0.1, 0.3), (-0.15, 0.4), (-0.15, 0.15), (0.2, 0.5), (-0.15, 0.4)]  # coords cart
        elif coords == 'esfericas':
            bounds = [(0.1, 0.5), (0.0, np.pi), (0.0, np.pi), (0.1, 0.5), (0.0, np.pi), (0.0, np.pi)]  # coords esf
    elif task == 'pick_and_place':
        bounds = [(-0.15, 0.15), (-0.15, 0.15), (-0.25, -0.15), (-0.1, 0.1), (-0.1, 0.1), (-0.25, -0.15)]
    elif task == 'push_button':
        bounds = [(0.0, 0.1), (-0.1, 0.1), (-0.15, 0.0), (np.pi / 4, np.pi / 4), (0.0, np.pi / 3),
                  (-np.pi / 2, np.pi / 2)]
    elif task == 'slide_block':
        if variation == '1block':
            bounds = [(-0.2, 0.2), (0.0, 0.2), (-0.25, 0.0), (0.05, 0.4), (-np.pi / 4, np.pi / 4)]
        elif variation == '2block':
            bounds = [(-0.15, 0.15), (0.0, 0.2), (-0.25, 0.0), (0.2, 0.5), (-np.pi / 4, np.pi / 4)]
    elif task == 'three_obstacles':
        bounds = [(0.1, 0.6), (0.0, 70 * np.pi / 180), (0.1, 0.4), (-np.pi/4, np.pi/4)]
    else:
        print('Nombre de la tarea no válido')
    return bounds
