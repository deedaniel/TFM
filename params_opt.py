import numpy as np


def bayesopt_bounds(task: str, coords="cartesianas", variation="1button"):
    n = 0
    lb = 0
    ub = 0
    if task == 'avoid_obstacle':
        n = 6  # n dimensions
        if coords == 'cartesianas':
            lb = np.array([-0.3, 0.1, -0.15, -0.1, 0.1, -0.1])  # coords cart
            ub = np.array([0.3, 0.3, 0.25, 0.1, 0.5, 0.1])
        elif coords == 'esfericas':
            lb = np.array([0.1, np.pi / 4, np.pi / 8, 0.1, np.pi / 6, np.pi / 6])  # coords esf
            ub = np.array(
                [0.5, (np.pi - np.pi / 4), (np.pi - np.pi / 8), 0.5, (np.pi - np.pi / 6), (np.pi - np.pi / 6)])
    elif task == 'pick_and_place':
        n = 6  # n dimensions
        if variation == "1container":
            lb = np.array([-0.1, -0.1, -0.27, -0.075, -0.075, -0.27])
            ub = np.array([0.1, 0.1, -0.14, 0.075, 0.075, -0.14])
        elif variation == "2container":
            lb = np.array([-0.1, -0.1, -0.27, -0.2, -0.075, -0.27])
            ub = np.array([0.1, 0.1, -0.14, 0.2, 0.075, -0.14])
    elif task == 'push_button':
        n = 6  # n dimensions
        if variation == "1button":
            lb = np.array([0.0, -0.05, -0.15, -np.pi / 4, 0.0, -np.pi / 2])
            ub = np.array([0.1, 0.05, 0.0, np.pi / 4, np.pi / 3, np.pi / 2])
        elif variation == "2button":
            lb = np.array([-0.1, -0.05, -0.2, -np.pi / 4, 0.0, -np.pi / 2])
            ub = np.array([0.15, 0.05, 0.0, np.pi / 4, np.pi / 3, np.pi / 2])
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
            bounds = [(-0.3, 0.3), (0.1, 0.3), (-0.15, 0.25), (-0.1, 0.1), (0.2, 0.5), (-0.1, 0.1)]  # coords cart
        elif coords == 'esfericas':
            bounds = [(0.1, 0.5), (np.pi / 4, (np.pi - np.pi / 4)), (np.pi / 8, (np.pi - np.pi / 8)), (0.1, 0.5),
                      (np.pi / 6, (np.pi - np.pi / 6)), (np.pi / 6, (np.pi - np.pi / 6))]  # coords esf
    elif task == 'pick_and_place':
        if variation == '1container':
            bounds = [(-0.1, 0.1), (-0.1, 0.1), (-0.27, -0.14), (-0.075, 0.075), (-0.075, 0.075), (-0.27, -0.14)]
        elif variation == '2container':
            bounds = [(-0.1, 0.1), (-0.1, 0.1), (-0.27, -0.14), (-0.2, 0.2), (-0.075, 0.075), (-0.27, -0.14)]
    elif task == 'push_button':
        if variation == '1button':
            bounds = [(0.0, 0.1), (-0.05, 0.05), (-0.15, 0.0), (-np.pi / 4, np.pi / 4), (0.0, np.pi / 3),
                      (-np.pi / 2, np.pi / 2)]
        elif variation == '2button':
            bounds = [(-0.1, 0.15), (-0.05, 0.05), (-0.2, 0.0), (-np.pi / 4, np.pi / 4), (0.0, np.pi / 3),
                      (-np.pi / 2, np.pi / 2)]
    elif task == 'slide_block':
        if variation == '1block':
            bounds = [(-0.2, 0.2), (0.0, 0.2), (-0.25, 0.0), (0.05, 0.4), (-np.pi / 4, np.pi / 4)]
        elif variation == '2block':
            bounds = [(-0.15, 0.15), (0.0, 0.2), (-0.25, 0.0), (0.2, 0.5), (-np.pi / 4, np.pi / 4)]
    elif task == 'three_obstacles':
        bounds = [(0.1, 0.6), (0.0, 70 * np.pi / 180), (0.1, 0.4), (-np.pi / 4, np.pi / 4)]
    else:
        print('Nombre de la tarea no válido')
    return bounds


def sigopt_parameters(task: str, variation="1button"):
    parameters = dict()
    if task == 'avoid_obstacle':
        if variation == 'cartesianas':
            parameters = [
                dict(name='x1', type='double', bounds=dict(min=-0.3, max=0.3)),
                dict(name='y1', type='double', bounds=dict(min=0.1, max=0.3)),
                dict(name='z1', type='double', bounds=dict(min=-0.15, max=0.25)),
                dict(name='x2', type='double', bounds=dict(min=-0.1, max=0.1)),
                dict(name='y2', type='double', bounds=dict(min=0.1, max=0.5)),
                dict(name='z2', type='double', bounds=dict(min=-0.1, max=0.1)),
            ]
        elif variation == 'esfericas':
            parameters = [
                dict(name='x1', type='double', bounds=dict(min=0.1, max=0.5)),
                dict(name='y1', type='double', bounds=dict(min=-0.1, max=0.1)),
                dict(name='z1', type='double', bounds=dict(min=-0.27, max=-0.14)),
                dict(name='x2', type='double', bounds=dict(min=-0.2, max=0.2)),
                dict(name='y2', type='double', bounds=dict(min=-0.075, max=0.075)),
                dict(name='z2', type='double', bounds=dict(min=-0.27, max=-0.14)),
            ]
    elif task == 'pick_and_place':
        if variation == '1container':
            parameters = [
                dict(name='x1', type='double', bounds=dict(min=-0.1, max=0.1)),
                dict(name='y1', type='double', bounds=dict(min=-0.1, max=0.1)),
                dict(name='z1', type='double', bounds=dict(min=-0.27, max=-0.14)),
                dict(name='x2', type='double', bounds=dict(min=-0.075, max=0.075)),
                dict(name='y2', type='double', bounds=dict(min=-0.075, max=0.075)),
                dict(name='z2', type='double', bounds=dict(min=-0.27, max=-0.14)),
            ]
        elif variation == '2container':
            parameters = [
                dict(name='x1', type='double', bounds=dict(min=-0.1, max=0.1)),
                dict(name='y1', type='double', bounds=dict(min=-0.1, max=0.1)),
                dict(name='z1', type='double', bounds=dict(min=-0.27, max=-0.14)),
                dict(name='x2', type='double', bounds=dict(min=-0.2, max=0.2)),
                dict(name='y2', type='double', bounds=dict(min=-0.075, max=0.075)),
                dict(name='z2', type='double', bounds=dict(min=-0.27, max=-0.14)),
            ]
    elif task == 'push_button':
        if variation == '1button':
            parameters = [
                dict(name='x', type='double', bounds=dict(min=0.0, max=0.1)),
                dict(name='y', type='double', bounds=dict(min=-0.05, max=0.05)),
                dict(name='z', type='double', bounds=dict(min=-0.15, max=0.0)),
                dict(name='alpha', type='double', bounds=dict(min=-np.pi / 4, max=np.pi / 4)),
                dict(name='beta', type='double', bounds=dict(min=0.0, max=np.pi / 3)),
                dict(name='gamma', type='double', bounds=dict(min=-np.pi / 2, max=np.pi / 2)),
            ]
        elif variation == '2button':
            parameters = [
                dict(name='x', type='double', bounds=dict(min=-0.1, max=0.5)),
                dict(name='y', type='double', bounds=dict(min=-0.05, max=0.05)),
                dict(name='z', type='double', bounds=dict(min=-0.2, max=0.0)),
                dict(name='alpha', type='double', bounds=dict(min=-np.pi / 4, max=np.pi / 4)),
                dict(name='beta', type='double', bounds=dict(min=0.0, max=np.pi / 3)),
                dict(name='gamma', type='double', bounds=dict(min=-np.pi / 2, max=np.pi / 2)),
            ]
    elif task == 'slide_block':
        if variation == '1block':
            parameters = [
                dict(name='x1', type='double', bounds=dict(min=-0.2, max=0.2)),
                dict(name='y1', type='double', bounds=dict(min=0.0, max=0.2)),
                dict(name='z1', type='double', bounds=dict(min=-0.25, max=-0.06)),
                dict(name='d', type='double', bounds=dict(min=-0.05, max=0.4)),
                dict(name='phi', type='double', bounds=dict(min=-np.pi / 4, max=-np.pi / 4)),
            ]
        elif variation == '2block':
            parameters = [
                dict(name='x1', type='double', bounds=dict(min=-0.15, max=0.15)),
                dict(name='y1', type='double', bounds=dict(min=0.0, max=0.2)),
                dict(name='z1', type='double', bounds=dict(min=-0.25, max=-0.06)),
                dict(name='d', type='double', bounds=dict(min=0.2, max=0.5)),
                dict(name='phi', type='double', bounds=dict(min=-np.pi / 4, max=np.pi / 4)),
            ]
    elif task == 'three_obstacles':
        parameters = [
            dict(name='r1', type='double', bounds=dict(min=0.1, max=0.6)),
            dict(name='tita1', type='double', bounds=dict(min=0.0, max=70 * np.pi / 180)),
            dict(name='r2', type='double', bounds=dict(min=0.1, max=0.4)),
            dict(name='tita2', type='double', bounds=dict(min=-np.pi / 4, max=np.pi / 4)),
        ]
    else:
        print('Nombre de la tarea no válido')
    return parameters


def sigopt_assignments(task: str, assignments: dict):
    params = []
    if task == 'pick_and_place':
        params = np.array([assignments['x1'],
                           assignments['y1'],
                           assignments['z1'],
                           assignments['x2'],
                           assignments['y2'],
                           assignments['z2']])
    elif task == 'push_button':
        params = np.array([assignments['x'],
                           assignments['y'],
                           assignments['z'],
                           assignments['alpha'],
                           assignments['beta'],
                           assignments['gamma']])
    elif task == 'three_obstacles':
        params = np.array([assignments['r1'],
                           assignments['tita1'],
                           assignments['r2'],
                           assignments['tita2']])
    else:
        print('Nombre de la tarea no válido')
    return params
