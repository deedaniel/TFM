from sigopt import Connection
import slide_object.slide_block_function as fun
import numpy as np
import pickle
import params_opt
# from sigopt.examples import franke_function

TASK_DIR = "slide_object/"
# coords_type = 'esfericas'
VARIATION = "2block"
TASK_NAME = "slide_block"  # + "_" + coords_type

SIGOPT_API_TOKEN = "AVRITBIJFQBNAZWRCPLCZLODWFKLNGMPJPFXLXYZLTJVHOSE"
conn = Connection(client_token=SIGOPT_API_TOKEN)

listas = []
param_solution = []
n_experiments = 5

function = fun.SlideBlock(headless_mode=True, variation=VARIATION)  # Inicializacion


# Evaluate your model with the suggested parameter assignments
def evaluate_model(assignments):
    params = np.array([assignments['x1'],
                       assignments['y1'],
                       assignments['z1'],
                       assignments['d'],
                       assignments['phi']])
    return function.slide_block(slide_params=params)


for i in range(n_experiments):
    function.clean_lists()
    experiment = conn.experiments().create(
        name='Optimizacion Slide Block ' + str(i),
        # Define which parameters you would like to tune
        parameters=params_opt.sigopt_parameters(task=TASK_NAME, variation=VARIATION),
        metrics=[dict(name='function_value', objective='minimize')],
        parallel_bandwidth=1,
        # Define an Observation Budget for your experiment
        observation_budget=300,
        project="sigopt-examples",
    )
    print("Created experiment: https://app.sigopt.com/experiment/" + experiment.id)

    # Run the Optimization Loop until the Observation Budget is exhausted
    while experiment.progress.observation_count < experiment.observation_budget:
        suggestion = conn.experiments(experiment.id).suggestions().create()
        value = evaluate_model(suggestion.assignments)
        conn.experiments(experiment.id).observations().create(
            suggestion=suggestion.id,
            value=value,
        )

        # Update the experiment object
        experiment = conn.experiments(experiment.id).fetch()

    # Fetch the best configuration and explore your experiment
    all_best_assignments = conn.experiments(experiment.id).best_assignments().fetch()
    # Returns a list of dict-like Observation objects
    best_assignments = all_best_assignments.data[0].assignments
    print("Best Assignments: " + str(best_assignments))
    print("Explore your experiment: https://app.sigopt.com/experiment/" + experiment.id + "/analysis")

    listas_optimizacion = function.return_lists()
    solution = np.array([best_assignments['x1'],
                         best_assignments['y1'],
                         best_assignments['z1'],
                         best_assignments['d'],
                         best_assignments['phi']])

    listas.append(listas_optimizacion)
    param_solution.append(solution)

pickle.dump(listas, open(TASK_DIR + "listas_sigopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_sigopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
