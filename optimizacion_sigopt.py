from sigopt import Connection
import pick_and_place.pick_and_place_function as fun
import numpy as np
import pickle
import params_opt
from sigopt.examples import franke_function

TASK_DIR = "pick_and_place/"
# coords_type = 'esfericas'
VARIATION = "1container"
TASK_NAME = "pick_and_place"  # + "_" + coords_type

SIGOPT_API_TOKEN = "AVRITBIJFQBNAZWRCPLCZLODWFKLNGMPJPFXLXYZLTJVHOSE"
conn = Connection(client_token=SIGOPT_API_TOKEN)

experiment = conn.experiments().create(
    name='Optimizacion Pick And Place',
    # Define which parameters you would like to tune
    parameters=params_opt.sigopt_parameters(task=TASK_NAME, variation=VARIATION),
    metrics=[dict(name='function_value', objective='minimize')],
    parallel_bandwidth=1,
    # Define an Observation Budget for your experiment
    observation_budget=300,
    project="sigopt-examples",
)
print("Created experiment: https://app.sigopt.com/experiment/" + experiment.id)

listas = []
param_solution = []
function = fun.PickAndPlace(headless_mode=True, variation=VARIATION)  # Inicializacion


# Evaluate your model with the suggested parameter assignments
# Franke function - http://www.sfu.ca/~ssurjano/franke2d.html
def evaluate_model(assignments):
    params = np.array([assignments['x1'],
                       assignments['y1'],
                       assignments['z1'],
                       assignments['x2'],
                       assignments['y2'],
                       assignments['z2']]
                      )
    return function.pick_and_place(wp_params=params)


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
param_solution = np.array([best_assignments['x1'],
                           best_assignments['y1'],
                           best_assignments['z1'],
                           best_assignments['x2'],
                           best_assignments['y2'],
                           best_assignments['z2']]
                          )

pickle.dump(listas, open(TASK_DIR + "listas_sigopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_sigopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
