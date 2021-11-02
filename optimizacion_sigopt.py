from sigopt import Connection
import pick_and_place.pick_and_place_function as fun
import pickle
import params_opt
# from sigopt.examples import franke_function

TASK_DIR = "three_obstacles/"
# coords_type = 'esfericas'
VARIATION = "three_obstacles"
TASK_NAME = "pick_and_place"

SIGOPT_API_TOKEN = #
conn = Connection(client_token=SIGOPT_API_TOKEN)

listas = []
param_solution = []
n_experiments = 5

function = fun.PickAndPlace(headless_mode=True, variation=VARIATION)  # Inicializacion
# function.set_coords(coords=VARIATION)


# Evaluate your model with the suggested parameter assignments
def evaluate_model(assignments):
    params = params_opt.sigopt_assignments(task=TASK_NAME, assignments=assignments)
    return function.pick_and_place(wp_params=params)


for i in range(n_experiments):
    function.clean_lists()
    experiment = conn.experiments().create(
        name='Optimizacion Pick and Place 2var ' + str(i),
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
    solution = params_opt.sigopt_assignments(task=TASK_NAME, assignments=best_assignments)

    pickle.dump(listas_optimizacion, open(TASK_DIR + "listas_sigopt_" + TASK_NAME + "_" + VARIATION + "_" + str(i)
                                          + ".p", "wb"))
    pickle.dump(solution, open(TASK_DIR + "solucion_sigopt_" + TASK_NAME + "_" + VARIATION + "_" + str(i) + ".p", "wb"))

    listas.append(listas_optimizacion)
    param_solution.append(solution)

pickle.dump(listas, open(TASK_DIR + "listas_sigopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_sigopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
