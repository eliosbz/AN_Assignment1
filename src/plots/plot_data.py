"""
Implement here your plotting functions
Below you can see a print function example.
You should use it as a reference to implements your own plotting function

IMPORTANT: if you need you can and should use other matplotlib functionalities! Use
            the following example only as a reference

The plot workflow is can be summarized as follows:

    1) Extensive simulations
    2) Json file containing results
    3) Compute averages and stds for each metric for each algorithm
    4) Plot the results

In order to maintain the code tidy you can use:

    - src.plots.config.py file to store all the parameters you need to
        get wonderful plots (see the file for an example)

    - src.plots.data.data_elaboration.py file to write the functions that compute averages and stds from json
        result files

    - src.plots.plot_data.py file to make the plots.

The script plot_data.py can be run using python -m src.plots.plot_data

"""

# ***EXAMPLE*** #
import matplotlib.pyplot as plt
import numpy as np
from src.experiments.json_and_plot import ALL_SIZE
from src.plots.config import ALGOS, METRICS_OF_INTEREST, PLOT_DICT, LABEL_SIZE, LEGEND_SIZE
from src.plots.data.data_elaboration import compute_data_avg_std


def plot(algorithms: list,
         y_data: list,
         y_data_std: list,
         type: str):
    """
    This method has the ONLY responsibility to plot data
    @param y_data_std:
    @param y_data:
    @param algorithm:
    @param type:
    @return:
    """

    fig, ax1 = plt.subplots(nrows=1, ncols=1, figsize=(8.5, 6.5))

    print(f"Algorithm: {algorithms}")

    print(f"y_data: {y_data}\ny_data_std: {y_data_std}")

    for algorithm, y_d, y_d_s in zip(algorithms, y_data, y_data_std):
        print(f"Algorithm: {algorithm}")
        print(f"y_d: {y_d}\ny_d_s: {y_d_s}")
        ax1.errorbar(x=np.array(PLOT_DICT[algorithm]["x_ticks_positions"]),
                    y=y_d,
                    yerr=y_d_s,
                    label=PLOT_DICT[algorithm]["label"],
                    marker=PLOT_DICT[algorithm]["markers"],
                    linestyle=PLOT_DICT[algorithm]["linestyle"],
                    color=PLOT_DICT[algorithm]["color"],
                    markersize=8)

    ax1.set_ylabel(ylabel=str(type).replace("_", " "), fontsize=LABEL_SIZE)
    ax1.set_xlabel(xlabel="UAVs", fontsize=LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=ALL_SIZE)

    plt.legend(ncol=1,
               handletextpad=0.1,
               columnspacing=0.7,
               prop={'size': LEGEND_SIZE})

    plt.grid(linewidth=0.3)
    plt.tight_layout()
    plt.savefig("src/plots/figures/" + type + ".svg")
    plt.savefig("src/plots/figures/" + type + ".png", dpi=400)
    plt.clf()

# ***EXAMPLE*** #

# TODO: Implement your code HERE

if __name__ == "__main__":
    """
    Run this file to get the plots.
    Of course, since you need to plot more than a single data series (one for each algorithm) you need to modify
    plot() in a way that it can handle a multi-dimensional data (one data series for each algorithm). 
    y_data and y_data_std could be for example a list of lists o a dictionary containing lists. It up to you to decide
    how to deal with data
    """

    # ***EXAMPLE***

    # you can call the compute_data_avg_std in data_elaboration function here to get all the data you need
    # in this example that function is "approximated" using np.linspace()

    path = "data/evaluation_tests"

    data = compute_data_avg_std(path=path)

    y_data, y_data_std =[], []
    for metric in METRICS_OF_INTEREST:
        metric_data, metric_data_std = [], []
        for algorithm in ALGOS:
            algo_data, algo_std = [], []
            for nd in np.array(PLOT_DICT[algorithm]["x_ticks_positions"]):
                algo_data.append(data[algorithm][metric]["num_drones"][nd]["mean"])
                algo_std.append(data[algorithm][metric]["num_drones"][nd]["std"])
            metric_data.append(algo_data)
            metric_data_std.append(algo_std)
        y_data.append(metric_data)
        y_data_std.append(metric_data_std)

    print(f"y_data: {y_data}\ny_data_std: {y_data_std}")

    for i, metric in enumerate(METRICS_OF_INTEREST):
        plot(algorithms=ALGOS, y_data=y_data[i], y_data_std=y_data_std[i], type=metric)

    # ***EXAMPLE***

    # TODO: Implement your code HERE