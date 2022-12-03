"""
Write your plot configuration script here
Below you can see just an example about how to write a config file.
You can import constants, lists and dictionaries in plot_data.py
"""

# *** EXAMPLE ***
import numpy as np
import matplotlib.pyplot as plt

LABEL_SIZE = 22
LEGEND_SIZE = 20
TITLE_SIZE = 26
TICKS_SIZE = 20
OTHER_SIZES = 20

ALGOS = ["QL", "RND", "GEO", "QL_DE", "QL2", "QL3"]
NUM_DRONES = [5, 10, 15, 20, 25, 30]

METRICS_OF_INTEREST = [
    "number_of_packets_to_depot",
    "packet_mean_delivery_time",
    "mean_number_of_relays"]

SCALE_LIM_DICT = {
    "number_of_packets_to_depot": {
        "scale": "linear",
        "ylim": (0, 1000)
    },
    "packet_mean_delivery_time": {
        "scale": "linear",
        "ylim": (0, 5)
    },
    "mean_number_of_relays": {
        "scale": "linear",
        "ylim": (0, 10)
    }
}

PLOT_DICT = {
    "QL": {
        "hatch": "",
        "markers": "X",
        "linestyle": "-",
        "color": plt.cm.tab10(0),
        "label": "QL",
        "x_ticks_positions": np.array(NUM_DRONES)
    },
    "RND": {
        "hatch": "",
        "markers": "p",
        "linestyle": "-",
        "color": plt.cm.tab10(1),
        "label": "RND",
        "x_ticks_positions": np.array(NUM_DRONES)

    },
    "GEO": {
        "hatch": "",
        "markers": "s",
        "linestyle": "-",
        "color": plt.cm.tab10(8),
        "label": "GEO",
        "x_ticks_positions": np.array(NUM_DRONES)
    },
    "QL_DE": {
        "hatch": "",
        "markers": "*",
        "linestyle": "-",
        "color": plt.cm.tab10(2),
        "label": "QL_DE",
        "x_ticks_positions": np.array(NUM_DRONES)
    },
    "QL2": {
        "hatch": "",
        "markers": "^",
        "linestyle": "-",
        "color": plt.cm.tab10(9),
        "label": "QL2",
        "x_ticks_positions": np.array(NUM_DRONES)
    },
    "QL3": {
        "hatch": "",
        "markers": "D",
        "linestyle": "-",
        "color": plt.cm.tab10(3),
        "label": "QL3",
        "x_ticks_positions": np.array(NUM_DRONES)
    }

}
# PLOT_DICT = {
#     "algo_1": {
#         "hatch": "",
#         "markers": "X",
#         "linestyle": "-",
#         "color": plt.cm.tab10(0),
#         "label": "Algo 1",
#         "x_ticks_positions": np.array(np.linspace(0, 8, 5))
#     },
#     "algo_2": {
#         "hatch": "",
#         "markers": "p",
#         "linestyle": "-",
#         "color": plt.cm.tab10(1),
#         "label": "Algo 2",
#         "x_ticks_positions": np.array(np.linspace(0, 8, 5))

#     },
#     "algo_n": {
#         "hatch": "",
#         "markers": "s",
#         "linestyle": "-",
#         "color": plt.cm.tab10(8),
#         "label": "Algo n",
#         "x_ticks_positions": np.array(np.linspace(0, 8, 5))

#     }
# }

# *** EXAMPLE ***


# TODO: Implement your code HERE