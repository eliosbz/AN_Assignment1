"""
You can write here the data elaboration function/s

You should read all the JSON files containing simulations results and compute
average and std of all the metrics of interest.

You can find the JSON file from the simulations into the data.evaluation_tests folder.
Each JSON file follows the naming convention: simulation-current date-simulation id__seed_drones number_routing algorithm

In this way you can parse the name and properly aggregate the data.

To aggregate data you can use also external libraries such as Pandas!

IMPORTANT: Both averages and stds must be computed over different seeds for the same metric!
"""
import numpy as np
import os
import json
from src.plots.config import ALGOS, NUM_DRONES, METRICS_OF_INTEREST

def compute_data_avg_std(path: str):
    """
    Computes averages and stds from JSON files
    @param path: results folder path
    @return: one or more data structure containing data
    """

    # TODO: Implement your code HERE
    results = {}
    for algo in ALGOS:
        print("Processing algorithm: ", algo)
        results[algo] = {}
        for metric in METRICS_OF_INTEREST:
            results[algo][metric] = {}
            print("Processing metric: ", metric)
            results[algo][metric]["num_drones"] = {}
            for nd in NUM_DRONES:
                results[algo][metric]["num_drones"][nd] = {}
                print("Processing nd: ", nd)
                data = []
                for file in os.listdir(path):
                    if file.endswith(".json") and file.split(".")[1] == algo and file.split("_")[3] == str(nd):
                            with open(os.path.join(path, file), 'r') as f:
                                js = json.load(f)
                                data.append(js[metric])
                results[algo][metric]["num_drones"][nd]["mean"] = np.mean(data)
                results[algo][metric]["num_drones"][nd]["std"] = np.std(data)
                #print(results)
    #print(results)
    return results


if __name__ == "__main__":
    """
    You can run this file to test your script
    """
    path = "data/evaluation_tests"

    compute_data_avg_std(path=path)