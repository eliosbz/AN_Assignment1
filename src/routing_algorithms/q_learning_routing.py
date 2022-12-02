from src.entities.uav_entities import Drone
from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
from typing import List, Dict


class QLearningRouting(BASE_routing):
    EPSILON = 0.05
    ALPHA = 0.3
    GAMMA = 0.6
    NUMBER_OF_CELL = 40
    NUMBER_OF_ACTIONS = 2
    # other discret hyperparam for few cell
    # EPSILON = 0.05
    # ALPHA = 0.99
    # GAMMA = 0.2
    # NUMBER_OF_CELL = 20
    # NUMBER_OF_ACTIONS = 2

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.q_table: Dict[int, List[int]] = {}
        for i in range(self.NUMBER_OF_CELL):
            self.q_table[i] = [0 for i in range(self.NUMBER_OF_ACTIONS)]
        self.random_gen = np.random.RandomState(self.simulator.seed)

    # def compute_quality():
    #    1/cur_step

    def TD(self, reward, max_action, Q_old):
        return reward + self.GAMMA * max_action - Q_old

    def compute_reward(self, delay, outcome, drone):
        # self.selection_count += 1 # to compute the UBC we need to keep the count of the times this action is selected and thus a reward is received

        # This is a more complex reward that reflects the goodness of the feedback
        if outcome == 1:
            # a packet is received with some delay
            reward = (
                4 * drone.buffer_length() / delay
            )  # try to exploit also the number of packet a drone has in buffer
        elif outcome == -1:
            reward = -2

        # self.total_rewards += reward # to compute the UBC we need to sum the reward
        return reward

    def feedback(self, drone, id_event, delay, outcome):
        """
        Feedback returned when the packet arrives at the depot or
        Expire. This function have to be implemented in RL-based protocols ONLY
        @param drone: The drone that holds the packet
        @param id_event: The Event id
        @param delay: packet delay
        @param outcome: -1 or 1 (read below)
        @return:
        """

        # outcome can be:
        #   -1 if the packet/event expired;
        #   1 if the packets has been delivered to the depot

        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple
        # feedback for the same packet!!

        if id_event in self.taken_actions:
            # BE AWARE, IMPLEMENT YOUR CODE WITHIN THIS IF CONDITION OTHERWISE IT WON'T WORK!
            # TIPS: implement here the q-table updating process

            # Drone id and Taken actions
            # print(f"\nIdentifier: {self.drone.identifier}, Taken Actions: {self.taken_actions}, Time Step: {self.simulator.cur_step}")

            # feedback from the environment

            # TODO: write your code here

            state, action = self.taken_actions[id_event]

            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

            # reward or update using the old state and the selected action at that time
            # do something or train the model (?)
            reward = self.compute_reward(delay, outcome, drone)

            radius_index_next_target = int(
                self.NUMBER_OF_CELL
                * util.euclidean_distance(drone.next_target(), self.drone.depot.coords)
                / util.euclidean_distance(
                    self.drone.depot.coords, (0, util.config.ENV_HEIGHT)
                )
            )

            self.q_table[state][action] += self.ALPHA * self.TD(
                reward,
                max(self.q_table[radius_index_next_target]),
                self.q_table[state][action],
            )

    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.

        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """
        # TODO: Implement your code HERE

        # A Drone can take 2 actions: send or keep the packet
        # The state is the distance between the drone and the depot

        # Only if you need!
        # cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
        #                                                width_area=self.simulator.env_width,
        #                                                x_pos=self.drone.coords[0],  # e.g. 1500
        #                                                y_pos=self.drone.coords[1])[0]  # e.g. 500
        # print(cell_index)

        state, action = None, None

        state = cell_index = int(
            self.NUMBER_OF_CELL
            * util.euclidean_distance(self.drone.coords, self.drone.depot.coords)
            / util.euclidean_distance(
                self.drone.depot.coords, (0, util.config.ENV_HEIGHT)
            )
        )

        neighbors_drones = {neighbor for hello_pkt, neighbor in opt_neighbors}

        # balance exploration and exploitation
        if self.random_gen.uniform(0, 1) < self.EPSILON:
            # print("explore")
            # explore
            # neighbors_drones.add(self.drone) # TODO: try with or without the self drone in neighbors_drones
            action = 0 if self.random_gen.uniform(0, 1) < 0.5 else 1

            relay = (
                self.drone
                if (action == 0)
                else self.simulator.rnd_routing.choice(list(neighbors_drones))
            )
        else:
            # exploit
            if self.q_table[state][0] == self.q_table[state][1]:
                # fallback to explore when don't have enough information
                # or "when you dont know what to do, do it at random"
                action = 0 if self.random_gen.uniform(0, 1) < 0.5 else 1
            else:
                action = self.q_table[state].index(max(self.q_table[state]))

            if action == 0:
                relay = self.drone
                # print("exploit - keep")
            else:
                # print("exploit - send")

                depot_max_diagonal_distance = util.euclidean_distance(
                    self.drone.depot.coords, (0, util.config.ENV_HEIGHT)
                )

                min_distance = np.sqrt(
                    util.config.ENV_HEIGHT**2 + util.config.ENV_WIDTH**2
                )
                relay = self.drone
                for drone in neighbors_drones:
                    
                    drone_target_depot_distance = util.euclidean_distance(
                        drone.next_target(), self.drone.depot.coords
                    )

                    distance = int(
                        self.NUMBER_OF_CELL
                        * drone_target_depot_distance
                        / depot_max_diagonal_distance
                    )
                    if min(min_distance, distance) < min_distance:
                        min_distance = distance
                        relay = drone

        # Store your current action --- you can add some stuff if needed to take a reward later
        self.taken_actions[packet.event_ref.identifier] = (state, action)

        return relay  # here you should return a drone object!
