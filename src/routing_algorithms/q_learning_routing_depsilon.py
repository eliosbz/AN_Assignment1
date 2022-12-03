from src.entities.uav_entities import Drone
from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
from typing import List, Dict


class QLearningRouting_DEpsilon(BASE_routing):
    MAX_EPSILON = 1.0  # Initial exploration probability
    MIN_EPSILON = 0.015  # End exploration probability
    DECAY_RATE = 0.005  # Exponential decay rate of the exploration probability
    ALPHA = 0.3
    GAMMA = 0.6
    NUMBER_OF_CELL = 26
    NUMBER_OF_SECTOR = 10
    NUMBER_OF_ACTIONS = 2

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.q_table: Dict[(int, int), List[int]] = {}
        for i in range(self.NUMBER_OF_CELL):
            for j in range(self.NUMBER_OF_SECTOR):
                self.q_table[(i, j)] = np.array(
                    [0.0 for i in range(self.NUMBER_OF_ACTIONS)]
                )
        self.random_gen = np.random.RandomState(self.simulator.seed)

    def TD(self, reward, max_action, Q_old):
        # TODO: transform this td0 in a td_lambda, because maybe monte carlo learning is better for this env
        return reward + self.GAMMA * max_action - Q_old

    def compute_reward(self, delay, outcome, drone):

        # This is a more complex reward that reflects the goodness of the feedback
        if outcome == 1:
            # a packet is received with some delay
            reward = (
                2 * drone.buffer_length() / delay
            )  # try to exploit also the number of packet a drone has in buffer
        elif outcome == -1:
            reward = -2

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
            # print(drone, id_event, delay, outcome)

            # TODO: write your code here

            state, action = self.taken_actions[id_event]

            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

            # reward or update using the old state and the selected action at that time
            # do something or train the model (?)
            reward = self.compute_reward(delay, outcome, drone)
            x, y = self.drone.coords
            d = util.euclidean_distance(self.drone.coords, self.drone.depot.coords)
            alpha = np.arccos(y / d)
            x_dep, y_dep = self.drone.depot.coords

            if x > x_dep:
                alpha + np.pi / 2
            sector_index = int(self.NUMBER_OF_SECTOR * alpha / np.pi)
            radius_index_next_target = int(
                self.NUMBER_OF_CELL
                * util.euclidean_distance(
                    self.drone.next_target(), self.drone.depot.coords
                )
                / util.euclidean_distance(
                    self.drone.depot.coords, (0, util.config.ENV_HEIGHT)
                )
            )

            self.q_table[state][action] += self.ALPHA * self.TD(
                reward,
                np.max(self.q_table[(radius_index_next_target, sector_index)]),
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
        # that calculate at what radius the drone is and the sector
        # is given by transformation in polar coordinates

        state, action = None, None

        radius_index = int(
            self.NUMBER_OF_CELL
            * util.euclidean_distance(self.drone.coords, self.drone.depot.coords)
            / util.euclidean_distance(
                self.drone.depot.coords, (0, util.config.ENV_HEIGHT)
            )
        )

        x, y = self.drone.coords
        d = util.euclidean_distance(self.drone.coords, self.drone.depot.coords)
        alpha = np.arccos(y / d)
        x_dep, y_dep = self.drone.depot.coords

        if x > x_dep:
            alpha + np.pi / 2
        sector_index = int(self.NUMBER_OF_SECTOR * alpha / np.pi)

        state = (radius_index, sector_index)

        neighbors_drones = {neighbor for hello_pkt, neighbor in opt_neighbors}
        neighbors_drones.add(self.drone)

        # balance exploration and exploitation
        # with an exponential decaying epsilon value
        epsilon = self.MIN_EPSILON + (self.MAX_EPSILON - self.MIN_EPSILON) * np.exp(
            -self.DECAY_RATE * self.simulator.cur_step
        )
        ##print(epsilon)
        if self.random_gen.uniform(0, 1) < epsilon:
            # print("explore")
            # explore
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
                action = np.argmax(
                    self.q_table[state]
                )  # np.where(self.q_table[state] == (np.max(self.q_table[state])))[0]
            self.taken_actions[packet.event_ref.identifier] = (state, action)

            if action == 0:
                return self.drone
            # print("exploit - keep")
            else:
                # print("exploit - send")
                min_distance = np.sqrt(
                    util.config.ENV_HEIGHT**2 + util.config.ENV_WIDTH**2
                )
                relay = self.drone

                for drone in neighbors_drones:

                    drone_pos_depot_distance = util.euclidean_distance(
                        drone.coords, self.drone.depot.coords
                    )
                    drone_target_depot_distance = util.euclidean_distance(
                        drone.next_target(), self.drone.depot.coords
                    )
                    depot_max_diagonal_distance = util.euclidean_distance(
                        self.drone.depot.coords,
                        (0, util.config.ENV_HEIGHT),
                    )

                    if (
                        drone_target_depot_distance
                        <= util.config.DEPOT_COMMUNICATION_RANGE
                        or drone_pos_depot_distance
                        <= util.config.DEPOT_COMMUNICATION_RANGE
                    ):
                        relay = drone
                        return relay
                    else:
                        # drone trajectory-depot range intersection
                        c1, c2 = drone.depot.coords
                        x1, y1 = drone.coords
                        x2, y2 = drone.next_target()

                        x1 = x1 - c1
                        x2 = x2 - c1
                        dx = x2 - x1
                        dy = y2 - y1
                        dr2 = dx**2 + dy**2
                        D = x1 * y2 - x2 * y1
                        delta = (
                            util.config.DEPOT_COMMUNICATION_RANGE**2 * dr2 - D**2
                        )
                        if (
                            delta >= 0  # there is an intersection
                            and dy <= 0  # the drone is going down
                            # and (
                            #     (x1 <= 0 and dx >= 0) or (x1 >= 0 and dx <= 0)
                            # )  # the drone is going
                        ):
                            # rx1 = D * dy + (1 if dy >= 0 else -1) * dx * np.sqrt(delta) / dr2
                            # rx2 = D * dy - (1 if dy >= 0 else -1) * dx * np.sqrt(delta) / dr2
                            ry1 = -D * dx + abs(dy) * np.sqrt(delta) / dr2
                            ry2 = -D * dx - abs(dy) * np.sqrt(delta) / dr2
                            if ry1 >= 0 or ry2 >= 0:
                                relay = drone
                            else:
                                distance = int(
                                    self.NUMBER_OF_CELL
                                    * drone_target_depot_distance
                                    / depot_max_diagonal_distance
                                )
                                if min(min_distance, distance) < min_distance:
                                    min_distance = distance
                                    relay = drone
                        else:
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
