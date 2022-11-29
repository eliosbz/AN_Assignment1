import numpy as np

from numpy.random import Generator, PCG64
from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util

class QLearningRouting2(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.cells_number = int(self.simulator.env_width / self.simulator.prob_size_cell) ** 2
        self.epsilon = 0.1
        self.alpha = 0.9
        self.gamma = 0.2
        self.actions_number = 2
        self.q_table = np.zeros((self.cells_number, self.actions_number), dtype=float)
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.rng = np.random.default_rng(self.simulator.seed)
        
    def feedback(self, drone, id_event, delay, outcome):
        """
        Feedback returned when the packet arrives at the depot or expires.
        This function have to be implemented in RL-based protocols ONLY
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
            #print(f"\nIdentifier: {self.drone.identifier}, Taken Actions: {self.taken_actions}, Time Step: {self.simulator.cur_step}")

            # feedback from the environment
            #print(drone, id_event, delay, outcome)

            # TODO: write your code here

            state, action = self.taken_actions[id_event]

            #compute reward - DA MIGLIORARE
            reward = outcome
            next_state = int(util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                               width_area=self.simulator.env_width,
                                                               x_pos=self.drone.next_target()[0],
                                                               y_pos=self.drone.next_target()[1])[0])

            self.q_table[state, action] += self.alpha * (reward + (self.gamma * self.q_table[next_state].max()) - self.q_table[state, action])

            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

            # reward or update using the old state and the selected action at that time
            # do something or train the model (?)

    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.

        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """
        # TODO: Implement your code HERE

        # Only if you need!
        cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                        width_area=self.simulator.env_width,
                                                        x_pos=self.drone.coords[0],  # e.g. 1500
                                                        y_pos=self.drone.coords[1])[0]  # e.g. 500
        #print(cell_index)
        state, action = None, None
        relay = None

        state = int(cell_index)
        neighbors_drones = [v[1] for v in opt_neighbors]

        if self.rng.random() < self.epsilon:
            #explore
            action = 0 if self.rng.random() < 0.5 else 1

            if action == 0: relay = self.drone
            else: relay = self.simulator.rnd_routing.choice(neighbors_drones)
        else:
            #exploit
            action = np.argmax(self.q_table[state])

            if action == 0: relay = self.drone
            else:
                closest_drone = (self.drone, util.euclidean_distance(self.drone.next_target(), self.drone.depot.coords))

                for drone in neighbors_drones:
                    distance = util.euclidean_distance(drone.next_target(), self.drone.depot.coords)
                    
                    if distance < closest_drone[1]:
                        closest_drone = (drone, distance)

                relay = closest_drone[0]
        
        #print(self.drone.identifier)

        # Store your current action --- you can add some stuff if needed to take a reward later
        self.taken_actions[packet.event_ref.identifier] = (state, action)

        #print(self.simulator.env_width)

        return relay  # here you should return a drone object!