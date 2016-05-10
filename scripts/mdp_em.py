from read_config import read_config
import numpy
import sys
import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt

class MarkovDecisionProcessWithEM:
    ACTION_NORTH = 'N'
    ACTION_SOUTH = 'S'
    ACTION_WEST = 'W'
    ACTION_EAST = 'E'

    ACTION_LIST = [ACTION_NORTH, ACTION_SOUTH, ACTION_WEST, ACTION_EAST]

    def __init__(self):
        self.init_config()
        self.init_map()


    def init_config(self):
        self.config = read_config()

        self.grid_size = self.config["mdp_grid_size"]

        self.rew_step = self.config["reward_for_each_step"]
        self.rew_wall = self.config["reward_for_hitting_wall"]
        self.rew_pit = self.config["reward_for_falling_in_pit"]
        self.rew_goal = self.config["reward_for_reaching_goal"]

        self.discount_factor = self.config["discount_factor"]

        self.prob_forward = self.config["prob_move_forward"]
        self.prob_backward = self.config["prob_move_backward"]
        self.prob_left = self.config["prob_move_left"]
        self.prob_right = self.config["prob_move_right"]

        self.mdp_goal = self.config["mdp_goal"]
        self.mdp_walls = self.config["mdp_walls"]
        self.mdp_pits = self.config["mdp_pits"]


    def init_map(self):
        """
        Initialize Map with traversable grids(0), walls(1) and pits(-1)
        """
        self.map = numpy.zeros((self.grid_size[0], self.grid_size[1])).tolist()
        for wall in self.mdp_walls:
            wall_x, wall_y = wall
            #Marking walls
            self.map[wall_x][wall_y] = 1

        for pit in self.mdp_pits:
            pit_x, pit_y = pit
            #Marking pits
            self.map[pit_x][pit_y] = -1

        self.map[self.mdp_goal[0]][self.mdp_goal[1]] = 100

    def create_utility_path(self):
        """
        Utility path for the whole grid. This is consumed during value iteration
        """
        utility_path = []
        for row in range(len(self.map)):
            row_path = []
            for col in range(len(self.map[0])):
                row_path.append(0)
            utility_path.append(row_path)
        return utility_path

    def print_2d_array(self, input_array):
        """
        Print 2d array
        """
        for row in range(len(input_array)):
            row_content = []
            for col in range(len(input_array[0])):
                val = input_array[row][col]
                if isinstance(val, float):
                    val = round(val, 3)
                row_content.append(val)
            print ''.join(['{:10}'.format(item) for item in row_content])
        #print('\n'.join([''.join(['{:8}'.format(item) for item in row]) for row in self.map]))


    def transition_function_based_on_action(self, current_position, action):
        """
        Given a current position and action (N, S, W, E), give the next position based on the action.
        End of grid, walls taken into account in this
        """
        current_position_x, current_position_y = current_position
        if action == self.ACTION_NORTH:
            new_position_x = max(0, current_position_x - 1)
            new_position_y = current_position_y

        elif action == self.ACTION_SOUTH:
            new_position_x = min(len(self.map) - 1 , current_position_x + 1)
            new_position_y = current_position_y

        elif action == self.ACTION_WEST:
            new_position_x = current_position_x
            new_position_y = max(0, current_position_y - 1)

        elif action == self.ACTION_EAST:
            new_position_x = current_position_x
            new_position_y = min(len(self.map[0]) - 1, current_position_y + 1)

        if self.map[new_position_x][new_position_y] == 1:
            return current_position

        return [new_position_x, new_position_y]


    def get_probabilities_for_action(self, action):
        """
        For each of the action, calculate the probabilities based on forward, backward, left, right probabilities,
         which are provided in the configuration
        """
        if action == self.ACTION_NORTH:
            return {
                self.ACTION_NORTH: self.prob_forward,
                self.ACTION_SOUTH: self.prob_backward,
                self.ACTION_WEST: self.prob_left,
                self.ACTION_EAST: self.prob_right
            }

        elif action == self.ACTION_SOUTH:
            return {
                self.ACTION_NORTH: self.prob_backward,
                self.ACTION_SOUTH: self.prob_forward,
                self.ACTION_WEST: self.prob_right,
                self.ACTION_EAST: self.prob_left
            }

        elif action == self.ACTION_WEST:
            return {
                self.ACTION_NORTH: self.prob_right,
                self.ACTION_SOUTH: self.prob_left,
                self.ACTION_WEST: self.prob_forward,
                self.ACTION_EAST: self.prob_backward
            }

        elif action == self.ACTION_EAST:
            return {
                self.ACTION_NORTH: self.prob_left,
                self.ACTION_SOUTH: self.prob_right,
                self.ACTION_WEST: self.prob_backward,
                self.ACTION_EAST: self.prob_forward
            }

    def get_reward_for_position(self, current_position):
        """
        Get reward for current position based on the configuration
        """
        current_position_x, current_position_y = current_position
        if current_position == self.mdp_goal:
            return self.rew_goal

        elif current_position in self.mdp_walls:
            return self.rew_wall

        elif current_position in self.mdp_pits:
            return self.rew_pit

        elif self.map[current_position_x][current_position_y] == 0:
            return self.rew_step

    def get_possible_destinations_on_action_with_probability(self, current_position, action):
        """
        For a given position and action, calculate the probabilities and return along with the probabilities
        for all other possible position.
        This is required for all the actions, since we compare and then pick the one which returns maximum value
        """
        if self.map[current_position[0]][current_position[1]] != 0:
            return []
        prob_for_action = self.get_probabilities_for_action(action)
        possible_destinations = []
        for action in self.ACTION_LIST:
            possible_destinations.append((action,
                                          self.transition_function_based_on_action(current_position, action),
                                          prob_for_action[action]))
        return possible_destinations


    def get_maximized_value_for_position(self, current_position):
        """
        For the given position, return the maximum value which the position can offer based on the possible destination
         positions from that position.
         Utilized in value iteration
        """
        current_position_x, current_position_y = current_position

        if self.map[current_position_x][current_position_y] == 0:
            maximum_sum = None
            for action in self.ACTION_LIST:
                current_sum = 0
                possible_destinations_on_action = self.get_possible_destinations_on_action_with_probability(current_position, action)
                for act, dest, prob in possible_destinations_on_action:
                    current_sum += (self.utility_path[dest[0]][dest[1]] * prob)

                if (maximum_sum is None) or (current_sum > maximum_sum):
                    maximum_sum = current_sum

            max_value = self.get_reward_for_position(current_position) + self.discount_factor * maximum_sum

        else:
            max_value = self.get_reward_for_position(current_position)

        return max_value


    def value_iteration(self):
        """
        Given the set of positions and goals and rewards, we find the optimal utility path for the grid.
        """
        self.utility_path = self.create_utility_path()
        val_epsilon = 0.1
        max_iter = 1000
        no_of_iter = 0

        while (True):
            no_of_iter += 1

            max_norm = 0
            new_utility_path = self.create_utility_path()

            for row in range(len(self.map)):
                for col in range(len(self.map[0])):
                    value = self.get_maximized_value_for_position([row, col])
                    if not value is None:
                        max_norm = max(max_norm, abs(self.utility_path[row][col] - value))
                    new_utility_path[row][col] = value

            self.utility_path = new_utility_path
            if max_norm <= val_epsilon * (1 - self.discount_factor) / self.discount_factor:
                #print "Converged in", no_of_iter, "iterations"
                break
            if no_of_iter >= max_iter:
                #does not converge
                break
        #self.print_2d_array(self.utility_path)
        return self.utility_path

    def get_utility_for_action(self, position, action = None):
        """
        Given position and action, calculate the utility considering the reward function and the utility of the
            destination position based on the action
        """
        if self.map[position[0]][position[1]] != 0:
            return None
        if action is None:
            result = {}
            for action in self.ACTION_LIST:
                result[action] = self.get_utility_for_action(position, action)
        else:
            curr_sum = 0
            dests = self.get_possible_destinations_on_action_with_probability(position, action)
            for act, dest, prob in dests:
                curr_sum += (self.utility_path[dest[0]][dest[1]] * prob)
            result = self.get_reward_for_position(position) + self.discount_factor * curr_sum

        return result

    def get_action_based_on_utility_values(self, position):
        """
        Maximization done here. Return the position which has the maximum utility
        """
        def argmax_utility_values(position):
            qv = self.get_utility_for_action(position)
            return (max(qv.items(), key = lambda c: c[1])[0] if qv else None)
        return argmax_utility_values(position)


    def get_action_from_utility_path(self):
        """
        From the final utility path, derive the optimal path and then print the map
        """
        if not hasattr(self, 'utility_path'):
            print "Perform Value Iteration"
            exit()

        final_action_list = []
        for row in range(len(self.map)):
            row_list = []
            for col in range(len(self.map[0])):
                action = self.get_action_based_on_utility_values([row, col])
                if [row, col] == self.mdp_goal:
                    row_list.append("EXIT")
                    sys.stdout.write(u"\u2605" + "\t")

                elif [row, col] in self.mdp_walls:
                    row_list.append("WALL")
                    sys.stdout.write(u"\u25a2" + "\t")
                else:
                    if action == self.ACTION_NORTH:
                        sys.stdout.write(u"\u25b2" + "\t")
                    elif action == self.ACTION_SOUTH:
                        sys.stdout.write(u"\u25bc" + "\t")
                    elif action == self.ACTION_WEST:
                        sys.stdout.write(u"\u25c0" + "\t")
                    elif action == self.ACTION_EAST:
                        sys.stdout.write(u"\u25b6" + "\t")
                    else:
                        sys.stdout.write(u"\u00d7" + "\t")
                    row_list.append(action)
            final_action_list.append(row_list)
            sys.stdout.write("\n")
        #self.print_2d_array(final_action_list)

    def print_map(self):
        for row in range(len(self.map)):
            for col in range(len(self.map[0])):
                if [row, col] == self.mdp_goal:
                    sys.stdout.write(u"\u2605" + "\t")
                elif [row, col] in self.mdp_walls:
                    sys.stdout.write(u"\u25a2" + "\t")
                elif [row, col] in self.mdp_pits:
                    sys.stdout.write(u"\u00d7" + "\t")
                else:
                    sys.stdout.write(u"\u003f" + "\t")
            sys.stdout.write("\n")



em_instance = MarkovDecisionProcessWithEM()
em_instance.print_map()
print "-" * 50
em_instance.value_iteration()
em_instance.get_action_from_utility_path()