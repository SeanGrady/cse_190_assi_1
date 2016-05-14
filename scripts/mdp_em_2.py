from read_config import read_config
import numpy
import sys
import image_util

class MarkovDecisionProcessWithEM:
    ACTION_NORTH = 'N'
    ACTION_SOUTH = 'S'
    ACTION_WEST = 'W'
    ACTION_EAST = 'E'

    ACTION_LIST = [ACTION_NORTH, ACTION_SOUTH, ACTION_WEST, ACTION_EAST]

    def __init__(self):
        self.init_config()
        self.init_map()
        self.utility_path = None


    def init_config(self):
        self.config = read_config()

        self.grid_size = self.config["map_size"]

        self.rew_step = self.config["reward_for_each_step"]
        self.rew_wall = self.config["reward_for_hitting_wall"]
        self.rew_pit = self.config["reward_for_falling_in_pit"]
        self.rew_goal = self.config["reward_for_reaching_goal"]

        self.discount_factor = self.config["discount_factor"]

        self.prob_forward = self.config["prob_move_forward"]
        self.prob_backward = self.config["prob_move_backward"]
        self.prob_left = self.config["prob_move_left"]
        self.prob_right = self.config["prob_move_right"]

        self.mdp_goal = self.config["goal"]
        self.mdp_walls = self.config["walls"]
        self.mdp_pits = self.config["pits"]


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

    def create_policy(self):
        """
        Policy for the whole grid. This is consumed during value iteration
        """
        policy_list = []
        for row in range(len(self.map)):
            row_policy = []
            for col in range(len(self.map[0])):
                row_policy.append("")
            policy_list.append(row_policy)
	return policy_list
 
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


    def get_maximized_value_and_action_for_position(self, current_position):
        """
        For the given position, return the maximum value which the position can offer based on the possible destination
         positions from that position and return the maximum action which is taken
         Utilized in value iteration
        """
        current_position_x, current_position_y = current_position
        maximum_sum = None
        maximum_action = None
        for action in self.ACTION_LIST:
            possible_destinations_on_action = self.get_possible_destinations_on_action_with_probability(current_position, action)

            current_sum = 0
            for act, dest, prob in possible_destinations_on_action:
                current_sum += prob * (self.get_reward_for_position(dest) + self.discount_factor * self.utility_path_list[-1][dest[0]][dest[1]])

            if (maximum_sum is None) or (current_sum > maximum_sum):
                maximum_sum = current_sum
                maximum_action = action


        return maximum_sum, maximum_action


    def value_iteration(self):
        """
        Given the set of positions and goals and rewards, we find the optimal utility path for the grid.
        """
        self.image_list = []
        self.utility_path_list = []
        result_policy_list = []
        self.utility_path_list.append(self.create_utility_path())

        max_iter = 1000
        self.no_of_iter = 0
        difference_threshold = 0.0

        while (True):
            current_utility_path = self.create_utility_path()
	    current_policy = self.create_policy()
            for row in range(len(self.map)):
                for col in range(len(self.map[0])):
                    #value iteration only for traversable paths
                    if self.map[row][col] == 0:
                        value, action = self.get_maximized_value_and_action_for_position([row, col])
                        current_policy[row][col] = action
                        current_utility_path[row][col] = value
                    elif [row, col] in self.mdp_pits:
                        current_policy[row][col] = "PIT"
                    elif [row, col] in self.mdp_walls:
                        current_policy[row][col] = "WALL"
                    elif [row, col] == self.mdp_goal:
                        current_policy[row][col] = "GOAL"

            current_value = current_utility_path
            prev_value = self.utility_path_list[-1]
            current_value_np = numpy.array(current_value)
            prev_value_np = numpy.array(prev_value)

            self.utility_path_list.append(current_utility_path)

            difference = numpy.sum(abs(current_value_np - prev_value_np))
	    
	    result_policy_list.append(current_policy)

	    self.no_of_iter += 1

            if difference <= difference_threshold:
                print "Converged in", self.no_of_iter
                break

            if self.no_of_iter >= max_iter:
                break
	
	return result_policy_list

