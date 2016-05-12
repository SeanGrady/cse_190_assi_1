from read_config import read_config
import numpy as np

class AStarSearch:
    def __init__(self):
        self.config = read_config()

        self.start = (self.config["start"][0], self.config["start"][1])
        self.goal = (self.config["goal"][0], self.config["goal"][1])
        self.map_size = self.config["map_size"]
        self.walls = self.config["walls"]
        self.pits = self.config["pits"]

        self.init_map()

        self.validate_start_and_goal()

        self.move_list = self.config["move_list"]

        self.closedset = set()
        self.openset = set()

        #cost from start position to current position
        self.g_score = {}
        #cost from current position to goal
        self.f_score = {}

        #prev position to track back
        self.prev_position = {}


        self.init_scores()

    def init_map(self):
        self.map = np.zeros(self.map_size)
        self.map.fill(255)

        for row in range(len(self.map)):
            for col in range(len(self.map[0])):
                if [row, col] in self.walls or [row, col] in self.pits:
                    #Marking as not traversable
                    self.map[row][col] = 0

    def init_scores(self):
        """ Initialize the f_score and g_score objects. Add starting position to the openset """

        self.g_score[self.start] = 0
        self.f_score[self.start] = self.g_score[self.start] + self.heuristic_cost_estimate(self.start, self.goal)
        self.openset.add(self.start)

    def validate_start_and_goal(self):
        """ Validate the start and end provided to the algorithm. Exit if its invalid """
        valid = 1
        if self.map[self.start] == 0:
            print "Cannot start from obstacle"
            valid = 0
        if self.map[self.goal] == 0:
            print "Goal cannot be at obstacle"
            valid = 0
        if valid == 0:
            print "Invalid Scenario.\nSleeping..."
            exit()


    def heuristic_cost_estimate(self, start_pos, end_pos):
        """ Heuristic cost estimate using manhattan distance as the metric """
        start_x, start_y = start_pos
        end_x, end_y = end_pos

        x = start_x - end_x
        y = start_y - end_y
        return (abs(x) + abs(y))


    def retrace_path(self, current_pos):
        """ Retrace path recursively using the prev_position object """
        try:
            path_from_prev_pos = self.retrace_path(self.prev_position[current_pos])
            return_path = []
            return_path.extend(path_from_prev_pos)
            return_path.append([current_pos[0], current_pos[1]])
            return return_path

        except KeyError, e:
            # we have reached the start node
            return [[current_pos[0], current_pos[1]]]

    def get_neighbour_positions(self, current_pos):
        """ Based on the allowed position, calculate the neighboring positions """

        neighbours = set()
        pos_x, pos_y = current_pos

        for move in self.move_list:
            move_x, move_y = move

            new_pos_x = pos_x + move_x
            new_pos_y = pos_y + move_y

            if new_pos_x >= 0 and new_pos_x < self.map_size[1] and new_pos_y >= 0 and new_pos_y < self.map_size[0]:
                if self.map[new_pos_x, new_pos_y] != 0:
                    neighbours.add((new_pos_x, new_pos_y))

        return neighbours



    def a_start_search(self):
        """ Start searching based on the given map, start and goal using A-star search """

        while len(self.openset) > 0:
            #print len(self.openset)
            #print self.openset
            #sort to get the lowest f_score for the ones in openset
            f_score_sorted = sorted(self.f_score, key = lambda position : self.g_score[position] + self.heuristic_cost_estimate(position, self.goal))

            i = 0
            for i in range(len(f_score_sorted)):
                if(f_score_sorted[i] in self.openset):
                    #position with the lowest f_score
                    self.current_pos = f_score_sorted[i]
                    break

            #reached goal, retrace path
            if self.current_pos == self.goal:
                final_path = self.retrace_path(self.current_pos)
                return final_path

            try:
                self.openset.remove(self.current_pos)
            except KeyError, e:
                pass

            self.closedset.add(self.current_pos)
            for neighbour_pos in self.get_neighbour_positions(self.current_pos):
                if neighbour_pos not in self.closedset:
                    tentative_g_score = self.g_score[self.current_pos] + 1
                    #if there is a shorter path, replace the old one
                    if (neighbour_pos not in self.openset) or (tentative_g_score < self.g_score[neighbour_pos]):
                        self.prev_position[neighbour_pos] = self.current_pos
                        self.g_score[neighbour_pos] = tentative_g_score
                        self.f_score[neighbour_pos] = self.g_score[neighbour_pos] + self.heuristic_cost_estimate(neighbour_pos, self.goal)

                        #if the neighbour is not added yet, add it
                        if neighbour_pos not in self.openset:
                            self.openset.add(neighbour_pos)

#a_star_instance = AStarSearch()
#a_star_search_path = a_star_instance.a_start_search()
#print "Reached Goal in", len(a_star_search_path), "steps"
#print a_star_search_path
