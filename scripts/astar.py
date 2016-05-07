import map_utils as mu
from read_config import read_config

class AStarSearch:
    def __init__(self):
        self.config = read_config()
        self.path_to_world_map = self.config["path_to_world_map"]
        self.start = (self.config["start"][0], self.config["start"][1])
        self.goal = (self.config["goal"][0], self.config["goal"][1])
        self.init_map()

        self.validate_start_and_goal()

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
        self.map = mu.convert_map_png_to_2d_array(self.path_to_world_map)

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
        return abs(x) +  abs(y)


    def retrace_path(self, current_pos):
        """ Retrace path recursively using the prev_position object """
        try:
            path_from_prev_pos = self.retrace_path(self.prev_position[current_pos])
            return_path = []
            return_path.extend(path_from_prev_pos)
            return_path.append(current_pos)
            return return_path

        except KeyError, e:
            # we have reached the start node
            return [current_pos]

    def get_neighbour_positions(self, current_pos):
        """ Based on the allowed position, calculate the neighboring positions """

        neighbours = set()

        pos_x, pos_y = current_pos

        north = (pos_x, pos_y - 1)
        south = (pos_x, pos_y + 1)
        east = (pos_x + 1, pos_y)
        west = (pos_x - 1, pos_y)

        if self.map[north] != 0:
            neighbours.add(north)
        if self.map[south] != 0:
            neighbours.add(south)
        if self.map[east] != 0:
            neighbours.add(east)
        if self.map[west] != 0:
            neighbours.add(west)

        return neighbours



    def a_start_search(self):
        """ Start searching based on the given map, start and goal using A-star search """

        while len(self.openset) > 0:
            #print len(self.openset)
            #sort to get the lowest f_score for the ones in openset
            f_score_sorted = sorted(self.f_score, key = lambda position : self.g_score[position] + self.heuristic_cost_estimate(position, self.goal))

            i = 0
            for i in range(len(f_score_sorted) - 1):
                if(f_score_sorted[i] not in self.closedset):
                    break
            #position with the lowest f_score
            self.current_pos = f_score_sorted[i]

            #reached goal, retrace path
            if self.current_pos == self.goal:
                final_path = self.retrace_path(self.current_pos)
                return final_path

            try:
                self.openset.remove(self.current_pos)
            except KeyError,e:
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

a_star_instance = AStarSearch()
a_star_search_path = a_star_instance.a_start_search()
print "Reached Goal in", len(a_star_search_path), "steps"
mu.draw_path_on_world_map_save(a_star_instance.path_to_world_map, a_star_search_path)
