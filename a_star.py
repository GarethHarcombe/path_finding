import re
from search import *
import math
import heapq


class RoutingGraph(Graph):    
    def __init__(self, map_str):
        self.obstacles = []
        self.goals = []
        self.fuel = []
        self.agents = []
        self.map_str = []
        for i, line in enumerate(map_str.split("\n")):
            line = line.strip()
            self.map_str.append(line)
            for j in range(len(line)):
                re_int = re.compile(r'^[0-9]+$')
                if line[j] == "X" or line[j] == "-" or line[j] == "|":
                    self.obstacles.append((i, j))
                elif line[j] == "G":
                    self.goals.append((i,j))
                elif line[j] == "F":
                    self.fuel.append((i, j))
                elif line[j] == "S":
                    self.agents.append((i, j, math.inf))
                elif re_int.match(line[j]):
                    self.agents.append((i, j, int(line[j])))
                    

    def starting_nodes(self):
        return self.agents

    def is_goal(self, node):
        for goal in self.goals:
            if goal[0] == node[0] and goal[1] == node[1]:
                return True
            
        return False
        

    def outgoing_arcs(self, tail_node):
        outgoing = []
        
        if tail_node[2] > 0:
            cards = [('N' , -1, 0),
                     ('E' ,  0, 1),
                     ('S' ,  1, 0),
                     ('W' ,  0, -1)]
            
            for card in cards:
                if (tail_node[0] + card[1], tail_node[1] + card[2]) not in self.obstacles:
                    outgoing.append(Arc(tail_node, 
                                        (tail_node[0] + card[1], tail_node[1] + card[2], tail_node[2] - 1),
                                        card[0], 5))
            
        if tail_node[2] != math.inf and tail_node[2] < 9 and (tail_node[0], tail_node[1]) in self.fuel:
            outgoing.append(Arc(tail_node, (tail_node[0], tail_node[1], 9), "Fuel up", 15))

        return outgoing    
    
    def estimated_cost_to_goal(self, node):
        if len(self.goals) > 0:
            return 5 * min([abs(goal[0] - node[0]) + abs(goal[1] - node[1]) for goal in self.goals])
        else:
            return 0


class AStarFrontier(Frontier):
    def __init__(self, map_graph):
        """The constructor takes no argument. It initialises the
        container to an empty stack."""
        self.container = []
        self.expanded = set()
        self.map_graph = map_graph
        self.entry = 0
    
    def add(self, path):
        if path[-1].head not in self.expanded:
            total = 0
            for i in path:
                total += i.cost
            h = self.map_graph.estimated_cost_to_goal((path[-1].head[0], path[-1].head[1]))
            heapq.heappush(self.container, (h + total, self.entry) + path)
            self.entry += 1

    def __iter__(self):
        """The object returns itself because it is implementing a __next__
        method and does not need any additional state for iteration."""
        return self

    def __next__(self):
        if len(self.container) > 0:
            removed = heapq.heappop(self.container)[2:]
            self.expanded.add(removed[-1].head)
            return removed
        else:
            raise StopIteration   # don't change this one


def print_list(lst):
    for item in lst:
        print(item)

def print_map(map_graph, frontier, solution):
    line_length = len(map_graph.map_str)
    reserved_points = ["X", "F", "G", "S"]
    if frontier.expanded is not None:
        for point in frontier.expanded:
            if map_graph.map_str[point[0]][point[1]] not in reserved_points:
                temp = list(map_graph.map_str[point[0]])
                temp[point[1]] = "."
                map_graph.map_str[point[0]] = "".join(temp)
            
    if solution is not None:
        for point in solution:
            if map_graph.map_str[point.head[0]][point.head[1]] not in reserved_points:
                temp = list(map_graph.map_str[point.head[0]])
                temp[point.head[1]] = "*"
                map_graph.map_str[point.head[0]] = "".join(temp)
    print_list(map_graph.map_str[:-1])


def main():
    # Test case 1 showing nodes checked by A* with a . and actual path with *
    map_str = """\
    +----------------+
    |                |
    |                |
    |                |
    |                |
    |                |
    |                |
    |        S       |
    |                |
    |                |
    |     G          |
    |                |
    |                |
    |                |
    +----------------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_map(map_graph, frontier, solution)
    
    
    # Test case 2 showing more complex behaviour
    map_str = """\
    +-------+
    |  F  X |
    |X XXXXG|
    | 3     |
    +-------+
    """
    
    map_graph = RoutingGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = next(generic_search(map_graph, frontier), None)
    print_actions(solution)    
    
    

if __name__ == "__main__":
    main()