import random
import math
import time
import psutil
import os
import sys


# This class defines the state of the problem in terms of board configuration
class Board:
    def __init__(self, tiles):
        self.size = int(math.sqrt(len(tiles)))  # defining length/width of the board
        self.tiles = tiles

    # This function returns the resulting state from taking particular action from current state
    def execute_action(self, action):
        new_tiles = self.tiles[:]
        empty_index = new_tiles.index('0')
        if action == 'L':
            if empty_index % self.size > 0:
                new_tiles[empty_index - 1], new_tiles[empty_index] = new_tiles[empty_index], new_tiles[empty_index - 1]
        if action == 'R':
            if empty_index % self.size < (self.size - 1):
                new_tiles[empty_index + 1], new_tiles[empty_index] = new_tiles[empty_index], new_tiles[empty_index + 1]
        if action == 'U':
            if empty_index - self.size >= 0:
                new_tiles[empty_index - self.size], new_tiles[empty_index] = new_tiles[empty_index], new_tiles[
                    empty_index - self.size]
        if action == 'D':
            if empty_index + self.size < self.size * self.size:
                new_tiles[empty_index + self.size], new_tiles[empty_index] = new_tiles[empty_index], new_tiles[
                    empty_index + self.size]
        return Board(new_tiles)


# This class defines the node on the search tree, consisting of state, parent and previous action
class Node:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

    # Returns string representation of the state
    def __repr__(self):
        return str(self.state.tiles)

    # Comparing current node with other node. They are equal if states are equal
    def __eq__(self, other):
        return self.state.tiles == other.state.tiles

    def __hash__(self):
        return hash(tuple(self.state.tiles))


class Search:

    def __init__(self):
        self.expanded_nodes = 0
        self.max_memory = 0

    # Utility function to randomly generate 15-puzzle
    def generate_puzzle(self, size):
        numbers = list(range(size * size))
        random.shuffle(numbers)
        return Node(Board(numbers), None, None)

    # This function returns the list of children obtained after simulating the actions on current node
    def get_children(self, parent_node):
        children = []
        actions = ['L', 'R', 'U', 'D']  # left,right, up , down ; actions define direction of movement of empty tile
        for action in actions:
            child_state = parent_node.state.execute_action(action)
            child_node = Node(child_state, parent_node, action)
            children.append(child_node)
        return children

    # This function backtracks from current node to reach initial configuration. The list of actions would constitute a solution path
    def find_path(self, node):
        path = []
        while (node.parent is not None):
            path.append(node.action)
            node = node.parent
        path.reverse()
        return path

    # Heuristic function for misplaced number of tiles
    def h1_m_tiles(self, Node):
        curr_tiles = Node.state.tiles
        solution = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']

        misplaced_tiles = 0

        for i in range(0, len(curr_tiles)):
            if curr_tiles[i] != solution[i]:
                misplaced_tiles += 1

        return misplaced_tiles

    # Heuristic function for Manhattan distance
    def h2_manhattan_d(self, Node):
        curr_tiles = Node.state.tiles
        solution = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']

        manhattan_distance = 0

        for i in range(0, len(curr_tiles)):
            i_solution = solution.index(curr_tiles[i])

            manhattan_distance += abs((i // 4) - (i_solution // 4)) + abs((i % 4) - (i_solution % 4))

        return manhattan_distance

    # method to perform iterative deepening A*. It uses another method to perform searching, and it has
    # two different heuristic functions (number of misplaced tiles and Manhattan distance)
    def ida_star(self, root_node):
        self.expanded_nodes = 0
        start_time = time.time()

        bound = self.h1_m_tiles(root_node)
        #bound = self.h2_manhattan_d(root_node)

        path = [root_node]

        while True:
            t, solution_node = self.search(path, 0, bound)

            if t == "FOUND":
                memory_usage = sys.getsizeof(path) + sys.getsizeof(solution_node) + sys.getsizeof(self)
                self.max_memory = max(self.max_memory, memory_usage)    # get the maximum memory

                return self.find_path(solution_node), self.expanded_nodes, (time.time() - start_time), self.max_memory

            if t == float('inf'):
                return "NOT_FOUND"

            bound = t   # update new bound

    # method to perform actual searching. It takes the current path, the g value and a bound used to check other nodes.
    # It works recursively to find a solution.
    def search(self, path, g, bound):
        self.expanded_nodes += 1

        node = path[-1]
        f = g + self.h1_m_tiles(node)
        #f = g + self.h2_manhattan_d(node)

        if f > bound:
            return f, None

        if self.goal_test(node.state.tiles):    # check for goal
            return "FOUND", node

        min_val = float('inf')

        for child in self.get_children(node):
            if child not in path:
                path.append(child)

                t, solution_node = self.search(path, g + 1, bound)  # recursive call to check for children

                if t == "FOUND":
                    return "FOUND", solution_node

                if t < min_val:     # update new minimum value
                    min_val = t

                path.pop(-1)

        return min_val, None

    # Function to test the goal
    def goal_test(self, cur_tiles):
        return cur_tiles == ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']

    def solve(self, input):

        initial_list = input.split(" ")
        root = Node(Board(initial_list), None, None)
        path, expanded_nodes, time_taken, memory_consumed = self.ida_star(root)
        print("Moves: " + " ".join(path))
        print("Number of expanded Nodes: " + str(expanded_nodes))
        print("Time Taken: " + str(time_taken))
        print("Max Memory (Bytes): " + str(memory_consumed))
        return "".join(path)


if __name__ == '__main__':
    agent = Search()
    agent.solve("1 3 4 8 5 2 0 6 9 10 7 11 13 14 15 12")

"""
Test cases
1 0 3 4 5 2 6 8 9 10 7 11 13 14 15 12 | D R D R D

1 2 3 4 5 6 8 0 9 11 7 12 13 10 14 15 | L D L D R R

1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15 | R D L D D R R

1 2 0 4 6 7 3 8 5 9 10 12 13 14 11 15 | D L L D R R D R

1 3 4 8 5 2 0 6 9 10 7 11 13 14 15 12 | R U L L D R D R D
"""
