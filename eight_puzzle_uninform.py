import numpy as np
from animation import draw
import argparse

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class EightPuzzle():
    
    def __init__(self, start_state, goal_state, algorithm, array_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.visited = [] # state
        self.algorithm = algorithm
        self.m, self.n = start_state.shape 
        self.array_index = array_index

    # goal test function
    def goal_test(self, current_state):
        return np.array_equal(current_state, self.goal_state)

    # get cost function
    def get_cost(self, current_state, next_state):
        return 1

    # get successor function
    def get_successors(self, state):
        successors = []

        #getting the position of where the empty spot is (0)
        positions = np.where(state == 0)
        #getting the specific row and column from the positions list to get the exact integers
        emptyrow, emptycol = positions[0][0], positions[1][0]

        def boundaries(coordinate, shift):
            rows, cols = state.shape
            if coordinate + shift not in range(rows):
                return True

        for moves in range(4):
            if moves % 2 == 0:
                shift = -1 # we go downwards or leftwards
            else:
                shift = 1 # we go upwards or rightwards

            if moves < 2:
                coordinate = emptyrow
            else:
                coordinate = emptycol

            if boundaries(coordinate, shift):
                continue

            temp = np.copy(state)
            if moves < 2:
                element = state[emptyrow + shift, emptycol]
                elementpos = np.where(state == element)
                temp[emptyrow, emptycol] = element
                temp[elementpos[0][0], elementpos[1][0]] = 0 # swapping the place of the empty spot and successor

            else:
                element = state[emptyrow, emptycol + shift]
                elementpos = np.where(state==element)
                temp[emptyrow, emptycol] = element
                temp[elementpos[0][0], elementpos[1][0]] = 0

            successors.append(temp)

        return successors
    
    # draw 
    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
        draw(path[::-1], self.array_index, self.algorithm)

    # solve it
    def solve(self):
        fringe = [] # node
        state = self.start_state.copy() # use copy() to copy value instead of reference 
        node = Node(state, 0, None)
        self.visited.append(state)

        # the maximum depth cut off for dfs
        max_depth = 15
        depth = 0

        if self.goal_test(state):
            return state

        if self.algorithm == 'Depth-Limited-DFS': 
            fringe.append(node)
            depth += 1

        elif self.algorithm == 'BFS':
            # push it into a queue
            fringe.insert(0, node)

        while fringe:
            if self.algorithm == 'BFS':
                current = fringe.pop()
            elif self.algorithm == 'Depth-Limited-DFS':
                current = fringe.pop(0)
            successors = self.get_successors(current.state)
            for next_state in successors:
                for visited_state in self.visited:
                    if np.array_equal(visited_state, next_state) is False:
                        # create a node for next_state
                        next_cost = node.cost_from_start + self.get_cost(node.state, next_state)
                        next_node = Node(next_state, next_cost, current)

                        if self.goal_test(next_state) is True:
                            self.draw(next_node)
                            return next_state
                        if self.algorithm == 'BFS':
                            fringe.insert(0, next_node)
                        elif self.algorithm == 'Depth-Limited-DFS':
                            fringe.append(next_node)
                            depth += 1
                            if depth == max_depth:
                                continue

if __name__ == "__main__":
    
    goal = np.array([[1,2,3],[4,5,6],[7,8,0]])
    start_arrays = [np.array([[0,1,3],[4,2,5],[7,8,6]]), # easy one. use this in lab
                    np.array([[0,2,3],[1,4,6],[7,5,8]])] # medium one.

    algorithms = ['Depth-Limited-DFS', 'BFS']
    
    parser = argparse.ArgumentParser(description='eight puzzle')

    parser.add_argument('-array', dest='array_index', required = True, type = int, help='index of array')
    parser.add_argument('-algorithm', dest='algorithm_index', required = True, type = int, help='index of algorithm')

    args = parser.parse_args()

    # run this in the terminal using array 0, algorithm BFS
    # python eight_puzzle_uninform.py -array 0 -algorithm 1
    game = EightPuzzle(start_arrays[args.array_index], goal, algorithms[args.algorithm_index], args.array_index)
    game.solve()