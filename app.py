'''
    app.py
    This file contains the implementation of the A* search algorithm for the robot path planning problem.
    Author: Nathan Behailu
    NetID: nz2212
    Date: 11/09/2024
'''

import heapq
import math
import sys
import copy
from vis import plot_maze

# global variables
K = 0
totalNodes = 1

# mapping the moves to numbers
movesToNums = {
            "up": 2, "down": 6, "left": 4, "right": 0,
            "up45": 1, "down45": 7, "up135": 3, "down135": 5
        }

# Node class
class Node:
    def __init__(self, state=0, parent=None, action=None, path_cost=0, orientation=0):
        # initializing the node
        self.STATE = state 
        self.PARENT = parent
        self.ACTION = action
        self.PATH_COST = path_cost
        self.TOTAL_COST = 0
        self.ORIENTATION = orientation 

    # less than operator for the priority queue
    def __lt__(self, other):
        return self.TOTAL_COST < other.TOTAL_COST

# Problem class
class Problem:
    def __init__(self, start, goal, grid):
        # initializing the problem
        self.INITIAL = start  
        self.GOAL = goal 
        self.grid = grid

    # checking if the state is the goal state
    def IS_GOAL(self, state):
        return state == self.GOAL

    # returning all the actions
    def ACTIONS(self):
        return ["up", "down", "left", "right", "up45", "down45", "up135", "down135"]

    # returning the direction of the action
    def direction(self, action):
        if action == "up": return 90
        if action == "down": return 270
        if action == "left": return 180
        if action == "right": return 0
        if action == "up45": return 45
        if action == "down45": return 315
        if action == "up135": return 135
        if action == "down135": return 225

    # calculating the cost of the action
    def ACTION_COST(self, action, node, s_prime):
        global K

        # Calculate the angle difference (deltaTheta) based on the previous orientation
        deltaTheta = abs(self.direction(action) - node.ORIENTATION)

        # if the angle difference is greater than 180, then we need to subtract it from 360
        if deltaTheta > 180:
            deltaTheta = 360 - deltaTheta
        
        # if the node is the root node, then the angle cost is 0
        if node.PARENT != None and node.PARENT.PARENT == None:
            angle_cost = 0
        else:
            angle_cost = K * deltaTheta / 180

        # if the action is one of the cardinal directions, then the distance cost is 1
        if action in ["up", "down", "left", "right"]:
            distance_cost = 1  
        else:
            distance_cost = math.sqrt(2)  

        # the total cost is the sum of the distance cost and the angle cost
        total_cost = distance_cost + angle_cost
        return total_cost

    # returning the result of the action 
    def RESULT(self, state, action):
        # mapping the moves to the changes in the x and y coordinates 
        moves = {
            "up": (0, 1), "down": (0, -1), "left": (-1, 0), "right": (1, 0),
            "up45": (1, 1), "down45": (1, -1), "up135": (-1, 1), "down135": (-1, -1)
        }

        # updating the state and the orientation 
        dx, dy = moves[action]
        updated_state = {'x': state['x'] + dx, 'y': state['y'] + dy}
        updated_orientation = self.direction(action)  
        return updated_state, updated_orientation

# calculating the euclidean distance between the current state and the goal state 
def euclidean_distance(state, goal):
    return math.sqrt((goal['x'] - state['x'])**2 + (goal['y'] - state['y'])**2)

# A* search algorithm
def AStarSearch(problem, f):
    global totalNodes

    # initializing the node
    node = Node(problem.INITIAL)
    node.TOTAL_COST = euclidean_distance(node.STATE, problem.GOAL)

    # initializing the frontier
    frontier = []
    heapq.heapify(frontier)
    heapq.heappush(frontier, (f(node, problem), node))

    # initializing the reached set
    reached = {tuple(problem.INITIAL.values()): node}

    # while the frontier is not empty
    while frontier:
        _, node = heapq.heappop(frontier)

        # if the node is the goal state, then return the node
        if problem.IS_GOAL(node.STATE):
            return node

        # expanding the node
        for child in EXPAND(problem, node):
            s = tuple(child.STATE.values())
            child_f = f(child, problem)
            child.TOTAL_COST = child_f

            if s not in reached:
                # TODO: check if this is correct place to increment totalNodes
                totalNodes += 1

            # if the state is not in the reached table or the child's total cost is less than the reached set's total cost, then add the child to the reached table and push it to the frontier
            if s not in reached or child.TOTAL_COST < reached[s].TOTAL_COST:
                reached[s] = child
                heapq.heappush(frontier, (child_f, child))

    # if the goal state is not found, then return None
    return None  

# expanding the node
def EXPAND(problem, node): 
    for action in problem.ACTIONS():
        s_prime, updated_orientation = problem.RESULT(node.STATE, action)
        # if the state is in range and the cell is not an obstacle, then yield the child node   
        if inRange(s_prime, problem) and problem.grid[s_prime['y']][s_prime['x']] != 1:
            cost = node.PATH_COST + problem.ACTION_COST(action, node, s_prime)
            yield Node(state=s_prime, parent=node, action=action, path_cost=cost, orientation=updated_orientation)

# calculating the total cost of the node
def f(node, problem):
    g = node.PATH_COST  
    h = euclidean_distance(node.STATE, problem.GOAL)
    return g + h

# checking if the state is in range
def inRange(s_prime, problem):
    return 0 <= s_prime['x'] < len(problem.grid[0]) and 0 <= s_prime['y'] < len(problem.grid)

# getting the tree information
def getTreeInfo(path, actions, total_costs):
    depth = len(path) - 1
    result = ''
    result += str(depth) + '\n' + str(totalNodes) + '\n'
    result += ' '.join(map(str, actions)) + '\n'
    result += ' '.join(map(str, total_costs)) + '\n'  

    return result

# main function
def main():
    # checking if the file path and the value for k are provided
    if len(sys.argv) < 3:
        print("Error, please enter: python app.py <file_path> <value for k>")
        return

    # getting the file path and the value for k
    file_path = sys.argv[1]
    global K 
    K = int(sys.argv[2])

    # reading the file
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # getting the start and goal states
    first_line = lines[0].strip().split()
    start = {'x': int(first_line[0]), 'y': int(first_line[1])}
    goal = {'x': int(first_line[2]), 'y': int(first_line[3])}

    # getting the grid
    grid = []
    for line in lines[1:]:
        grid.append(list(map(int, line.strip().split())))

    # reversing the grid to make the bottom-left the origin
    grid = grid[::-1] 

    problem = Problem(start, goal, grid)
    
    # finding the solution node
    solution_node = AStarSearch(problem, f)

    # if the solution node is found, then print the path
    if solution_node:
        print("Solution found!")

        # getting the path, actions, and total costs
        path = []
        actions = []
        total_costs = []
        node = solution_node
        while node:
            path.append(node.STATE)
            if node.ACTION != None:
                actions.append(movesToNums[node.ACTION])
            total_costs.append(round(node.TOTAL_COST, 2))
            node = node.PARENT

        # reversing the path, actions, and total costs
        path.reverse()
        actions.reverse()
        total_costs.reverse()

        # creating the final grid
        final_grid = copy.deepcopy(grid)

        for p in path:
            if not final_grid[p['y']][p['x']] in [2, 5]:
                final_grid[p['y']][p['x']] = 4 

        final_grid = final_grid[::-1]
        
        # writing the output to the file
        # Remove .txt extension if present
        file_path = file_path.rstrip('.txt')
        output_file_path = f"output/{file_path.split('/')[-1]}_output_{K}.txt"
        with open(output_file_path, 'w') as output_file:
            output_file.write(getTreeInfo(path, actions, total_costs))
            for row in final_grid:
                output_file.write(" ".join(map(str, row)) + "\n")
            
            print("Output successfully written to", output_file_path)

        # asking the user if they want to visualize the robot's path
        choice = input("Wanna visualize the robot's path? (y/n) ")
        while True:
            if choice.lower() == 'y':
                plot_maze(output_file_path)
                break
            elif choice.lower() == 'n':
                print("Goodbye!")
                break
            else:
                print("Please enter the correct input: y (yes) or n (no)")
                choice = input("Wanna visualize the robot's path? (y/n) ")

    else:
        print("No solution found.")

if __name__ == "__main__":
    main()
