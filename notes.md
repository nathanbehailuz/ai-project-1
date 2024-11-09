Arguments inputted to the program are start and goal positions of a point robot, and a 2D integer array that represents the robot workspace.

Create 
1. A StarSearch(problem, f) returns a solution node or a failure
2. Frontier
    - should be a priority queue use heapq
    - Stores (f(node), node) where f(node) is the priority.
3. Problem class
    properties: 
        - INITIAL
            start state with x and y 
        - GOAL
            the goal state
        - grid
            2d array representing the workspace
    method:
        - IS-GOAL(node.STATE) returns node else -1?
        - ACTIONS(state)
            Returns all possible actions from the current state (`up`, `down`, `left`, `right`, `up45`, `down45`, `up135`, `down135`)
        - ACTION-COST(state, action, s')
            Returns the cost of taking the action from `state` to `s'`. If `s'` is an illegal state (i.e., black tile, `1` in the grid), it returns an infinite cost.
        - RESULT (state, action)
            Returns the next state after applying the action.
4. Node class
    properties:
        - STATE: 
            a dictionary with keys x and y (its row and col positions in the workspace)
        - PATH-COST:
            The cost accumulated from the start state to the current node.
        - ACTIONs:
            up, down, left , right, up45, down45, up135, down135
        - PARENT
            The parent node from which this node was generated.
5. Reached
    a lookup table, with one entry with key problem.INITIAL and value node
6. IS-EMPTY
    function checking if the frontier is empty
7. EXPAND(problem,node) return node
    - For each valid action in problem.ACTIONS(node.STATE), generates a new child node and returns it
    - check if the action is valid
        - doesnot place robot in obstacle
        - in bound in workspace
8. f(node) returns a number which is g(n) + h(n)
    - Returns a number g(n) + h(n) where:
        - g(n): The accumulated cost of the path to the node (step-cost + path-cost).
        - h(n): The heuristic (Euclidean distance from the current node to the goal).
    
    - g(n) (Step-cost + Path-cost):
        - Step-cost: 
            - Horizontal/vertical moves (up, down, left, right) cost `1`.
            - Diagonal moves (up45, down45, up135, down135) cost `√2`.
        - Angle-cost: 
            - `angle-cost = k * (deltaTheta) / 180` where `k` is a user-defined constant (`2` or `4`).
            - `deltaTheta = abs(theta(s) - theta(s'))`, where `theta` is calculated using .
            - For the initial node, angle-cost is `0`.

    - h(n) (Heuristic):
        - Use Euclidean distance from the current node to the goal node.
    
    h(n): euclidan distance from n to goal node

9. workspace
    - a 2d array with some numbers in it 
        if val is 
                0 - white(legal) state
                1 - black(illegal) state
                2 - start position
                4 - solution path
                5 - goal position 

10. Theta Calculation:
    - For each move, calculate the angle `theta` using `atan2(y, x)` to determine the direction.
    - Ensure that special cases are handled, especially when moving diagonally.

11. Priority Queue with `heapq`:
    - Use Python's `heapq` to manage the frontier as a priority queue:


12. User Input for `k`:
    - Allow the user to specify `k` (angle-cost multiplier). Validate the input to ensure it's either `2` or `4`.



File reading: 


______
function BEST-FIRST-SEARCH(problem, f) returns a solution node or failure
    node ← NODE(STATE=problem.INITIAL)
    frontier ← a priority queue ordered by f, with node as an element
    reached ← a lookup table, with one entry with key problem.INITIAL and value node
    while not IS-EMPTY(frontier) do
        node ← POP(frontier)
        if problem.IS-GOAL(node.STATE) then return node
        for each child in EXPAND(problem, node) do
            s ← child.STATE
            if s is not in reached or child.PATH-COST < reached[s].PATH-COST then
                reached[s] ← child
                add child to frontier
    return failure

function EXPAND(problem, node) yields nodes
    s ← node.STATE
    for each action in problem.ACTIONS(s) do
        s' ← problem.RESULT(s, action)
        cost ← node.PATH-COST + problem.ACTION-COST(s, action, s')
        yield NODE(STATE=s', PARENT=node, ACTION=action, PATH-COST=cost)


the illegal states 
every box is a state
reading the input from the text and representing it as an array bear some dofferences:
    - if you follow the files natural sequence, 0,0 is gonna be on top-left but the assignment asks us to make it on bottom left 
        solution: 
            array[::-1, :]

roots f shouldnot be 0; since it have a non zero h(n)
_____

dont count repeated states 