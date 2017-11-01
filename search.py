# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    This algorithm returns a list of actions that reaches the goal by traversing
    as deep as possible from neighbour to neighbour before backtracking. While
    doing this the algorithm checks whether the node it would expand was
    previously visited or not.
    The pacman is assumed to travel in graph structure and does so by taking
    steps in one of the following directions: North, South, East and West.
    The data structure used for implementation of depth first search algorith
    is STACK, since it works on LIFO order and helps us to traverse till deepest
    node.
    """

    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        from game import Directions
        dfs_stack = util.Stack() #initializing stack
        v1 = problem.getStartState()
        visited = [] #list to collect nodes those are visisted
        dfs_stack.push(v1)
        previous_N = {} #list to collect parent nodes of the current node
        previous_A = {} #list to collect actions leading to next nodes
        LIST=[]
        while (not dfs_stack.isEmpty()):
            vertex_popped = dfs_stack.pop()
            if(problem.isGoalState(vertex_popped)):
                Path = []
                tempPath=[]
                g = vertex_popped
                while g in previous_N: #traversing from goal to start startState
                    temp = previous_A[g]
                    tempPath.append(temp)
                    g = previous_N[g]
                for j in reversed(tempPath): # reversing the path since we want path from start node till goal state
                    Path.append(j)
                return Path
            if((vertex_popped not in visited)):
                visited.append(vertex_popped)
            LIST = problem.getSuccessors(vertex_popped)

            for (N,A,C) in LIST:
                if ((A == Directions.SOUTH)):
                    if((N not in visited)):
                        previous_N[N] = vertex_popped
                        previous_A[N] = A
                        dfs_stack.push(N)
                    visited.append(N)
                    break

            for (N,A,C) in LIST:
                if((N not in visited)):
                    previous_N[N] = vertex_popped
                    previous_A[N] = A
                    dfs_stack.push(N)

def breadthFirstSearch(problem):
    """
    Search all the neighbours of a node before visiting neighbours' neighbours.

    This algorithm returns a list of actions that reaches the goal by traversing
    all the nodes at one level before expanding nodes of next level. While
    doing this the algorithm checks whether the node it would expand was
    previously visited or not.
    The pacman is assumed to travel in graph structure and does so by taking
    steps in one of the following directions: North, South, East and West.
    The data structure used for implementation of depth first search algorith
    is QUEUE, since it works on FIFO order and helps us traverse all the
    neighbours.
    """
    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        from game import Directions
        bfs_queue = util.Queue() #initializing queue
        v1 = problem.getStartState()
        visited = []    #list to collect nodes those are visisted
        bfs_queue.push(v1)
        previous_N = {} #list to collect parent nodes of the current node
        previous_A = {} #list to collect actions leading to next nodes
        LIST=[]
        visited.append(v1)
        while (not bfs_queue.isEmpty()):
            vertex_popped = bfs_queue.pop()

            if(problem.isGoalState(vertex_popped)):
                Path = []
                tempPath=[]
                g = vertex_popped
                while g in previous_N:  #traversing from goal to start startState
                    temp = previous_A[g]
                    tempPath.append(temp)
                    g = previous_N[g]
                # reversing the path since we want path from start node till goal state
                for j in reversed(tempPath):
                    Path.append(j)
                return Path
            LIST = problem.getSuccessors(vertex_popped)
            for (N,A,C) in LIST:
                if((N not in visited)):
                    visited.append(N)
                    previous_N[N] = vertex_popped
                    previous_A[N] = A
                    bfs_queue.push(N)

def uniformCostSearch(problem):
    """
    This algorithm is generalization of breadth first search.
    All nodes have cost associated with them. Based upon this cost we find the
    next node to be expanded. The node with least cost is expanded first.
    While doing this the algorithm checks whether the node it would expand was
    previously visited or not.
    The pacman is assumed to travel in graph structure and does so by taking
    steps in one of the following directions: North, South, East and West.
    The data structure used for implementation of Uniform Cost search algorithm
    is Priority QUEUE, where we push tuple of node and cost.
    """
    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        v1 = problem.getStartState()
        ucs_queue = util.PriorityQueue() #initializing priority queue
        visited = [] #list to collect nodes those are visisted
        cost = 0
        tupl = (v1, [], 0)

        ucs_queue.push(tupl, cost) #pushing node (N,A,C) and cost=f(n)
        while (not ucs_queue.isEmpty()):
            node = ucs_queue.pop()
            vertex_popped = node[0]

            if(problem.isGoalState(vertex_popped)):
                return node[1]

            if((vertex_popped not in visited)):
                visited.append(vertex_popped) #expanding the successors only when the node popped is already not expanded.

                for (N,A,C) in problem.getSuccessors(vertex_popped):
                    cost =  node[2]
                    path = node[1]
                    path = path + [A] #update full path by appending action of node  that is being pushed
                    cost += C #update total cost by adding cost of node  that is being pushed
                    ucs_queue.push((N, path, cost), cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    The issues(expansion of unnecessary nodes) of uniform cost search are solved
    using A* search which adds a bias also known as heuristic to cost function
    of each node. By default the heuristic used is nullHeuristic which makes A*
    behave as UCS.
    When A* is called using manhattanHeuristic it behaves better than UCS as it
    expands only in direction of goal.
    While doing this the algorithm checks whether the node it would expand was
    previously visited or not.
    The pacman is assumed to travel in graph structure and does so by taking
    steps in one of the following directions: North, South, East and West.
    The data structure used for implementation of A* search algorithm
    is Priority QUEUE, where we push tuple of node and cost.
    """
    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        v1 = problem.getStartState()
        as_queue = util.PriorityQueue() #initializing priority queue
        visited = [] #list to collect nodes those are visisted
        cost = 0
        h_value = heuristic(v1, problem) # with manhattanHeuristic as input to aStarSearch function h_value is the manhattanDistance
        tupl = (v1, [], 0)

        as_queue.push(tupl, cost + h_value) #pushing node (N,A,C) and cost+heuristic= f(n)
        while (not as_queue.isEmpty()):
            node = as_queue.pop()
            vertex_popped = node[0]

            if(problem.isGoalState(vertex_popped)):
                return node[1]

            if((vertex_popped not in visited)):#expanding the successors only when the node popped is already not expanded.
                visited.append(vertex_popped)

                for (N,A,C) in problem.getSuccessors(vertex_popped):
                    cost = node[2]
                    path = node[1]
                    path = path + [A] #update full path by appending action of node  that is being pushed
                    cost += C         #update total cost by adding cost of node  that is being pushed
                    new_hvalue = heuristic(N, problem)
                    as_queue.push((N, path, cost), new_hvalue + cost)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
