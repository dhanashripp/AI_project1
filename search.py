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
import searchAgents

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

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
     "*** YOUR CODE HERE ***"
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())


    """

    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        from game import Directions
        dfs_stack = util.Stack()
        v1 = problem.getStartState()
        visited = []
        dfs_stack.push(v1)
        previous_N = {}
        previous_A = {}
        LIST=[]
        while (not dfs_stack.isEmpty()):
            vertex_popped = dfs_stack.pop()
            if(problem.isGoalState(vertex_popped)):
                Path = []
                tempPath=[]
                g = vertex_popped
                while g in previous_N:
                    temp = previous_A[g]
                    tempPath.append(temp)
                    g = previous_N[g]
                for j in reversed(tempPath):
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
    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        from game import Directions
        bfs_queue = util.Queue()
        v1 = problem.getStartState()
        visited = []
        bfs_queue.push(v1)
        previous_N = {}
        previous_A = {}
        LIST=[]
        visited.append(v1)
        while (not bfs_queue.isEmpty()):
            vertex_popped = bfs_queue.pop()
            if(problem.isGoalState(vertex_popped)):
                Path = []
                tempPath=[]
                g = vertex_popped
                while g in previous_N:
                    temp = previous_A[g]
                    tempPath.append(temp)
                    g = previous_N[g]
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
    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        v1 = problem.getStartState()
        ucs_queue = util.PriorityQueue()
        visited = []
        cost = 0
        tupl = (v1, [], 0)

        ucs_queue.push(tupl, cost)
        while (not ucs_queue.isEmpty()):
            node = ucs_queue.pop()
            vertex_popped = node[0]

            if(problem.isGoalState(vertex_popped)):
                return node[1]

            if((vertex_popped not in visited)):
                visited.append(vertex_popped)
                s_cost = node[2]
                for (N,A,C) in problem.getSuccessors(vertex_popped):
                    cost = s_cost
                    path = node[1]
                    path = path + [A]
                    cost += C
                    ucs_queue.push((N, path, cost), cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    
    if(problem.isGoalState(problem.getStartState())):
        return []
    else:
        v1 = problem.getStartState()
        ucs_queue = util.PriorityQueue()
        visited = []
        cost = 0
        h_value = heuristic(v1, problem)
        tupl = (v1, [], 0)

        ucs_queue.push(tupl, cost + h_value)
        while (not ucs_queue.isEmpty()):
            node = ucs_queue.pop()
            vertex_popped = node[0]

            if(problem.isGoalState(vertex_popped)):
                return node[1]

            if((vertex_popped not in visited)):
                visited.append(vertex_popped)
                s_cost = node[2]
                for (N,A,C) in problem.getSuccessors(vertex_popped):
                    cost = s_cost
                    path = node[1]
                    path = path + [A]
                    cost += C
                    new_hvalue = heuristic(N, problem)
                    ucs_queue.push((N, path, cost), new_hvalue + cost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
