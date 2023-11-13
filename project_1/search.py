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

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # print("Start:", problem.getStartState())
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    # print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    color = {}
    pd = {}

    def DFS(state):
        color[state] = 1
        if(problem.isGoalState(state)):
            return state
        for nextmove in problem.getSuccessors(state):
            pos = nextmove[0]
            direct = nextmove[1]
            cost = nextmove[2]
            if pos not in color:
                pd[pos] = (state,direct)
                result = DFS(pos)
                if result is not None:
                    return result
                
    goal = DFS(problem.getStartState())
    # print("begin:", problem.getStartState())
    # print("It's goal", goal)
    # print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    # print("direction to goal:", pd[goal][0], pd[goal][1])
   
    trace = []
    while(goal != problem.getStartState()):
        dir_move = pd[goal][1]
        trace.append(dir_move)
        goal = pd[goal][0]
    
    res = trace[::-1]
    print("res:", res)
    return res

    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    queue = Queue()
    visited = []
    pd = {}

    beginState = problem.getStartState()
    queue.push(beginState)
    goal = problem.getStartState()
    visited.append(beginState)

    while not queue.isEmpty():
        state = queue.pop()
        # print(state)
        if problem.isGoalState(state):
            goal = state
            break

        for (nextState, direct, dis) in problem.getSuccessors(state):
            if nextState not in visited:
                queue.push(nextState)
                visited.append(nextState)
                pd[nextState] = (state, direct)

    trace = []
    while(goal != problem.getStartState()):
        dir_move = pd[goal][1]
        trace.append(dir_move)
        goal = pd[goal][0]
    
    res = trace[::-1]
    # print("res:", res)
    return res
    

    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    from util import PriorityQueue

    queue = PriorityQueue()
    visit = []
    trace = []

    queue.push((problem.getStartState(), trace), 0)

    while not queue.isEmpty():
        curr = queue.pop()
        state = curr[0]
        trace = curr[1]

        if(problem.isGoalState(state)):
            return trace
        
        if state not in visit:
            visit.append(state)
            for (nextState, direct, dis) in problem.getSuccessors(state):
                    path = trace + [direct]
                    queue.push((nextState, path), problem.getCostOfActions(path))
    return []
    #print("problem next successor:", problem.getSuccessors(problem.getStartState()))
    # from util import PriorityQueue
    # queue = PriorityQueue()
    # visited = []
    # pd = {}

    # beginState = problem.getStartState()
    # queue.push((beginState,0),0)
    # goal = problem.getStartState()
    # visited.append(beginState)

    # while not queue.isEmpty():
    #     curr = queue.pop()
    #     # print("curr:",curr)
    #     state = curr[0]
    #     cost = curr[1]

    #     # print(state)
    #     if problem.isGoalState(state):
    #         goal = state
    #         break

    #     for (nextState, direct, dis) in problem.getSuccessors(state):
    #         if nextState not in visited:
    #             queue.push((nextState,cost + dis), cost + dis)
    #             visited.append(nextState)
    #             pd[nextState] = (state, direct)

    # trace = []
    # while(goal != problem.getStartState()):
    #     dir_move = pd[goal][1]
    #     trace.append(dir_move)
    #     goal = pd[goal][0]
    
    # res = trace[::-1]

    # return res

    
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    queue = PriorityQueue()
    visit = []
    trace = []

    queue.push((problem.getStartState(), trace), 0)

    while not queue.isEmpty():
        curr = queue.pop()
        state = curr[0]
        trace = curr[1]

        if(problem.isGoalState(state)):
            return trace
        
        if state not in visit:
            visit.append(state)
            for (nextState, direct, dis) in problem.getSuccessors(state):
                    path = trace + [direct]
                    f_v_nextState = problem.getCostOfActions(path) + heuristic(nextState, problem)
                    queue.push((nextState, path), f_v_nextState)
    return []

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
