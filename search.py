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
    return [s, s, w, s, w, w, s, w]


def trace_path(xy, nodes_dict):
    """Helper for search methods.
    trace_path(xy, node_dict) traces the path backward from the goal node
    to the start node, then reverses the path so that the python list that
    is returned is ordered from start to goal.
    node is of the form: ['action', parent_xy]"""

    actions_path = []
    node = nodes_dict[xy]  # goal node

    # trace backward until I hit the start node whose parent is None @ node[1]
    while node[1] is not None:
        action = node[0]
        actions_path.append(action)
        parent_xy = node[1]
        node = nodes_dict.get(parent_xy)

    actions_path.reverse()  # returns None

    return actions_path


def search_helper(problem, fringe, priority_queue=False, a_star=False, custom_heuristic=None):
    """Helper for all search methods."""

    closed = set()
    xy = problem.getStartState()  # is of form (x, y)

    if priority_queue:
        fringe.push((xy, [], 0), 0)

    else:
        fringe.push((xy, [], 0))

    while not fringe.isEmpty():
        xy, actions_path, cost = fringe.pop()

        if problem.isGoalState(xy):
            return actions_path

        # Add the node to the closed set if it hasn't been expanded previously.
        if xy not in closed:
            closed.add(xy)

            # Expand the node
            children_info = problem.getSuccessors(xy)  # list of [(x,y), 'action', cost]

            for child_xy, child_action, child_cost in children_info:

                if child_xy not in closed:

                    if a_star:
                        back_cost = cost + child_cost
                        forward_cost = custom_heuristic(child_xy, problem)
                        priority = back_cost + forward_cost
                        fringe.push((child_xy, actions_path + [child_action], back_cost), priority)

                    elif priority_queue:
                        back_cost = cost + child_cost
                        fringe.push((child_xy, actions_path + [child_action], back_cost), back_cost)

                    else:
                        fringe.push((child_xy, actions_path + [child_action], child_cost))

    # Goal was not found
    return []


def depthFirstSearch(problem):
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

    fringe = util.Stack()
    return search_helper(problem, fringe)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    fringe = util.Queue()
    return search_helper(problem, fringe)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    fringe = util.PriorityQueue()
    return search_helper(problem, fringe, priority_queue=True)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    fringe = util.PriorityQueue()
    return search_helper(problem, fringe, priority_queue=True, a_star=True, custom_heuristic=heuristic)



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


