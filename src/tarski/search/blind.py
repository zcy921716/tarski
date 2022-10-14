import logging
from collections import deque

from .model import GroundForwardSearchModel


class BreadthFirstSearch:
    """ Full expansion of a problem through Breadth-First search.
    Note that ATM we return no plan.
    """
    def __init__(self, model: GroundForwardSearchModel, max_expansions=-1):
        self.model = model
        self.max_expansions = max_expansions

    def run(self):
        return self.search(self.model.init())

    # def full_run(self):
    #     return self.full_search(self.model.init())
    #
    # def full_search(self, root):
    #     stats = SearchStats()
    #
    #     openlist = deque()  # fifo-queue storing the nodes which are next to explore
    #     openlist.append(make_root_node(root))
    #
    #     sol_length = None
    #
    #     sum_move_choices = 0
    #
    #     while openlist:
    #         stats.iterations += 1
    #         # logging.debug("brfs: Iteration {}, #unexplored={}".format(iteration, len(open_)))
    #
    #         node = openlist.popleft()
    #         if sol_length and len(node.extractPath()) > sol_length:
    #             break
    #
    #
    #         if self.model.is_goal(node.state):
    #             stats.num_goals += 1
    #             sol_length = len(node.extractPath())
    #             sum_move_choices += node.moveChoices(self.model)
    #             # logging.info(f"Goal found after {stats.nexpansions} expansions. {stats.num_goals} goal states found.")
    #
    #
    #
    #         if 0 <= self.max_expansions <= stats.nexpansions:
    #             # logging.info(f"Max. expansions reached. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
    #             return sum_move_choices, stats
    #
    #         for operator, successor_state in self.model.successors(node.state):
    #             openlist.append(make_child_node(node, operator, successor_state))
    #         stats.nexpansions += 1
    #
    #     # logging.info(f"Search space exhausted. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
    #     # space.complete = True
    #     return sum_move_choices, stats

    def search(self, root):
        # create obj to track state space
        # space = SearchSpace()
        stats = SearchStats()

        openlist = deque()  # fifo-queue storing the nodes which are next to explore
        openlist.append(make_root_node(root))
        closed = set()

        while openlist:
            stats.iterations += 1
            # logging.debug("brfs: Iteration {}, #unexplored={}".format(iteration, len(open_)))

            node = openlist.popleft()
            if self.model.is_goal(node.state):
                stats.num_goals += 1
                # logging.info(f"Goal found after {stats.nexpansions} expansions. {stats.num_goals} goal states found.")
                return node.extractPath(), stats

            if node.state in closed:
                continue

            closed.add(node.state)
            if 0 <= self.max_expansions <= stats.nexpansions:
                # logging.info(f"Max. expansions reached. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
                return None, stats

            for operator, successor_state in self.model.successors(node.state):
                openlist.append(make_child_node(node, operator, successor_state))
            stats.nexpansions += 1

        # logging.info(f"Search space exhausted. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
        # space.complete = True
        return None, stats

class DepthFirstSearch:
    """ Full expansion of a problem through Breadth-First search.
    Note that ATM we return no plan.
    """
    def __init__(self, model: GroundForwardSearchModel, max_expansions=-1):
        self.model = model
        self.max_expansions = max_expansions

    def run(self):
        return self.search(self.model.init())

    def search(self, root):
        # create obj to track state space
        # space = SearchSpace()
        stats = SearchStats()

        openlist = deque()  # fifo-queue storing the nodes which are next to explore
        openlist.append(make_root_node(root))
        closed = set()

        while openlist:
            stats.iterations += 1
            # logging.debug("brfs: Iteration {}, #unexplored={}".format(iteration, len(open_)))

            node = openlist.pop()
            if self.model.is_goal(node.state):
                stats.num_goals += 1
                # logging.info(f"Goal found after {stats.nexpansions} expansions. {stats.num_goals} goal states found.")
                return node.extractPath(), stats

            if node.state in closed:
                continue

            closed.add(node.state)
            if 0 <= self.max_expansions <= stats.nexpansions:
                # logging.info(f"Max. expansions reached. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
                return None, stats

            for operator, successor_state in self.model.successors(node.state):
                openlist.append(make_child_node(node, operator, successor_state))

            stats.nexpansions += 1

        # logging.info(f"Search space exhausted. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
        # space.complete = True
        return None, stats


class SearchNode:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

    def extractPath(self):
        if self.action:
            path = self.parent.extractPath() + [self.action]
        else:
            path = []

        return path

    def moveChoices(self, model):
        if self.action:
            count = 0
            for _, _ in model.successors(self.parent.state):
                count += 1
            return count + self.parent.moveChoices(model)
        else:
            return 0


class SearchSpace:
    """ A representation of a search space / transition system corresponding to some planning problem """
    def __init__(self):
        self.nodes = set()
        self.last_node_id = 0
        self.complete = False  # Whether the state space contains all states reachable from the initial state
    #
    # def expand(self, node: SearchNode):
    #     self.nodes.add(node)


class SearchStats:
    def __init__(self):
        self.iterations = 0
        self.num_goals = 0
        self.nexpansions = 0


def make_root_node(state):
    """ Construct the initial root node without parent nor action """
    return SearchNode(state, None, None)


def make_child_node(parent_node, action, state):
    """ Construct an child search node """
    return SearchNode(state, parent_node, action)
