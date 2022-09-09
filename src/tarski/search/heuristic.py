import logging, math
from queue import PriorityQueue

from .model import GroundForwardSearchModel

def zero_heuristic(state):
    return 0


class AStarSearch:
    """ Full expansion of a problem through Breadth-First search.
    Note that ATM we return no plan.
    """
    def __init__(self, model: GroundForwardSearchModel, max_expansions=-1, heuristic=zero_heuristic):
        self.model = model
        self.max_expansions = max_expansions
        self.heuristic = heuristic

    def run(self):
        return self.search(self.model.init())

    def search(self, root):
        # create obj to track state space
        # space = SearchSpace()
        stats = SearchStats()

        node_count = 1
        openlist = PriorityQueue()  # fifo-queue storing the nodes which are next to explore
        openlist.put((self.heuristic(root), node_count, make_root_node(root)))
        closed = dict(root = 0)

        while not openlist.empty():
            stats.iterations += 1
            # logging.debug("brfs: Iteration {}, #unexplored={}".format(iteration, len(open_)))

            _, _, node  = openlist.get()
            if self.model.is_goal(node.state):
                stats.num_goals += 1
                # logging.info(f"Goal found after {stats.nexpansions} expansions. {stats.num_goals} goal states found.")
                return node.extractPath(), stats

            if 0 <= self.max_expansions <= stats.nexpansions:
                # logging.info(f"Max. expansions reached. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
                return None, stats

            for operator, successor_state in self.model.successors(node.state):
                if successor_state not in closed or closed[successor_state] > node.accumulated_cost + 1:
                    node_count += 1
                    openlist.put((node.accumulated_cost + 1 + self.heuristic(successor_state), node_count, make_child_node(node, operator, successor_state, 1))) # assume uniform cost
                    closed[successor_state] = node.accumulated_cost + 1
            stats.nexpansions += 1

        # logging.info(f"Search space exhausted. # expanded: {stats.nexpansions}, # goals: {stats.num_goals}.")
        # space.complete = True
        return None, stats

class TreeSearch:
    """ Full expansion of a problem through Breadth-First search.
    Note that ATM we return no plan.
    """
    def __init__(self, model: GroundForwardSearchModel, max_expansions=-1, heuristic=zero_heuristic):
        self.model = model
        self.max_expansions = max_expansions
        self.heuristic = heuristic

    def run(self):
        return self.search(self.model.init())

    def search(self, root):
        # create obj to track state space
        # space = SearchSpace()

        tree = SearchTree(self.model, root)
        while self.max_expansions<=0 or tree.nexpansions<=self.max_expansions:
            node = tree.getFrontier()
            if self.model.is_goal(node.state):
                return node.extractPath(), tree
            tree.expand(node, self.heuristic)

        return None, None


class SearchNode:
    def __init__(self, state, parent, action, accumulated_cost = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.accumulated_cost = accumulated_cost

    def extractPath(self):
        if self.action:
            path = self.parent.extractPath() + [self.action]
        else:
            path = []

        return path

def best_child_val(children):
    best_child = None
    best_val = float('inf')

    for child in children:
        if child.h < best_val:
            best_child = child
            best_val = child.h

    return best_child, best_val

def UCB1_child_val(node):
    best_child = None
    best_val = float('inf')
    for child in node.children:
        # print(node.visits)
        # print(child.visits)
        ucb = child.h - math.sqrt(2*math.log(node.visits)/child.visits)
        if ucb < best_val:
            best_child = child
            best_val = ucb

    return best_child, best_val

class SearchTree:
    def __init__(self, model, state):
        self.model = model
        self.root = TreeNode(model, state, None, None)
        self.nexpansions = 0
        self.closed = dict(state=(0,self.root))

    def getFrontier(self):
        node = self.root
        while node.expanded:
            unexpanded_children = [child for child in node.children if child.expanded == False]
            if unexpanded_children:
                node, val = best_child_val(unexpanded_children)
            else:
                node, val = UCB1_child_val(node)

        return node

    def expand(self, node, heuristic):
        self.nexpansions += 1
        node.expanded = True
        for operator, successor_state in self.model.successors(node.state):
            if successor_state not in self.closed or self.closed[successor_state][0] > node.accumulated_cost + 1:
                node.children.append(TreeNode(self.model, successor_state, node, operator, node.accumulated_cost+1, heuristic)) # assume uniform cost

                if successor_state in self.closed:
                    old_parent = self.closed[successor_state][1]
                    for child in old_parent.children:
                        if child.state == successor_state:
                            old_parent.children.remove(child)
                            break

                self.closed[successor_state] = (node.accumulated_cost + 1, node)


        node.update()


class TreeNode:
    def __init__(self, model, state, parent, action, accumulated_cost = 0, heuristic = zero_heuristic):
        self.model = model
        self.state = state
        self.parent = parent
        self.action = action
        self.accumulated_cost = accumulated_cost
        self.visits = 0
        self.h = heuristic(state)
        self.children = []
        self.expanded = False

    def extractPath(self):
        if self.action:
            path = self.parent.extractPath() + [self.action]
        else:
            path = []

        return path

    def update(self):
        self.visits += 1
        val = float('inf')
        for child in self.children:
            if child.h < val:
                val = child.h

        self.h = val + 1
        if self.parent:
            self.parent.update()

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


def make_child_node(parent_node, action, state, action_cost):
    """ Construct an child search node """
    return SearchNode(state, parent_node, action, parent_node.accumulated_cost + action_cost)
