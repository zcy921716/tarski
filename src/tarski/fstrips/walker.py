""" A Walker (Visitor) for FSTRIPS entities.
Note that there is some code duplication with the FOLWalker at syntax/walker.py,
but that one is only in charge of FOL elements and should remain agnostic wrt planning, FSTRIPS, actions, effects, etc.
"""

import copy
from enum import Enum

from ..errors import TarskiError
from ..utils.algorithms import dispatch


class WalkerError(TarskiError):
    def __init__(self, msg=None):
        msg = msg or 'Unspecified error while executing ProblemWalker'
        super().__init__(msg)


class NoHandlerError(WalkerError):
    def __init__(self, node):
        super().__init__(f'ProblemWalker: No handler was specified for node "{node}" of type "{type(node)}"')


class WalkerAction(Enum):
    """ """
    Supress = "supress"

    def __str__(self):
        return self.value


class WalkerContext(Enum):
    """ """
    Effect = "effect"
    Formula = "formula"

    def __str__(self):
        return self.value


class ProblemWalker:
    """
    """
    def __init__(self, raise_on_undefined=False):
        self.default_handler = self._raise if raise_on_undefined else self._donothing
        self.context = None

    def _raise(self, node):
        raise NoHandlerError(node)

    def _donothing(self, node):
        return node

    @dispatch
    def visit(self, node):
        return self.default_handler(node)

    def run(self, expression, inplace=True):
        from . import Action, BaseEffect, Problem  # Import here to break circular refs
        from ..syntax import Formula, Term
        # Simply dispatch according to type
        expression = expression if inplace else copy.deepcopy(expression)
        if isinstance(expression, (Formula, Term)):
            self.visit_expression(expression, inplace=True)
        elif isinstance(expression, BaseEffect):
            self.visit_effect(expression, inplace=True)
        elif isinstance(expression, Action):
            self.visit_effect(expression, inplace=True)
        elif isinstance(expression, Problem):
            self.visit_problem(expression, inplace=True)
        return expression

    def visit_problem(self, problem, inplace=False):
        problem = problem if inplace else copy.deepcopy(problem)
        problem.goal = self.visit_expression(problem.goal, inplace=True)

        for aname, a in list(problem.actions.items()):
            res = self.visit_action(a, inplace=True)
            if res is WalkerAction.Supress:
                del problem.actions[aname]

        return problem

    def visit_action(self, node, inplace=False):
        node = node if inplace else copy.deepcopy(node)
        node.precondition = self.visit_expression(node.precondition, inplace=True)

        node.effects = self.accept(self.visit_effect(eff, inplace=True) for eff in node.effects)

        return node

    def visit_effect(self, effect, inplace=True):
        from . import AddEffect, DelEffect, UniversalEffect, FunctionalEffect  # Import here to break circular refs
        effect = effect if inplace else copy.deepcopy(effect)

        if isinstance(effect, (AddEffect, DelEffect)):
            effect.condition = self.visit_expression(effect.condition)
            effect.atom = self.visit_effect_atom(effect.atom)

        elif isinstance(effect, FunctionalEffect):
            effect.condition = self.visit_expression(effect.condition)
            effect.lhs = self.visit_effect_atom(effect.lhs)
            effect.rhs = self.visit_effect_atom(effect.rhs)

        elif isinstance(effect, UniversalEffect):
            effect.effects = self.accept(self.visit_effect(eff, inplace=True) for eff in effect.effects)

        else:
            raise RuntimeError(f'Effect "{effect}" of type "{type(effect)}" cannot be analysed')

        return self.visit(effect)

    def visit_expression(self, node, inplace=True):
        from ..syntax import CompoundFormula, QuantifiedFormula, Atom, Tautology, Contradiction, Constant, Variable,\
            CompoundTerm, IfThenElse  # Import here to break circular refs
        node = node if inplace else copy.deepcopy(node)

        if isinstance(node, (Variable, Constant, Contradiction, Tautology)):
            pass

        elif isinstance(node, (CompoundTerm, Atom)):
            node.subterms = self.accept(self.visit_expression(sub, inplace=True) for sub in node.subterms)

        elif isinstance(node, CompoundFormula):
            node.subformulas = self.accept(self.visit_expression(sub, inplace=True) for sub in node.subformulas)

        elif isinstance(node, IfThenElse):
            node.condition = self.visit_expression(node.condition, inplace=True)
            node.subterms = self.accept(self.visit_expression(sub, inplace=True) for sub in node.subterms)

        elif isinstance(node, QuantifiedFormula):
            node.formula = self.visit_expression(node.formula)
            node.variables = self.accept(self.visit_expression(eff, inplace=True) for eff in node.variables)
        else:
            raise RuntimeError(f'Unexpected expression "{node}" of type "{type(node)}"')

        return self.visit(node)

    def visit_effect_atom(self, node):
        self.context = WalkerContext.Effect
        x = self.visit_expression(node)
        self.context = WalkerContext.Formula
        return x

    def accept(self, iterator):
        return [x for x in iterator if x is not WalkerAction.Supress]