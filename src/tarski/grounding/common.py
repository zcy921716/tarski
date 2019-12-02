
from ..syntax import Predicate, Function, Constant, termlists_are_equal, termlist_hash
from ..fstrips import AddEffect, DelEffect, FunctionalEffect, LinearEffect
from .. import errors as err


class StateVariableLite:
    """ A state variable is a ground CompoundTerm or Atom which can possibly change its value along the execution of a
    plan. The set of all state variables of a problem makes up all information necessary to represent a state.
    State variables are different to static atoms, whose truth value can be proven to remain the same.
    This proof can be based on simple techniques such as looking at the effects of actions, or on more sophisticated
    reachability analyses.

    Note that we could use the CompoundTerm or Atom classes to represent the same concept represented by a
    StateVariableLite, but currently we prefer to use a single class, hence the existence of StateVariableLite.
    Note: This is a lightweight version of the StateVariable class above, hoping that it can eventually replace it.
    """

    def __init__(self, symbol, binding):
        assert isinstance(symbol, (Predicate, Function))
        assert all(isinstance(c, Constant) for c in binding)
        self.symbol = symbol
        self.binding = binding

    def __hash__(self):
        return hash((self.symbol, termlist_hash(self.binding)))

    def __eq__(self, other):
        return self.symbol == other.symbol and termlists_are_equal(self.binding, other.binding)

    def __str__(self):
        return '{}({})'.format(self.symbol.symbol, ','.join(map(str, self.binding)))

    __repr__ = __str__

    @staticmethod
    def from_atom(atom):
        return StateVariableLite(atom.predicate, atom.subterms)


def approximate_symbol_fluency(problem):
    """ Sort out all predicate and function symbols of a given problem into static and fluent symbols.

    Fluent symbols are those whose denotation might (in principle) be affected by the effect of some action schema;
    the rest are static symbols. This methods works on a purely syntactic basis, and does not take into account any
    reachability information, nor whether the action schemas can be detected as unapplicable at preprocessing. This
    for instance means that an action a(x) with precondition False and atomic add effect "p(x)" will be taken as
    evidence that predicate symbol "p" is fluent. It also means that "fluency" is only considered at the symbol
    level (i.e. "p" is either fluent or static), not at the state variable level (where e.g. p(a) could be seen as
    static, and p(b) as fluent).

    The results of this method overapproaximate the set of symbol fluents (i.e. false positives are possible, false
    negatives are not).

    This can be used as a basis for a finer-grained analysis that can be performed e.g. with the ASP-based reachability
    grounder.
    """
    lang = problem.language
    all_symbols = set(lang.functions) | set(lang.predicates)

    # All symbols appearing on some action, process or event effect are fluent
    fluents = set()
    for action in problem.actions.values():
        for eff in action.effects:
            symbols = _compute_effect_head_symbol(eff)
            if isinstance(symbols, list):
                fluents |= set(symbols)
            else:
                fluents |= set([symbols])
    # fluents = set(_compute_effect_head_symbol(eff) for action in problem.actions.values() for eff in action.effects)

    # The rest of symbols are considered static
    statics = set(s for s in all_symbols if not s.builtin and s not in fluents)
    return fluents, statics


def _compute_effect_head_symbol(effect):
    """ Return the symbol affected by a given effect. """
    if isinstance(effect, (AddEffect, DelEffect)):
        return effect.atom.predicate
    elif isinstance(effect, FunctionalEffect):
        return effect.lhs.symbol
    elif isinstance(effect, LinearEffect):
        return [lhs.symbol for lhs in effect.y[:,0]]
    else:
        raise err.UnexpectedElementType(effect)
