from tarski.syntax import Variable, CompoundTerm, LiftedNestedTerm
from ... import errors as err
from ... grounding.naive import instantiation
from .. transform import TermSubstitution
from .. builtins import BuiltinPredicateSymbol, BuiltinFunctionSymbol

def sum( *args ):
    """
        Summation of a (nested) sequence of expressions defined over one or more
        variables.
    """
    expr = args[-1]
    if not isinstance(expr, CompoundTerm):
        raise err.SyntacticError(msg='sum(x0,x1,...,xn,expr) requires last \
        argument "expr" to be an instance of CompoundTerm')
    vars = []
    for x in args[:-1]:
        if not isinstance(x, Variable):
            raise err.SyntacticError(msg='sum(x0,...,xn,expr) require each\
            argument xi to be an instance of Variable')
        vars.append(x)

    L = expr.language
    K, syms, substs = instantiation.enumerate_groundings(L, list(vars))
    processed_expr = []
    for values in itertools.product(*substs):
        subst = {syms[k]: v for k, v in enumerate(values)}
        expr_subst = copy.deepcopy(expr)
        op = TermSubstitution(L, subst)
        expr_subst.accept(op)
        processed_expr.append(expr_subst)

    lhs = processed_expr[0]
    for k in range(1,len(processed_expr)):
        lhs = L.dispatch_operator(BuiltinFunctionSymbol.ADD, CompoundTerm, CompoundTerm, lhs, processed_expr[k])

    return lhs