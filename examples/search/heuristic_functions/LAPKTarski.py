import time
import tarski
import random
from tarski.syntax import Atom, CompoundFormula, land, Connective
import tarski.fstrips as fs
import tarski.io.fstrips as fsio

class writer:
    """ Only works for grounded theories planning action """

    def __init__(self):
        self.atom_table = None

    def formatState( self, atom_name_list ):
        return [ (self.atom_table[ "{}".format(atom)], False) for atom in atom_name_list ]

    """ Get goal formula recursively, as Tarski nests the goal with -and- connectives"""
    def getGoal(self, formula):
        ret = []
        for f in formula:
            if isinstance(f, Atom):
                ret.append( "({})".format(f.predicate.name) )
            elif isinstance(f, CompoundFormula):
                ret += self.getGoal(f.subformulas)
        return ret


    def groundedTarski( self, planner, problem ) :
        parsing_time_init = time.perf_counter()

        print("Domain: %s Problem: %s"%(problem.domain_name, problem.name) )


        # Adding atoms to the problem
        index = 0
        self.atom_table = {}
        atom_names =  [ "({})".format(f.symbol) for f in problem.language.predicates ]

        atom_names.sort()

        for atom in atom_names :
            self.atom_table[ atom ] = index
            planner.add_atom( atom )
            index += 1


        # Notifying that there may be atoms deleted. We do not add negated atoms explicitly
        planner.create_negated_fluents()
        print("Deterministic %d actions" % len(problem.actions))

        # Adding actions
        index = 0
        for action in problem.actions.values():

            # Name
            planner.add_action( action.name )

            # Preconditions: list of tuple(index, BooleanNegated)
            action_tarski = self.getGoal(action.precondition.subformulas)
            preconditions = [ (self.atom_table[f], False) for f in action_tarski ]
            planner.add_precondition( index, preconditions )

            # Effects: list of tuple(index, BooleanNegated). Note that Delete effects are introduced turning the boolean to True
            effects = [ ( self.atom_table[ "({})".format(e.atom.predicate.name)], False) if isinstance(e, fs.AddEffect)  else \
                ( self.atom_table[ "({})".format(e.atom.predicate.name)], True)  for e in action.effects ]
            planner.add_effect( index, effects )

            # Set action cost, if none is defined use 1
            cost = action.cost.addend.rhs if action.cost else 1.0
            planner.set_cost( index, cost )

            index += 1

        # Add domain and problem names
        planner.set_domain_name( problem.domain_name )
        planner.set_problem_name( problem.name )

        # Set Initial state
        init = [ (self.atom_table[ "({})".format(signature[0])], False) for signature, _ in problem.init.predicate_extensions.items() ]
        planner.set_init( init )

        #Set Goal state

        if not isinstance(problem.goal, CompoundFormula):
            formulas = [problem.goal]
        else:
            formulas = problem.goal.subformulas

        goal_tarski = self.getGoal( formulas )
        goal = [ (self.atom_table[g] , False ) for g in goal_tarski ]
        planner.set_goal( goal )

        parsing_time_end = time.perf_counter()
        planner.parsing_time = parsing_time_end - parsing_time_init
