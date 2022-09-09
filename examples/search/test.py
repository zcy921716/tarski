#!/usr/bin/env python3
from tarski.io import PDDLReader
from tarski.search import GroundForwardSearchModel, BreadthFirstSearch, AStarSearch, TreeSearch
from heuristic_functions import LAPKTarski, Planner



    # NIR: Comment line below to deactivate profiling

def main():
    domain_file = "../data/pddl/TOL/domain.pddl"
    problem_file = "../data/pddl/TOL/problem.pddl"

    reader = PDDLReader(raise_on_error = True)
    reader.parse_domain(domain_file)
    problem = reader.parse_instance(problem_file)

    model = GroundForwardSearchModel(problem)


    BFS = BreadthFirstSearch(model)
    path, info = BFS.run()
    plan = ["(" + action.name + ")" for action in path]
    print(info.nexpansions)
    with open("../data/pddl/TOL/test_BFS_plan.dat", "w+") as f:
        f.write("\n".join(plan))

    planner = Planner( )

    # NIR: Load Tarksi problem into the Planner
    lw = LAPKTarski.writer( )

    # NIR: Load Grounded problem
    lw.groundedTarski(planner, problem)

    planner.log_filename = 'planner.log'

    # NIR: plan file
    planner.plan_filename = "plan"


    #profiler_start( 'planner.prof' )

    # NIR: We call the setup method in Planner
    planner.setup()

    def get_lmvalue(state):
        formatedState = ["({})".format(signature[0]) for signature, _ in state.predicate_extensions.items() if signature[0] in [item.predicate.name for item in state.as_atoms()]]
        return planner.eval_hff(lw.formatState(formatedState))
    def get_gcvalue(state):
        formatedState = ["({})".format(signature[0]) for signature, _ in state.predicate_extensions.items() if signature[0] in [item.predicate.name for item in state.as_atoms()]]
        return planner.eval_hgc(lw.formatState(formatedState))

    AStar = AStarSearch(model, heuristic = get_lmvalue)
    path, info = AStar.run()
    print(info.nexpansions)
    plan = ["(" + action.name + ")" for action in path]
    with open("../data/pddl/TOL/test_AStar_plan.dat", "w+") as f:
        f.write("\n".join(plan))

    tree = TreeSearch(model, heuristic = get_gcvalue)
    path, info = tree.run()
    print(info.nexpansions)
    plan = ["(" + action.name + ")" for action in path]
    with open("../data/pddl/TOL/test_tree_plan.dat", "w+") as f:
        f.write("\n".join(plan))



if __name__ == "__main__":
    main()
