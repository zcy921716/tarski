#!/usr/bin/env python3
from tarski.io import PDDLReader
from tarski.search import GroundForwardSearchModel, DepthFirstSearch, BreadthFirstSearch, AStarSearch, TreeSearch, GreedySearch
from heuristic_functions import LAPKTarski, Planner
import json, os


    # NIR: Comment line below to deactivate profiling

def main():
    problem_id = 0
    instance_folder = "../data/pddl/TOLdataset"
    while os.path.exists(instance_folder+"/TOL_"+str(problem_id)):
    # use new PDDL model (with handempty predicate)
        folder = instance_folder+"/TOL_"+str(problem_id)
        domain_file = folder + "/domain_2.pddl"
        problem_file = folder + "/problem_2.pddl"

        reader = PDDLReader(raise_on_error = True)
        reader.parse_domain(domain_file)
        problem = reader.parse_instance(problem_file)

        model = GroundForwardSearchModel(problem)

        with open(folder + "/info.json") as f:
            info_dict = json.load(f)

        # BFS
        BFS = BreadthFirstSearch(model)
        path, info = BFS.run()
        plan = ["(" + action.name + ")" for action in path]
        print(info.nexpansions)


        info_dict['sol_length(BFS)'] = len(plan)
        info_dict['num_expanded_nodes(BFS)'] = info.nexpansions

        # DFS
        DFS = DepthFirstSearch(model)
        path, info = DFS.run()
        plan = ["(" + action.name + ")" for action in path]
        print(info.nexpansions)

        info_dict['sol_length(DFS)'] = len(plan)
        info_dict['num_expanded_nodes(DFS)'] = info.nexpansions

        # heuristic search
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

        # Astar + goal counting
        AStar = AStarSearch(model, heuristic = get_lmvalue)
        path, info = AStar.run()
        print(info.nexpansions)
        plan = ["(" + action.name + ")" for action in path]

        info_dict['sol_length(astar+gc)'] = len(plan)
        info_dict['num_expanded_nodes(astar+gc)'] = info.nexpansions


        # Greedy + goal counting
        Greedy = GreedySearch(model, heuristic = get_lmvalue)
        path, info = Greedy.run()
        print(info.nexpansions)
        plan = ["(" + action.name + ")" for action in path]

        info_dict['sol_length(greedy+gc)'] = len(plan)
        info_dict['num_expanded_nodes(greedy+gc)'] = info.nexpansions

        # tree = TreeSearch(model, heuristic = get_gcvalue)
        # path, info = tree.run()
        # print(info.nexpansions)
        # plan = ["(" + action.name + ")" for action in path]
        # with open("../data/pddl/TOL/test_tree_plan.dat", "w+") as f:
        #     f.write("\n".join(plan))
        with open(folder + "/info.json", 'w') as f:
            json.dump(info_dict, f)
        problem_id += 1

if __name__ == "__main__":
    main()
