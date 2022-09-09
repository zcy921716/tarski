import sys
import os

#Compiled API from LAPKT
from libtarskiplanner import Planner 

#Tarski to LAPKT writer
import LAPKTarski

#Grid Generator
import gridTarskiGenerator

#Profiler imports
#from prof import profiler_start, profiler_stop
    
def main(  ) :
	
    #NIR: Generate Tarksi Grid
    lang, ats, visiteds, connecteds = gridTarskiGenerator.gridworldGenerator(10, 10)
    problem = gridTarskiGenerator.gridworldProblemGenerator(lang, ats, visiteds, connecteds, 'test_grounding')

    # NIR: Write to PDDL from Tarski
    print(problem)
    gridTarskiGenerator.write_to_pddl( problem )

    # NIR: Create LAPKT Planner
    planner = Planner( )
    
    # NIR: Load Tarksi problem into the Planner
    lw = LAPKTarski.writer( )
    
    # NIR: Load Grounded problem
    lw.groundedTarski(planner, problem)
    
    # #NIR: Uncomment to check what actions are being loaded
    # for i in range( 0, planner.num_actions() ) :
    # 	planner.print_action( i )

    # NIR: Setting planner parameters is as easy as setting the values
    # of Python object attributes

    # #NIR: Ignore action costs even if they are given. Treat them all as unit cost
    # planner.ignore_action_costs = True
    
    # NIR: log filename set
    planner.log_filename = 'planner.log'

    # NIR: plan file
    planner.plan_filename = "plan"

    # NIR: Comment line below to deactivate profiling
    #profiler_start( 'planner.prof' )

    # NIR: We call the setup method in Planner
    planner.setup()

    # NIR: And then we're ready to go to compute any heurstic

    h_lmcount = planner.eval_landmarks_init()
    print("Landmarkc Heuristic from initial state is {}".format(h_lmcount))

    h_max = planner.eval_hmax_init()
    print("H_Max Heuristic from initial state is {}".format(h_max))


    # Create list of atom_names from Tarski initial state, but discard (at ) and (visited ) fluents
    new_state = [ "({})".format(signature[0]) for signature, _ in problem.init.predicate_extensions.items() if 'at' not in signature[0] and 'visited' not in signature[0] ]

    new_state.append("(at_0_0)")
    new_state.append("(visited_0_0)")

    # NIR: Format the state into LAPKT State API
    new_state_formatted = lw.formatState( new_state )

    # NIR: Compute heuristic from a state
    h_max = planner.eval_hmax( new_state_formatted )
    print("H_Max Heuristic from different state is {}".format(h_max))

    #NIR: Comment lines below to deactivate profile
    #profiler_stop()	

    #rv = os.system( 'google-pprof --pdf libsiw.so planner.prof > planner.pdf' )
    #if rv != 0 :
    #	print >> sys.stderr, "An error occurred while translating google-perftools profiling information into valgrind format"


def debug() :
	main( )

if __name__ == "__main__":
	main(  )

