#include <planner.hxx>
using namespace boost::python;

BOOST_PYTHON_MODULE( libtarskiplanner )
{
	class_<Planner>("Planner")
		.def( init< std::string, std::string >() )
		.def( "add_atom", &Planner::add_atom )
		.def( "add_action", &Planner::add_action )
		.def( "add_mutex_group", &Planner::add_mutex_group )
		.def( "num_atoms", &Planner::n_atoms )
		.def( "num_actions", &Planner::n_actions )
		.def( "get_atom_name", &Planner::get_atom_name )
		.def( "get_domain_name", &Planner::get_domain_name )
		.def( "get_problem_name", &Planner::get_problem_name )
		.def( "add_precondition", &Planner::add_precondition )
		.def( "add_effect", &Planner::add_effect )
		.def( "add_cond_effect", &Planner::add_cond_effect )
		.def( "set_cost", &Planner::set_cost )
		.def( "notify_negated_conditions", &Planner::notify_negated_conditions )
		.def( "create_negated_fluents", &Planner::create_negated_fluents )
		.def( "set_init", &Planner::set_init )
		.def( "set_goal", &Planner::set_goal )
		.def( "set_domain_name", &Planner::set_domain_name )
		.def( "set_problem_name", &Planner::set_problem_name )
		.def( "write_ground_pddl", &Planner::write_ground_pddl )
		.def( "print_action", &Planner::print_action )
		.def_readwrite( "ignore_action_costs", &Planner::m_ignore_action_costs )
		.def_readwrite( "parsing_time", &Planner::m_parsing_time )
		.def_readwrite( "log_filename", &Planner::m_log_filename )
		.def( "setup", &Planner::setup )
		.def( "eval_landmarks_init", &Planner::eval_landmarks_init )
		.def( "eval_hmax_init", &Planner::eval_hmax_init )
		.def( "eval_hmax", &Planner::eval_hmax )
		.def( "eval_hadd", &Planner::eval_hadd )
		.def( "eval_hff", &Planner::eval_hff )
		.def( "eval_h2", &Planner::eval_h2 )
		.def( "eval_hmin", &Planner::eval_hmin )
		.def( "eval_hgc", &Planner::eval_hgc )
		.def( "generate_best_action", &Planner::generate_best_action )
		.def( "optimal_cost", &Planner::optimal_cost )
		.def( "expanded_node", &Planner::expanded_node )
		.def( "generated_node", &Planner::generated_node )
		.def( "all_info", &Planner::all_info )



	;
}
