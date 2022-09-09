#include <planner.hxx>
#include <strips_state.hxx>
#include <aptk/string_conversions.hxx>

#include <iostream>
#include <fstream>

using	aptk::agnostic::Fwd_Search_Problem;
using namespace boost::python;

Planner::Planner()
	: STRIPS_Problem(), m_log_filename( "planner.log"), m_plan_filename( "plan.ipc" ) {
}

Planner::Planner( std::string domain_file, std::string instance_file )
	: STRIPS_Problem( domain_file, instance_file ),  m_log_filename( "planner.log" ), m_plan_filename( "plan.ipc" ) {
}

Planner::~Planner() {

	// Create Forward state space
	delete m_search_prob;

	// Create Landmarkc count Heuristic
	delete m_lm_count;

	// Landmark Graph Generator
	delete m_gen_lms;

	// Create Hmax heuristic
	delete m_hmax;
	delete m_hadd;
	delete m_hff;
	delete m_h2;
	delete m_hmin;
	delete m_hgc;

	delete engine;
	delete engine_gc;

	// Initial State
	delete m_init_state;
}


void
Planner::setup() {
	//Call superclass method, then do you own thing here
	STRIPS_Problem::setup();
	std::cout << "PDDL problem description loaded: " << std::endl;
	std::cout << "\tDomain: " << instance()->domain_name() << std::endl;
	std::cout << "\tProblem: " << instance()->problem_name() << std::endl;
	std::cout << "\t#Actions: " << instance()->num_actions() << std::endl;
	std::cout << "\t#Fluents: " << instance()->num_fluents() << std::endl;

	// Create Forward state space
	m_search_prob = new Fwd_Search_Problem( instance() );

	// Initial State
	m_init_state = m_search_prob->init();

	// Create Landmarkc count Heuristic
	m_lm_count = new H_Lmcount_Fwd( *m_search_prob );

	// Landmark Graph Generator
	m_gen_lms = new Gen_Lms_Fwd( *m_search_prob );

	// Create Hmax heuristic
	m_hmax = new H_Max_Fwd( *m_search_prob );
	m_hadd = new H_Add_Fwd( *m_search_prob );
	m_hff = new H_FF_Fwd( *m_search_prob );
	m_h2 = new H2_Fwd( *m_search_prob );
	m_hmin = new H_Min_Fwd( *m_search_prob );
	m_hgc = new H_GC_Fwd( *m_search_prob );

	engine = new BFS_H_Lmcut_Fwd ( *m_search_prob );
	engine_gc = new BFS_H_GC_Fwd ( *m_search_prob );
}


float Planner::eval_landmarks_init(){

	float val = 0.0;

	// Landmark Graph (initialized using )
	Landmarks_Graph graph( *instance() );

	// Compute Landmark Graph
	m_gen_lms->compute_lm_graph_set_additive( graph );

	std::cout << "Total number of Landmarks found: " << graph.num_landmarks() << std::endl;

	// Attach Computed Landmarkc graph to the count Heuristic
	m_lm_count->set_graph( &graph );

	// Evaluate Heuristic
	m_lm_count->eval( *m_init_state, val );

	return val;

}

float Planner::eval_hmax_init(){

	float val = 0.0;

	// Evaluate Heuristic
	m_hmax->eval( *m_init_state, val );

	return val;

}

float Planner::eval_hmax( boost::python::list& fluent_list ){

	State s( *instance() );

	for( int i = 0; i < len(fluent_list); i++ ) {
		boost::python::tuple li = extract< tuple >( fluent_list[i] );
		int 	fl_idx 		= extract<int>(li[0]);
		bool	negated 	= extract<bool>(li[1]);
		if ( negated ) {
			s.set( m_negated[fl_idx]->index() );
			continue;
		}
		s.set( fl_idx );
	}

	float val = 0.0;

	// Evaluate Heuristic
	m_hmax->eval( s, val );



	return val;

}

float Planner::eval_hadd( boost::python::list& fluent_list ){

	State s( *instance() );

	for( int i = 0; i < len(fluent_list); i++ ) {
		boost::python::tuple li = extract< tuple >( fluent_list[i] );
		int 	fl_idx 		= extract<int>(li[0]);
		bool	negated 	= extract<bool>(li[1]);
		if ( negated ) {
			s.set( m_negated[fl_idx]->index() );
			continue;
		}
		s.set( fl_idx );
	}

	float val = 0.0;

	// Evaluate Heuristic
	m_hadd->eval( s, val );



	return val;

}

float Planner::eval_hff( boost::python::list& fluent_list ){

	State s( *instance() );

	for( int i = 0; i < len(fluent_list); i++ ) {
		boost::python::tuple li = extract< tuple >( fluent_list[i] );
		int 	fl_idx 		= extract<int>(li[0]);
		bool	negated 	= extract<bool>(li[1]);
		if ( negated ) {
			s.set( m_negated[fl_idx]->index() );
			continue;
		}
		s.set( fl_idx );
	}

	float val = 0.0;

	// Evaluate Heuristic
	m_hff->eval( s, val );



	return val;

}

float Planner::eval_h2( boost::python::list& fluent_list ){

	State s( *instance() );

	for( int i = 0; i < len(fluent_list); i++ ) {
		boost::python::tuple li = extract< tuple >( fluent_list[i] );
		int 	fl_idx 		= extract<int>(li[0]);
		bool	negated 	= extract<bool>(li[1]);
		if ( negated ) {
			s.set( m_negated[fl_idx]->index() );
			continue;
		}
		s.set( fl_idx );
	}

	float val = 0.0;

	// Evaluate Heuristic
	m_h2->eval( s, val );



	return val;

}

float Planner::eval_hmin( boost::python::list& fluent_list ){

	State s( *instance() );

	for( int i = 0; i < len(fluent_list); i++ ) {
		boost::python::tuple li = extract< tuple >( fluent_list[i] );
		int 	fl_idx 		= extract<int>(li[0]);
		bool	negated 	= extract<bool>(li[1]);
		if ( negated ) {
			s.set( m_negated[fl_idx]->index() );
			continue;
		}
		s.set( fl_idx );
	}

	float val = 0.0;


	// Evaluate Heuristic
	m_hmin->eval( s, val );



	return val;

}

float Planner::eval_hgc( boost::python::list& fluent_list ){

	State s( *instance() );

	for( int i = 0; i < len(fluent_list); i++ ) {
		boost::python::tuple li = extract< tuple >( fluent_list[i] );
		int 	fl_idx 		= extract<int>(li[0]);
		bool	negated 	= extract<bool>(li[1]);
		if ( negated ) {
			s.set( m_negated[fl_idx]->index() );
			continue;
		}
		s.set( fl_idx );
	}

	float val = 0.0;


	// Evaluate Heuristic
	m_hgc->eval( s, val );



	return val;

}

std::string Planner::generate_best_action(){

	std::vector< aptk::Action_Idx> plan;
	int cost = int_infty;
	engine->set_greedy( false );
	engine->set_delay_eval( false );
	engine->start();


	engine->find_solution( cost, plan );
	return ( m_search_prob->task().actions()[ plan[0] ] )->signature();

}

int Planner::optimal_cost(){
	std::vector< aptk::Action_Idx> plan;
	int cost = int_infty;
	engine->set_greedy( false );
	engine->set_delay_eval( false );
	engine->start();


	engine->find_solution( cost, plan );
	return cost;
}

int Planner::expanded_node(){
	std::vector< aptk::Action_Idx> plan;
	int cost = int_infty;

	engine->set_greedy( false );
	engine->set_delay_eval( false );
	engine->start();

	int e0 = engine->expanded();
	engine->find_solution( cost, plan );
	return engine->expanded() - e0;
}

int Planner::generated_node(){
	std::vector< aptk::Action_Idx> plan;
	int cost = int_infty;

	engine->set_greedy( false );
	engine->set_delay_eval( false );
	engine->start();

	int g0 = engine->generated();
	engine->find_solution( cost, plan );
	return engine->generated() - g0;
}

std::string Planner::all_info(){
	std::vector< aptk::Action_Idx> plan;
	int cost = int_infty;

	engine->set_greedy( false );
	engine->set_delay_eval( false );
	engine->start();

	int g0 = engine->generated();
	int e0 = engine->expanded();
	engine->find_solution( cost, plan );


	return std::to_string(cost) + " " + std::to_string(engine->generated()-g0) + " " + std::to_string(engine->expanded()-e0);
}
