#ifndef __PLANNER__
#define __PLANNER__

#include <boost/python.hpp>

#include <py_strips_prob.hxx>
#include <fwd_search_prob.hxx>
#include <strips_state.hxx>

//Include several heuristics
#include <landmark_graph.hxx>
#include <landmark_graph_generator.hxx>
#include <landmark_graph_manager.hxx>
#include <landmark_count.hxx>
#include <h_2.hxx>
#include <h_1.hxx>
#include <h_unsat.hxx>
#include <rp_heuristic.hxx>


// #include <ff_to_aptk.hxx>
#include <strips_prob.hxx>
#include <fluent.hxx>
#include <action.hxx>
#include <cond_eff.hxx>

//#include <h_1.hxx>

#include <aptk/open_list.hxx>
#include <aptk/string_conversions.hxx>
#include "at_bfs.hxx"

#include <fstream>

#include <boost/program_options.hpp>

#include "h_lmcut.hxx"

namespace po = boost::program_options;


using	aptk::Action;

using 	aptk::agnostic_int::LMCUT_Heuristic;

using 	aptk::search::Open_List;
using	aptk::search::bfs::AT_BFS_SQ_SH;
using	aptk::search::Node_Comparer;


// shorten their namespace for code legibility
using	aptk::agnostic::Fwd_Search_Problem;
using	aptk::State;

using 	aptk::agnostic::Landmarks_Graph;
using 	aptk::agnostic::Landmarks_Graph_Generator;
using   aptk::agnostic::Landmarks_Graph_Manager;
using 	aptk::agnostic::Landmarks_Count_Heuristic;

using 	aptk::agnostic::H2_Heuristic;
using 	aptk::agnostic::H1_Heuristic;
using	aptk::agnostic::H_Add_Evaluation_Function;
using	aptk::agnostic::H_Max_Evaluation_Function;
using	aptk::agnostic::H_Min_Evaluation_Function;
using	aptk::agnostic::Unsat_Goals_Heuristic;
using	aptk::agnostic_int::H_Add_Evaluation_Function_int;
using	aptk::agnostic_int::H_Max_Evaluation_Function_int;
using	aptk::agnostic_int::H_Min_Evaluation_Function_int;
using	aptk::agnostic::Relaxed_Plan_Heuristic;

// NIR: Define the heuristics


typedef		aptk::search::bfs::Node< State >	Search_Node;

//NIR: Then we define the type of the tie-breaking algorithm
// for the open list we are going to use
typedef		Node_Comparer< Search_Node >					Tie_Breaking_Algorithm;


//NIR: Now we define the Open List type by combining the types we have defined before
typedef		Open_List< Tie_Breaking_Algorithm, Search_Node >		BFS_Open_List;


//NIR: Now we define the heuristics

typedef 	LMCUT_Heuristic<Fwd_Search_Problem, H_Max_Evaluation_Function_int>	H_Lmcut_Fwd;
typedef		Unsat_Goals_Heuristic<Fwd_Search_Problem>	H_GC_Fwd; //, aptk::agnostic::H1_Cost_Function::Ignore_Costs
typedef		H1_Heuristic<Fwd_Search_Problem, H_Max_Evaluation_Function>	H_Max_Fwd; //, aptk::agnostic::H1_Cost_Function::Ignore_Costs
typedef		Relaxed_Plan_Heuristic< Fwd_Search_Problem, H_Max_Fwd >		H_FF_Fwd;

//NIR: Now we're ready to define the BFS algorithm we're going to use
typedef		AT_BFS_SQ_SH< Fwd_Search_Problem, H_FF_Fwd, BFS_Open_List >		BFS_H_Lmcut_Fwd;
typedef		AT_BFS_SQ_SH< Fwd_Search_Problem, H_GC_Fwd, BFS_Open_List >		BFS_H_GC_Fwd;
// Critical paths
typedef         H2_Heuristic<Fwd_Search_Problem>                  H2_Fwd;

// Goal and Landmark Counting
typedef         Landmarks_Graph_Generator<Fwd_Search_Problem>     Gen_Lms_Fwd;
typedef         Landmarks_Count_Heuristic<Fwd_Search_Problem>     H_Lmcount_Fwd;
typedef         Landmarks_Graph_Manager<Fwd_Search_Problem>       Land_Graph_Man;

// Delete Relaxation heuristics
typedef		H1_Heuristic<Fwd_Search_Problem, H_Add_Evaluation_Function>	H_Add_Fwd; //, aptk::agnostic::H1_Cost_Function::Ignore_Costs

typedef		H1_Heuristic<Fwd_Search_Problem, H_Min_Evaluation_Function>	H_Min_Fwd; //, aptk::agnostic::H1_Cost_Function::Ignore_Costs





class	Planner : public STRIPS_Problem
{
public:

	Planner( );
	Planner( std::string, std::string );
	virtual ~Planner();

	/**
	 * Setup the data structures to compute heuristics
	 * once the problem is loaded
	*/
	virtual void setup();

	/**
	 *  Heuristics API
	 * */

	// Eval using the initial state of the problem
	float eval_landmarks_init();
	float eval_hmax_init();

	// Eval using the state in the argument
	float	eval_hmax( boost::python::list& fluent_list );
	float eval_hadd( boost::python::list& fluent_list );
	float eval_hff( boost::python::list& fluent_list );
	float eval_h2( boost::python::list& fluent_list );
	float eval_hmin( boost::python::list& fluent_list );
	float eval_hgc( boost::python::list& fluent_list );
	std::string generate_best_action();
	int optimal_cost();
	int expanded_node();
	int generated_node();
	std::string all_info();

	/**
	 * Members
	*/

	std::string	m_log_filename;
	std::string	m_plan_filename;


protected:

	// Create Forward state space
	Fwd_Search_Problem*	m_search_prob;

	// Initial State
	State* m_init_state;

	// Create Landmarkc count Heuristic
	H_Lmcount_Fwd* m_lm_count;

	// Landmark Graph Generator
	Gen_Lms_Fwd*    m_gen_lms;

	// Create Hmax heuristic
	H_Max_Fwd*    m_hmax;
	H_Add_Fwd*    m_hadd;
	H_FF_Fwd* 		m_hff;
	H2_Fwd* 			m_h2;
	H_Min_Fwd*    m_hmin;
	H_GC_Fwd* 		m_hgc;
	BFS_H_Lmcut_Fwd* engine;
	BFS_H_GC_Fwd* engine_gc;


};

#endif
