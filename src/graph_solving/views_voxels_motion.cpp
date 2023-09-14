#include <vector> 
#include <iostream>
#include <unordered_map>

#include <octomap/octomap.h> // Assuming Octomap headers are included properly.

#include <gurobi_c++.h> // Assuming Gurobi headers are included properly.

//views_voxels_motion class hpp header
/**
 * @class sublooplim
 * @brief A class derived from GRBCallback for solving a specific optimization problem.
 *
 * This class defines a GRBCallback for finding optimal paths in a given graph.
 * It checks for subloops in the paths found and adds constraints to eliminate them.
 */
class sublooplim : public GRBCallback{
public:
	GRBVar** path_selection;  ///< 2D array of decision variables representing path selections.
	int num_of_view;          ///< Total number of views.
	int start_view_id;        ///< ID of the starting view.
	bool print_log; 		  ///< Flag for printing log.

	/**
	 * @brief Constructor for sublooplim class.
	 * @param _path_selection 2D array of decision variables representing path selections.
	 * @param _num_of_view Total number of views.
	 * @param _start_view_id ID of the starting view.
	 * @param _print_log Flag for printing log.
	 */
	sublooplim(GRBVar** _path_selection, int _num_of_view, int _start_view_id, bool _print_log = false) {
		path_selection = _path_selection;
		num_of_view = _num_of_view;
		start_view_id = _start_view_id;
		print_log = _print_log;
	}

	/**
	 * @brief Find a path starting from the specified view, considering given constraints.
	 * @param _cur_view ID of the current view.
	 * @param visited Vector indicating if a view has been visited or not.
	 * @param x 2D array of solution values.
	 * @return Vector of view IDs representing the path.
	 */
	std::vector<int> find_path(int _cur_view, std::vector<bool>& visited, double** x) {
		std::vector<int> path;
		int cur_view = _cur_view;
		while (true) {
			path.push_back(cur_view);
			visited[cur_view] = true;
			int next_view_id;
			for (next_view_id = 0; next_view_id < num_of_view; next_view_id++) {
				if(visited[next_view_id]) continue;
				if (x[cur_view][next_view_id] > 0.5) {
					break;
				}
			}
			if (next_view_id == num_of_view) {
				break;
			}
			cur_view = next_view_id;
		}
		return path;
	}

protected:
	/**
	 * @brief Overridden callback function that is executed during optimization.
	 */
	void callback() {
		try {
			if (where == GRB_CB_MIPSOL) {
				// Found an integer feasible solution - does it have one way path without single loop?
				double** x = new double* [num_of_view];
				for (int i = 0; i < num_of_view; i++)
					x[i] = getSolution(path_selection[i], num_of_view);
				std::vector<std::vector<int>> paths;
				std::vector<bool> visited(num_of_view, false);

				// Find the path starting from start_view_id
				paths.push_back(find_path(start_view_id, visited, x));

				// Find the path starting from other view
				for (int i = 0; i < num_of_view; i++) {
					if (visited[i]) continue;
					bool selected = false;
					for (int j = 0; j< num_of_view; j++)
						if (x[i][j] > 0.5) {
							selected = true;
							break;
						}
					if (selected) paths.push_back(find_path(i, visited, x));
				}

				if (print_log) {
					std::cout << "Found " << paths.size() << " paths." << std::endl;
					for (int i = 0; i < paths.size(); i++) {
						std::cout << "Path " << i << ": ";
						for (int j = 0; j < paths[i].size() - 1; j++) {
							std::cout << paths[i][j] << "->";
						}
						std::cout << paths[i][paths[i].size() - 1] << std::endl;
					}
				}
				
				// Check for subloops and add constraints to eliminate them
				if (paths.size() > 1) {
					// Add subloop elimination constraint
					for (int i = 1; i < paths.size(); i++) {
						GRBLinExpr subloop = 0;
						for (int j = 0; j < paths[i].size() - 1; j++){
							subloop += path_selection[paths[i][j]][paths[i][j + 1]];
						}
						subloop += path_selection[paths[i][paths[i].size() - 1]][paths[i][0]];
						addLazy(subloop <= paths[i].size() - 1);
					}
				}

				for (int i = 0; i < num_of_view; i++)
					delete[] x[i];
				delete[] x;
				paths.clear();
				paths.shrink_to_fit();
				visited.clear();
				visited.shrink_to_fit();
			}
		}
		catch (GRBException e) {
			std::cout << "Error number: " << e.getErrorCode() << std::endl;
			std::cout << e.getMessage() << std::endl;
		}
		catch (...) {
			std::cout << "Error during callback" << std::endl;
		}
	}
};

/**
 * @class views_voxels_motion
 * @brief A class for solving an optimization problem related to motion planning with view selection and voxel coverage.
 *
 * This class defines an optimization problem where motion planning is performed considering view selection and voxel coverage.
 */
class views_voxels_motion {
public:
	int start_view_id;                      ///< ID of the starting view.
	int virtual_end_view_id;                ///< ID of the virtual end view.
	int num_of_view;                        ///< Total number of views.
	int num_of_voxel;                       ///< Total number of voxels.
	std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> voxel_id_map;  ///< Mapping of voxel keys to IDs.
	std::vector<std::vector<bool>> connection_graph;  ///< Connection graph indicating voxel coverage by views.
	int time_limit; 					    ///< Time limit for optimization in seconds.
	bool print_log;                         ///< Flag for printing log messages.

	// Optimizer components
	GRBEnv* env;                            ///< Gurobi environment.
	GRBModel* model;                        ///< Gurobi optimization model.
	GRBVar* view_selection;                 ///< Decision variables for view selection.
	GRBVar** path_selection;                ///< 2D array of decision variables for path selection.
	GRBLinExpr obj;                         ///< Linear expression for the objective function.
	sublooplim* cb;                         ///< Callback object for subloop elimination.

	// Output
	std::vector<int> selected_view_ids;           ///< List of selected view IDs.
	std::vector<int> optimal_paths;               ///< Optimal path of view IDs.

	/**
	 * @brief Constructor for the views_voxels_motion class.
	 * @param views_motion Motion cost between views.
	 * @param views_voxels Voxel coverage information for each view.
	 * @param _start_view_id ID of the starting view.
	 * @param _print_log Flag for printing log messages.
	 * @param time_limit Time limit for optimization in seconds.
	 */
	views_voxels_motion(std::vector<std::vector<double>> views_motion, std::vector<std::vector<octomap::OcTreeKey>>& views_voxels, int _start_view_id, int _time_limit = -1, bool _print_log = false) {
		start_view_id = _start_view_id;
		print_log = _print_log;
		time_limit = _time_limit;
		virtual_end_view_id = views_motion.size();
		//check input view number
		if (views_motion.size() != views_voxels.size()) {
			std::cout << "Error: num_of_view: views_motion.size() != views_voxels.size()" << std::endl;
			return;
		}

		//add a virtual end view
		for (int i = 0; i < views_motion.size(); i++)
			views_motion[i].push_back(0);
		std::vector<double> virtual_end_view_motion;
		for (int i = 0; i < views_motion.size() + 1; i++)
			virtual_end_view_motion.push_back(0);
		views_motion.push_back(virtual_end_view_motion);
		num_of_view = views_motion.size();

		if (print_log) {
			//Print Motion Costs Between Viewpoints
			std::cout << "Motion cost: " << std::endl;
			for (int i = 0; i < num_of_view; i++)
				std::cout << "\t" << i;
			std::cout << std::endl;
			for (int i = 0; i < num_of_view; i++) {
				std::cout << i << "\t";
				for (int j = 0; j < num_of_view; j++)
					std::cout << views_motion[i][j] << "\t";
				std::cout << std::endl;
			}
		}

		//create an id map for all view voxels
		num_of_voxel = 0;
		voxel_id_map.clear();
		for (int i = 0; i < views_voxels.size(); i++) {
			for (int j = 0; j < views_voxels[i].size(); j++) {
				if (voxel_id_map.find(views_voxels[i][j]) == voxel_id_map.end()) {
					voxel_id_map[views_voxels[i][j]] = num_of_voxel++;
				}
			}
		}
		if (print_log) std::cout << "num_of_voxel: " << num_of_voxel << std::endl;

		//create a connection graph for all view voxels
		connection_graph.resize(num_of_view);
		for (int i = 0; i < num_of_view; i++) {
			connection_graph[i].resize(num_of_voxel);
			for (int j = 0; j < num_of_voxel; j++) {
				connection_graph[i][j] = 0;
			}
		}
		for (int i = 0; i < views_voxels.size(); i++) {
			for (int j = 0; j < views_voxels[i].size(); j++) {
				connection_graph[i][voxel_id_map[views_voxels[i][j]]] = 1;
			}
		}

		if (print_log) {
			//print connection graph
			std::cout << "connection_graph:" << std::endl;
			std::cout << '\t';
			for (int i = 0; i < num_of_voxel; i++) {
				std::cout << i << '\t';
			}
			std::cout << std::endl;
			for (int i = 0; i < num_of_view; i++) {
				std::cout << i << '\t';
				for (int j = 0; j < num_of_voxel; j++) {
					std::cout << connection_graph[i][j] << '\t';
				}
				std::cout << std::endl;
			}
		}

		//setup linear programming solver
		env = new GRBEnv();
		model = new GRBModel(*env);
		// Must set LazyConstraints parameter when using lazy constraints
		model->set(GRB_IntParam_LazyConstraints, 1);

		// Create variables
		view_selection = new GRBVar[num_of_view];
		path_selection = new GRBVar * [num_of_view];
		for (int i = 0; i < num_of_view; i++) {
			view_selection[i] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "view_selection" + std::to_string(i));
			path_selection[i] = new GRBVar[num_of_view];
			for (int j = 0; j < num_of_view; j++) {
				path_selection[i][j] = model->addVar(0.0, 1.0, 0.0, GRB_BINARY, "path_selection" + std::to_string(i) + std::to_string(j));
			}
		}
		if (print_log) std::cout << "variables created." << std::endl;

		// Set objective : smallest motion cast
		for (int i = 0; i < num_of_view; i++) {
			for (int j = 0; j < num_of_view; j++) {
				obj += views_motion[i][j] * path_selection[i][j];
			}
		}
		model->setObjective(obj, GRB_MINIMIZE);
		if (print_log) std::cout << "objective set." << std::endl;

		// View and path realtion
		for (int i = 0; i < num_of_view; i++) {
			GRBLinExpr subject_of_view_in;
			GRBLinExpr subject_of_view_out;
			for (int j = 0; j < num_of_view; j++) {
				subject_of_view_in += path_selection[j][i];
				subject_of_view_out += path_selection[i][j];
			}
			model->addConstr(subject_of_view_in <= view_selection[i], "view_path_relation_00inremove" + std::to_string(i));
			model->addConstr(subject_of_view_out <= view_selection[i], "view_path_relation_00outremove" + std::to_string(i));
			model->addConstr(subject_of_view_in + subject_of_view_out >= view_selection[i], "view_path_relation_001remove" + std::to_string(i));

		}
		if (print_log) std::cout << "view and path relation set." << std::endl;

		// Forbid edge from node back to itself
		for (int i = 0; i < num_of_view; i++)
			path_selection[i][i].set(GRB_DoubleAttr_UB, 0);

		// Forbid start to virtual end
		path_selection[start_view_id][virtual_end_view_id].set(GRB_DoubleAttr_UB, 0);

		// Select start and virtual end
		view_selection[start_view_id].set(GRB_DoubleAttr_LB, 1);
		view_selection[virtual_end_view_id].set(GRB_DoubleAttr_LB, 1);

		// Degree-1 constraints
		for (int i = 0; i < num_of_view; i++) {
			GRBLinExpr subject_of_indgree;
			GRBLinExpr subject_of_outdgree;
			for (int j = 0; j < num_of_view; j++) {
				subject_of_indgree += path_selection[j][i];
				subject_of_outdgree += path_selection[i][j];
			}
			if (i == start_view_id) {
				model->addConstr(subject_of_indgree == 0, "indgree" + std::to_string(i));
				model->addConstr(subject_of_outdgree == 1, "outdgree" + std::to_string(i));
			}
			else if (i == virtual_end_view_id) {
				model->addConstr(subject_of_indgree == 1, "indgree" + std::to_string(i));
				model->addConstr(subject_of_outdgree == 0, "outdgree" + std::to_string(i));
			}
			else {
				model->addConstr(subject_of_indgree <= 1, "indgree" + std::to_string(i));
				model->addConstr(subject_of_outdgree <= 1, "outdgree" + std::to_string(i));
				model->addConstr(subject_of_indgree == subject_of_outdgree, "inoutdgree" + std::to_string(i));
			}
		}
		if (print_log) std::cout << "Degree-1 constraints set." << std::endl;

		// Full coverage constraints
		for (int j = 0; j < num_of_voxel; j++) {
			GRBLinExpr subject_of_voxel;
			for (int i = 0; i < num_of_view; i++)
				if (connection_graph[i][j] == 1) subject_of_voxel += view_selection[i];
			model->addConstr(subject_of_voxel >= 1, "coverage" + std::to_string(j));
		}
		if (print_log) std::cout << "Full coverage constraints set." << std::endl;

		// Set callback function
		cb = new sublooplim(path_selection, num_of_view, start_view_id, print_log);
		model->setCallback(cb);

		// Set time limit
		if (time_limit > 0) {
			model->set("TimeLimit", std::to_string(time_limit));
		}

		std::cout<< "Build optimazation model done." << std::endl;
	}

	/**
	 * @brief Destructor for the views_voxels_motion class.
	 */
	~views_voxels_motion() {
		delete env;
		delete model;
		delete[] view_selection;
		for (int i = 0; i < num_of_view; i++)
			delete[] path_selection[i];
		delete[] path_selection;
		delete cb;
		connection_graph.clear();
		connection_graph.shrink_to_fit();
		selected_view_ids.clear();
		selected_view_ids.shrink_to_fit();
		optimal_paths.clear();
		optimal_paths.shrink_to_fit();
	}

	/**
	 * @brief Solve the optimization problem.
	 */
	void solve() {
		double now_time = clock();
		// Optimize model
		model->optimize();
		if (print_log) {
			// show selected views
			std::cout << "Selected views: ";
			for (int i = 0; i < num_of_view; i++)
				if (view_selection[i].get(GRB_DoubleAttr_X) == 1.0) {
					std::cout << i << " ";
					if (i != virtual_end_view_id) selected_view_ids.push_back(i);
				}
			std::cout << std::endl;
			//show all connected path
			std::cout << "Selected paths: " << std::endl;
			for (int i = 0; i < num_of_view; i++) {
				for (int j = 0; j < num_of_view; j++) {
					if (path_selection[i][j].get(GRB_DoubleAttr_X) == 1.0) {
						std::cout << i << " " << j << std::endl;
					}
				}
			}
		}
		// show motion cost
		std::cout << "Obj: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
		// show path from start view
		std::cout << "Path: ";
		int cur_view = start_view_id;
		while (true) {
			std::cout << cur_view << " ";
			optimal_paths.push_back(cur_view);
			int next_view_id;
			for (next_view_id = 0; next_view_id < num_of_view; next_view_id++)
				if (path_selection[cur_view][next_view_id].get(GRB_DoubleAttr_X) == 1.0) {
					break;
				}
			if (next_view_id == num_of_view) {
				std::cout << "(Virtual End)";
				optimal_paths.pop_back();
				break;
			}
			cur_view = next_view_id;
		}
		std::cout << std::endl;
		std::cout << "Solve Time: " << (clock() - now_time) / CLOCKS_PER_SEC << "s." << std::endl;
	}

	/**
	 * @brief Get the set of selected view IDs.
	 * @return Vector containing selected view IDs.
	 */
	std::vector<int> get_views_id_set() {
		return selected_view_ids;
	}
	
	/**
	 * @brief Get the optimal path of view IDs.
	 * @return Vector containing optimal path of view IDs.
	 */
	std::vector<int> get_optimal_path() {
		return optimal_paths;
	}
};

int main()
{
	/**
	 * good test case:
	 * seed 1, view 4, voxel 2, start 1; <1s
	 * seed 1, view 10, voxel 10, start 0; <1s
	 * seed 1, view 10, voxel 100, start 4; <1s
	 * seed 1, view 100, voxel 1000, start 2; <20s
	 * seed 1, view 200, voxel 500, start 3; <50s
	 */

	// Seed the random number generator.
	srand(1); 

	// Set up test parameters.
	int num_of_view = 10;
	int num_of_voxel = 10;
	int start_view_id = 0;
	int time_limit = -1;		// in seconds, -1 for no limit
	bool print_log = false;		// whether to print log

	//Randomly Generating Motion Costs Between Viewpoints
	std::vector<std::vector<double>> views_motion;
	views_motion.resize(num_of_view);
	for (int i = 0; i < num_of_view; i++) {
		views_motion[i].resize(num_of_view);
		for (int j = 0; j < i; j++) {
			views_motion[i][j] = rand() % 100 + 1;
			views_motion[j][i] = views_motion[i][j];
		}
		views_motion[i][i] = 0;
	}

	if (print_log) {
		//Print Motion Costs Between Viewpoints
		std::cout << "Motion cost: " << std::endl;
		for (int i = 0; i < num_of_view; i++)
			std::cout << "\t" << i;
		std::cout << std::endl;
		for (int i = 0; i < num_of_view; i++) {
			std::cout << i << "\t";
			for (int j = 0; j < num_of_view; j++)
				std::cout << views_motion[i][j] << "\t";
			std::cout << std::endl;
		}
	}
	
	
	//Randomly Generate Voxels for Viewpoints
	std::vector<octomap::OcTreeKey> voxels;
	voxels.resize(num_of_voxel);
	for (int i = 0; i < num_of_voxel; i++) {
		voxels[i].k[0] = rand() % 30000;
		voxels[i].k[1] = rand() % 30000;
		voxels[i].k[2] = rand() % 30000;
	}

	//Randomly Allocate Voxels for Viewpoints
	std::vector<std::vector<octomap::OcTreeKey>> views_voxels;
	views_voxels.resize(num_of_view);
	for (int i = 0; i < num_of_view; i++) {
		int sub_num_of_voxel = rand() % num_of_voxel + 1;
		views_voxels[i].resize(sub_num_of_voxel);
		for (int j = 0; j < sub_num_of_voxel; j++)
			views_voxels[i][j] = voxels[rand() % num_of_voxel];
		if(i == start_view_id) views_voxels[i].clear();
	}

	if (print_log) {
		//Print Voxels for Viewpoints
		std::cout << "Views voxels: " << std::endl;
		for (int i = 0; i < num_of_view; i++) {
			std::cout << i << "\t" << "(";
			for (int j = 0; j < views_voxels[i].size(); j++)
				std::cout << views_voxels[i][j].k[0] << "," << views_voxels[i][j].k[1] << "," << views_voxels[i][j].k[2] << ";";
			std::cout << ")" << std::endl;
		}
	}
	
	//Calculate Optimal Coverage Path
	views_voxels_motion* vvm = new views_voxels_motion(views_motion, views_voxels, start_view_id, time_limit, print_log);
	vvm->solve();
	std::vector<int> selected_view_ids = vvm->get_views_id_set();
	std::vector<int> optimal_paths = vvm->get_optimal_path();

	// Print the selected views and optimal path.
	std::cout << "Selected views: ";
	for (int i = 0; i < selected_view_ids.size(); i++)
		std::cout << selected_view_ids[i] << " ";
	std::cout << std::endl;

	std::cout << "Optimal path: ";
	for (int i = 0; i < optimal_paths.size(); i++)
		std::cout << optimal_paths[i] << " ";
	std::cout << std::endl;

	// Clean up.
	delete vvm;

	return 0;
}