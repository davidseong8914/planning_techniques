/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/

/*
Run command:
g++ -std=c++17 planner.cpp -o planner.out
./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 myOutput.txt
python visualizer.py myOutput.txt --gifFilepath=myGif.gif

Run for testing:
./planner.out map2.txt 5 0.54,2.30,2.36,4.12,4.34 1.12,0.02,3.37,2.52,4.91 0 myOutput.txt
g++ -std=c++17 verifier.cpp -o verifier.out
python grader.py
*/

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

// ones I added
#include <random>
#include <set>
#include <queue>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

/////////        /////////
/// FUNCTIONS I ADDED  ///
/////////        /////////


// Node: joint combinations, parent index, cost
struct Node {
	double* joint_comb;
	int parent;
	double cost;
};

// calculates euclidean distance between 2 joint combinations
double euclidean_distance (double* angles1, double* angles2, int numofDOFs) {
	double dist = 0.0;
	for (int i = 0; i < numofDOFs; i++) {
		double diff = angles1[i] - angles2[i];
		dist += diff * diff;
	}
	return sqrt(dist);
}

// extend status
enum ExtendStatus {
    TRAPPED = 0,
    ADVANCED = 1,
    REACHED = 2
};

// check configuration within bounds
bool isConfigWithinBounds(double* angles, int numofDOFs, int x_size, int y_size) {
    double x = ((double)x_size)/2.0;
    double y = 0;
    
    for(int i = 0; i < numofDOFs; i++){
        double x0 = x;
        double y0 = y;
        x = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
        y = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);
        
        if(x < 0 || x >= x_size || y < 0 || y >= y_size) {
            return false;
        }
    }
    return true;
}

// interpolates points between 2 joint configs and checks if the "line" between them are valid
bool isValidStraightLinePath(double* config1, double* config2, int numofDOFs, double* map, int x_size, int y_size, double step_size = 0.05) {
    double dist = euclidean_distance(config1, config2, numofDOFs);
    int num_steps = std::max(3, (int)(dist / step_size));
    for (int i = 1; i < num_steps; i++) {
        double ratio = (double)i / num_steps;
        double* intermediate = new double[numofDOFs];
        for (int j = 0; j < numofDOFs; j++) {
            intermediate[j] = config1[j] + ratio * (config2[j] - config1[j]);
        }
        bool valid = IsValidArmConfiguration(intermediate, numofDOFs, map, x_size, y_size);
        delete[] intermediate;
        if (!valid) {
            return false;
        }
    }
    
    return true;
}

// finds index of nearest neighbor of target node from nodes
int nearest_neighbor(Node q_target, const std::vector<Node>& nodes, int numofDOFs) {
    int near_ind = 0;
    double min_dist = euclidean_distance(nodes[0].joint_comb, q_target.joint_comb, numofDOFs);

    for (int i = 1; i < nodes.size(); i++) {
        double dist = euclidean_distance(nodes[i].joint_comb, q_target.joint_comb, numofDOFs);
        if (dist < min_dist) {
            min_dist = dist;
            near_ind = i;
        }
    }

    return near_ind;
}

// extend closest node from q_rand to q_rand
ExtendStatus extend(std::vector<Node>& nodes, Node q_rand, double stepsize, int numofDOFs, double* map, int x_size, int y_size) {
	int neighbor_node_idx = nearest_neighbor(q_rand, nodes, numofDOFs);

	// q_new : step size from q_near to q_rand
	Node q_new;
	q_new.joint_comb = new double[numofDOFs];
	q_new.parent = neighbor_node_idx;

	// calculate vector to q_rand
	double magnitude = 0;
	std::vector<double> direction_vector(numofDOFs);

	for (int i = 0; i < numofDOFs; i++) {
		double diff = q_rand.joint_comb[i] - nodes[neighbor_node_idx].joint_comb[i];
		magnitude += diff * diff;
		direction_vector[i] = diff;
	}

	magnitude = sqrt(magnitude);
	bool reached = magnitude <= stepsize;

    for (int i = 0; i < numofDOFs; i++) {
        if (reached) {
            q_new.joint_comb[i] = q_rand.joint_comb[i];
        } else {
            q_new.joint_comb[i] = nodes[neighbor_node_idx].joint_comb[i] + 
                                 (direction_vector[i] / magnitude) * stepsize;
        }
    }
		q_new.cost = nodes[neighbor_node_idx].cost + (reached ? magnitude : stepsize);

	// if outside of map
    if (!isConfigWithinBounds(q_new.joint_comb, numofDOFs, x_size, y_size)) {
        delete[] q_new.joint_comb;
        return TRAPPED;
    }
    //check both the configuration, transition path
    if (IsValidArmConfiguration(q_new.joint_comb, numofDOFs, map, x_size, y_size) && 
        isValidStraightLinePath(nodes[neighbor_node_idx].joint_comb, q_new.joint_comb, numofDOFs, map, x_size, y_size)) {
        nodes.push_back(q_new);
        if (reached) {
            return REACHED;
        }
        return ADVANCED;
    } else {
        delete[] q_new.joint_comb;
        return TRAPPED;
    }
}

// connect to last added node 
ExtendStatus connect(std::vector<Node>& nodes, const Node& q_target, double stepsize, int numofDOFs, double* map, int x_size, int y_size) {
    ExtendStatus status = ADVANCED;
    
    while (status == ADVANCED) {
        status = extend(nodes, q_target, stepsize, numofDOFs, map, x_size, y_size);
    }
    
    return status;
}

// shortcut post processing, stepsize formatting
void shortcutPath(std::vector<std::vector<double>>& path, int numofDOFs, double* map, int x_size, int y_size, double stepsize) {
    if (path.size() <= 2) return; // Nothing to shortcut
    
    // start and goal configurations
    std::vector<double> exact_start = path.front();
    std::vector<double> exact_goal = path.back();
    
    // shortcut path - starting with start
    std::vector<std::vector<double>> newPath;
    newPath.push_back(path.front());
    
    int i = 0;
    while (i < path.size() - 1) {
        int j;
        // find the furthest valid connection from current point
        for (j = path.size() - 1; j > i + 1; j--) {
            if (isValidStraightLinePath(&path[i][0], &path[j][0], numofDOFs, map, x_size, y_size, 0.05)) {
                break;
            }
        }
        newPath.push_back(path[j]);
        i = j; 
    }
        if (newPath.back() != path.back()) {
        newPath.push_back(path.back());
    }
    
    // update path
    path = newPath;
}
    

//// for RRT* ////
// return indices of neighboring nodes
std::vector<int> near_neighbors(const std::vector<Node>& nodes, int node_idx, double radius, int numofDOFs) {
    std::vector<int> neighbors;
    neighbors.reserve(20);
    
    for (int i = 0; i < nodes.size(); i++) {
        if (i == node_idx) continue; 
        
        double dist = euclidean_distance(nodes[i].joint_comb, nodes[node_idx].joint_comb, numofDOFs);
        if (dist <= radius) {
            neighbors.push_back(i);
        }
    }
    
    return neighbors;
}

// choosing parent with lowest cost
int choose_parent(std::vector<Node>& nodes, int new_node_idx, const std::vector<int>& near_indices, 
                 double* map, int x_size, int y_size, int numofDOFs) {
    int min_cost_idx = nodes[new_node_idx].parent;
    double min_cost = nodes[new_node_idx].cost;
    
    for (int near_idx : near_indices) {
        double edge_cost = euclidean_distance(nodes[near_idx].joint_comb, nodes[new_node_idx].joint_comb, numofDOFs);
        double potential_cost = nodes[near_idx].cost + edge_cost;
        
        if (potential_cost < min_cost && 
            isValidStraightLinePath(nodes[near_idx].joint_comb, nodes[new_node_idx].joint_comb, numofDOFs, map, x_size, y_size)) {
            min_cost = potential_cost;
            min_cost_idx = near_idx;
        }
    }
    
    // update parent and cost
    nodes[new_node_idx].parent = min_cost_idx;
    nodes[new_node_idx].cost = min_cost;
    
    return min_cost_idx;
}

// rewiring based on cost
void rewire(std::vector<Node>& nodes, int new_node_idx, const std::vector<int>& near_indices, 
           double* map, int x_size, int y_size, int numofDOFs) {
    for (int near_idx : near_indices) {
        // skip parent
        if (near_idx == nodes[new_node_idx].parent) continue;
        
        if (isValidStraightLinePath(nodes[new_node_idx].joint_comb, nodes[near_idx].joint_comb, numofDOFs, map, x_size, y_size)) {
            double edge_cost = euclidean_distance(nodes[new_node_idx].joint_comb, nodes[near_idx].joint_comb, numofDOFs);
            double potential_cost = nodes[new_node_idx].cost + edge_cost;
            
            // rewire
            if (potential_cost < nodes[near_idx].cost) {
                // add new edge
                nodes[near_idx].parent = new_node_idx;
                nodes[near_idx].cost = potential_cost;
            }
        }
    }
}



// allocating memory
void allocatePlanMemory(double*** plan, int* planlength, 
                        const std::vector<std::vector<double>>& path, int numofDOFs) {
    *planlength = path.size();
    *plan = (double**) malloc((*planlength) * sizeof(double*));
    
    for (int i = 0; i < *planlength; i++) {
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[i][j] = path[i][j];
        }
    }
}

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength,
			int whichPlanner)
{

	// store nodes RRT
	std::vector<Node> nodes;

	// store nodes RRT-connect
	std::vector<Node> nodes_init;
	std::vector<Node> nodes_goal;

	// number of samples
	int K = 50000;
	// how often to generate biased node
	int bias_check = 20;
	// angle change step size
	double epsilon = PI/5;
	// step size for visualization
	double stepsize = PI/10;
	// shortcut
	bool shortcut = true;
	// can also adjust {stepsize} for valid straight line path

	// initial  node
	Node q_init;
	// memory for joint angles
	q_init.joint_comb = new double[numofDOFs];
	// set initial configuration
	for (int i = 0; i < numofDOFs; i++) {
		q_init.joint_comb[i] = armstart_anglesV_rad[i];
	}
	// set index
	q_init.parent = -1;

	// final node
	Node q_goal;
	q_goal.joint_comb = new double[numofDOFs];
	for (int i = 0; i < numofDOFs; i++) {
		q_goal.joint_comb[i] = armgoal_anglesV_rad[i];
	}

	// generate random angle between 0 - 2pi
	std::random_device rd;  
	std::mt19937 gen(rd()); 
	std::uniform_real_distribution<double> distribution(-PI, PI);

	// path cost track
	double pathCost = 0.0;
	// generated vertex track
	int numVertices = 0;
	// time track
	auto startTime = std::chrono::high_resolution_clock::now();

	// RRT : planner id = 0
	if (whichPlanner == 0) {
		nodes.push_back(q_init);
		bool goal_reached = false;

		// random sampling 
		for (int k = 0; k < K; k++) {
			Node q_rand;
			q_rand.joint_comb = new double [numofDOFs];

			// biased towards goal every n times
			if (k % bias_check == 0) {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = q_goal.joint_comb[i];
				}
			}
			else {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = distribution(gen);
				}
			}

			// extend towards q_rand, returns true if can connect 
			extend(nodes, q_rand, epsilon, numofDOFs, map, x_size, y_size);
			delete[] q_rand.joint_comb;

			// check if can connect to goal
			if (k % (bias_check * 2) == 0) {
				if (extend(nodes, q_goal, epsilon, numofDOFs, map, x_size, y_size) == REACHED) {
					// stops sampling if goal can be reached
					goal_reached = true;
					break;
				}
			}
		}

		// backtracking to find path
		std::vector<int> RRT_path;

		// goal reached
		if (goal_reached) {
			int goal_idx = nodes.size() - 1; 
			int current = goal_idx;
			while (current != -1) {
				RRT_path.push_back(current);
				current = nodes[current].parent;
			}
			std::cout << "Goal reached with planner 0: RRT " << std::endl;
		} else {
		// goal not reached -> go to nearest 
			std::cout << "Goal not reached with planner 0: RRT " << std::endl;
			int nearest = nearest_neighbor(q_goal, nodes, numofDOFs);

			q_goal.parent = nearest;

			if (IsValidArmConfiguration(q_goal.joint_comb, numofDOFs, map, x_size, y_size)) {
				nodes.push_back(q_goal);
				nearest = nodes.size() - 1;
			} 

			int current = nearest;
			while (current != -1) {
				RRT_path.push_back(current);
				current = nodes[current].parent;
			}
		}

		std::reverse(RRT_path.begin(), RRT_path.end());
		
		std::vector<std::vector<double>> path;
		for (int idx : RRT_path) {
			std::vector<double> config(numofDOFs);
			for (int j = 0; j < numofDOFs; j++) {
				config[j] = nodes[idx].joint_comb[j];
			}
			path.push_back(config);
		}
		
		numVertices = nodes.size();

		std::cout << "Path before shortcut: " << path.size() << " points" << std::endl;

		if (shortcut) {
			shortcutPath(path, numofDOFs, map, x_size, y_size, stepsize);
			std::cout << "Path after shortcut: " << path.size() << " points" << std::endl;
		}
		
		allocatePlanMemory(plan, planlength, path, numofDOFs);
		std::cout << "Path length: " << path.size() << std::endl;

		for (int i = 0; i < numofDOFs; ++i) {
			(*plan)[*planlength - 1][i] = armgoal_anglesV_rad[i];
		}
		
		// cleanup
		for (size_t i = 0; i < nodes.size(); i++) {
			delete[] nodes[i].joint_comb;
		}
	}
	
	// RRT-Connect : planner id = 1
	else if (whichPlanner == 1) {
		// Initialize the two trees
		q_init.cost = 0.0;
		q_init.parent = -1;
		nodes_init.push_back(q_init); // Tree a (from start)
		
		q_goal.cost = 0.0;
		q_goal.parent = -1;
		nodes_goal.push_back(q_goal); // Tree b (from goal)
		
		int connect_init_idx = -1;
		int connect_goal_idx = -1;
		bool connection = false;
		
		// direct connection
		if (isValidStraightLinePath(q_init.joint_comb, q_goal.joint_comb, numofDOFs, map, x_size, y_size)) {
			connection = true;
			connect_init_idx = 0;
			connect_goal_idx = 0;
		} else {
			for (int k = 0; k < K && !connection; k++) {
				//define trees & swapping
				std::vector<Node>& tree_a = (k % 2 == 0) ? nodes_init : nodes_goal;
				std::vector<Node>& tree_b = (k % 2 == 0) ? nodes_goal : nodes_init;
				
				Node q_rand;
				q_rand.joint_comb = new double[numofDOFs];
				
				if (k % bias_check == 0) {
					// bias... start as well?
					for (int i = 0; i < numofDOFs; i++) {
						q_rand.joint_comb[i] = (k % 2 == 0) ? q_goal.joint_comb[i] : q_init.joint_comb[i];
					}
				} else {
					for (int i = 0; i < numofDOFs; i++) {
						q_rand.joint_comb[i] = distribution(gen);
					}
				}
				
				int tree_a_size_before = tree_a.size();
            if (extend(tree_a, q_rand, epsilon, numofDOFs, map, x_size, y_size) != TRAPPED) {
                Node& q_new = tree_a.back();
                
                // Eearly termination if connected
                if (connect(tree_b, q_new, epsilon, numofDOFs, map, x_size, y_size) == REACHED) {
                    connection = true;
                    connect_init_idx = (k % 2 == 0) ? tree_a.size() - 1 : tree_b.size() - 1;
                    connect_goal_idx = (k % 2 == 0) ? tree_b.size() - 1 : tree_a.size() - 1;
                    delete[] q_rand.joint_comb;
                    break;
                }
            }
            
            delete[] q_rand.joint_comb;
        }
		}				
		
		
		if (connection) {
			std::cout << "Goal reached with planner 1: RRT-Connect" << std::endl;
			
			std::vector<std::vector<double>> path;
			
			std::vector<double> exact_start(numofDOFs);
			for (int i = 0; i < numofDOFs; i++) {
				exact_start[i] = armstart_anglesV_rad[i];
			}
			path.push_back(exact_start);
			
			if (connect_init_idx != 0 || connect_goal_idx != 0) {
				// nodes from init tree (except start node)
				std::vector<int> init_path;
				int current = connect_init_idx;
				
				// connection to start
				while (current != 0) {
					init_path.push_back(current);
					current = nodes_init[current].parent;
				}
				
				for (int i = init_path.size() - 1; i >= 0; i--) {
					std::vector<double> config(numofDOFs);
					for (int j = 0; j < numofDOFs; j++) {
						config[j] = nodes_init[init_path[i]].joint_comb[j];
					}
					path.push_back(config);
				}
				
				
				std::vector<int> goal_path;
				current = connect_goal_idx;
				
				// conection to goal
				while (current != 0) {
					goal_path.push_back(current);
					current = nodes_goal[current].parent;
				}
				
				for (int i = goal_path.size() - 1; i >= 0; i--) {
					std::vector<double> config(numofDOFs);
					for (int j = 0; j < numofDOFs; j++) {
						config[j] = nodes_goal[goal_path[i]].joint_comb[j];
					}
					path.push_back(config);
				}
			}
			
			std::vector<double> exact_goal(numofDOFs);
			for (int i = 0; i < numofDOFs; i++) {
				exact_goal[i] = armgoal_anglesV_rad[i];
			}
			path.push_back(exact_goal);
			
			numVertices = nodes_init.size() + nodes_goal.size();
			
			std::cout << "Path before shortcut: " << path.size() << " points" << std::endl;

			if (shortcut) {
				shortcutPath(path, numofDOFs, map, x_size, y_size, stepsize);
				std::cout << "Path after shortcut: " << path.size() << " points" << std::endl;
			}
			
			allocatePlanMemory(plan, planlength, path, numofDOFs);
		} else {
			std::cout << "Goal not reached with planner 1: RRT-Connect" << std::endl;			
			
			// segfault prevention
			std::vector<std::vector<double>> path;
			std::vector<double> start_config(numofDOFs);
			for (int i = 0; i < numofDOFs; i++) {
				start_config[i] = armstart_anglesV_rad[i];
			}
			path.push_back(start_config);
			
			int closest_idx = 0;
			double min_dist = INFINITY;
			for (int i = 0; i < nodes_init.size(); i++) {
				double dist = euclidean_distance(nodes_init[i].joint_comb, armgoal_anglesV_rad, numofDOFs);
				if (dist < min_dist) {
					min_dist = dist;
					closest_idx = i;
				}
			}
			
			std::vector<int> path_indices;
			int current = closest_idx;
			while (current != -1) {
				path_indices.push_back(current);
				current = nodes_init[current].parent;
			}
			
			for (int i = path_indices.size() - 1; i >= 0; i--) {
				std::vector<double> config(numofDOFs);
				for (int j = 0; j < numofDOFs; j++) {
					config[j] = nodes_init[path_indices[i]].joint_comb[j];
				}
				path.push_back(config);
			}
			
			if (path.size() < 2) {
				path.push_back(start_config);
			}
			
			allocatePlanMemory(plan, planlength, path, numofDOFs);
		}

		for (int i = 0; i < numofDOFs; ++i) {
			(*plan)[*planlength - 1][i] = armgoal_anglesV_rad[i];
		}
		
		// cleanup
		for (size_t i = 0; i < nodes_init.size(); i++) {
			delete[] nodes_init[i].joint_comb;
		}
		for (size_t i = 0; i < nodes_goal.size(); i++) {
			delete[] nodes_goal[i].joint_comb;
		}
	}

	// RRT* : planner id = 2
	else if (whichPlanner == 2) {
		K = K;
		q_init.cost = 0.0;
		nodes.push_back(q_init);
		bool goal_reached = false;
		int goal_node_idx = -1;
		double best_cost = INFINITY;

		double gamma = 1.0;

		for (int k = 0; k < K; k++) {

			Node q_rand;
			q_rand.joint_comb = new double[numofDOFs];

			if (k % bias_check == 0 && !goal_reached) {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = q_goal.joint_comb[i];
				}
			} else {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = distribution(gen);
				}
			}

			// int old_size = nodes.size();
			ExtendStatus status = extend(nodes, q_rand, epsilon, numofDOFs, map, x_size, y_size);
			delete[] q_rand.joint_comb;
			
			if (status != TRAPPED) {
				double radius = std::min(gamma * std::pow(std::log(nodes.size()) / nodes.size(), 1.0/numofDOFs), 1.0);
				std::vector<int> near_indices = near_neighbors(nodes, nodes.size()-1, radius, numofDOFs);
				
				choose_parent(nodes, nodes.size()-1, near_indices, map, x_size, y_size, numofDOFs);
				
				rewire(nodes, nodes.size()-1, near_indices, map, x_size, y_size, numofDOFs);
            
				if (!goal_reached || k % (bias_check * 5) == 0) {
					double dist_to_goal = euclidean_distance(nodes.back().joint_comb, q_goal.joint_comb, numofDOFs);
					if (dist_to_goal < epsilon * 2.0 && 
						isValidStraightLinePath(nodes.back().joint_comb, q_goal.joint_comb, numofDOFs, map, x_size, y_size)) {
						
						double potential_goal_cost = nodes.back().cost + dist_to_goal;
								
						if (!goal_reached || potential_goal_cost < best_cost) {
							if (goal_reached) {
								// update existing goal node if it exists
								nodes[goal_node_idx].parent = nodes.size() - 1;
								nodes[goal_node_idx].cost = potential_goal_cost;
							} else {
								// add new goal node
								q_goal.parent = nodes.size() - 1;
								q_goal.cost = potential_goal_cost;
								nodes.push_back(q_goal);
								goal_node_idx = nodes.size() - 1;
							}
							
							goal_reached = true;
							best_cost = potential_goal_cost;
							std::cout << "Found a path to goal with cost: " << best_cost << std::endl;

							////// FOR EXTRA CREDIT ONLY
							// break;
							//////
						}
					}
				}
			}
		}
    
		std::vector<int> RRT_path;

		if (goal_reached) {
			int current = goal_node_idx;
			while (current != -1) {
				RRT_path.push_back(current);
				current = nodes[current].parent;
			}
			std::cout << "Goal reached with planner 2: RRT* with final cost: " << best_cost << std::endl;
		} else {
			int nearest = nearest_neighbor(q_goal, nodes, numofDOFs);

			q_goal.parent = nearest;

				if (IsValidArmConfiguration(q_goal.joint_comb, numofDOFs, map, x_size, y_size)) {
					nodes.push_back(q_goal);
					nearest = nodes.size() - 1;
				} 

				int current = nearest;
				while (current != -1) {
					RRT_path.push_back(current);
					current = nodes[current].parent;
				}
			}

			std::reverse(RRT_path.begin(), RRT_path.end());
			
			std::vector<std::vector<double>> path;
			for (int idx : RRT_path) {
				std::vector<double> config(numofDOFs);
				for (int j = 0; j < numofDOFs; j++) {
					config[j] = nodes[idx].joint_comb[j];
				}
				path.push_back(config);
			}
		
			numVertices = nodes.size();

			std::cout << "Path before shortcut: " << path.size() << " points" << std::endl;
			if (shortcut) {
				shortcutPath(path, numofDOFs, map, x_size, y_size, stepsize);
				std::cout << "Path after shortcut: " << path.size() << " points" << std::endl;
			}
			
			allocatePlanMemory(plan, planlength, path, numofDOFs);
			
			for (int i = 0; i < numofDOFs; ++i) {
				(*plan)[*planlength - 1][i] = armgoal_anglesV_rad[i];
			}

			std::cout << "Path length: " << path.size() << std::endl;
			
			for (size_t i = 0; i < nodes.size(); i++) {
				if (i != goal_node_idx) {
					delete[] nodes[i].joint_comb;
				}
			}
		}
	
	
	//PRM : planner id = 3
	else if (whichPlanner == 3) {
		int n_sample = K; 
		double connect_radius = std::min(4 * epsilon, std::sqrt(numofDOFs) * std::pow(std::log(nodes.size()) / nodes.size(), 1.0/numofDOFs));
		int max_neighbor = 10; // 10 takes too long

		q_init.cost = 0.0;
		q_init.parent = -1;
		nodes.push_back(q_init);

		q_goal.cost = 0.0;
		q_goal.parent = -1;
		nodes.push_back(q_goal);

		for (int k = 0; k < n_sample; k++) {
			Node q_rand;
			q_rand.joint_comb = new double[numofDOFs];
			q_rand.cost = INFINITY;
			q_rand.parent = -1;

			for (int i = 0; i < numofDOFs; i++) {
				q_rand.joint_comb[i] = distribution(gen);
			}
			
			if (IsValidArmConfiguration(q_rand.joint_comb, numofDOFs, map, x_size, y_size)) {
				nodes.push_back(q_rand);
			} else {
				delete[] q_rand.joint_comb;
			}
		}
		
		// direct connect attempt
		bool direct_connection = isValidStraightLinePath(nodes[0].joint_comb, nodes[1].joint_comb, numofDOFs, map, x_size, y_size, stepsize);
		if (direct_connection) {
			std::cout << "Goal reached with planner 3: PRM" << std::endl;
			
			std::vector<std::vector<double>> path;
			std::vector<double> start_config(numofDOFs);
			std::vector<double> goal_config(numofDOFs);
			
			for (int i = 0; i < numofDOFs; i++) {
				start_config[i] = armstart_anglesV_rad[i];
				goal_config[i] = armgoal_anglesV_rad[i];
			}

			path.push_back(start_config);
			path.push_back(goal_config);
			
			shortcutPath(path, numofDOFs, map, x_size, y_size, stepsize);
			allocatePlanMemory(plan, planlength, path, numofDOFs);
			numVertices = 2; 
			std::cout << "Path length: " << path.size() << std::endl;

		} else {
			std::vector<std::vector<int>> neighbors(nodes.size());

			for (int i = 0; i < nodes.size(); i++) {
				std::vector<std::pair<double, int>> distances;
				// Calculate distances to all other nodes
				for (int j = 0; j < nodes.size(); j++) {
					if (i != j) {
						double dist = euclidean_distance(nodes[i].joint_comb, nodes[j].joint_comb, numofDOFs);
						distances.push_back({dist, j});
					}
				}

				std::sort(distances.begin(), distances.end());

				int connections = 0;
				for (int k = 0; k < std::min((int)distances.size(), max_neighbor); k++) {
					if (connections >= max_neighbor) break;
					
					int j = distances[k].second;
					if (isValidStraightLinePath(nodes[i].joint_comb, nodes[j].joint_comb, numofDOFs, map, x_size, y_size, stepsize)) {
						neighbors[i].push_back(j);
						connections++;
					}
				}
			}

			// A* with priority queue
			std::vector<double> dist(nodes.size(), INFINITY);
			std::vector<int> prev(nodes.size(), -1);
			std::vector<bool> visited(nodes.size(), false);

			// Priority queue of pairs: (distance, node index)
			std::priority_queue<std::pair<double, int>, 
							std::vector<std::pair<double, int>>, 
							std::greater<std::pair<double, int>>> pq;

			dist[0] = 0; 
			pq.push({0, 0});

			while (!pq.empty()) {
	
				int min_idx = pq.top().second;
				double min_dist = pq.top().first;
				pq.pop();
				
				// skip visited
				if (visited[min_idx]) continue;
				
				visited[min_idx] = true;
				
				// goal reached
				if (min_idx == 1) {
					std::cout << "Goal reached with planner 3: PRM " << std::endl;
					break;
				}
				
				// distance update
				for (int neighbor : neighbors[min_idx]) {
					double edge_cost = euclidean_distance(nodes[min_idx].joint_comb, nodes[neighbor].joint_comb, numofDOFs);
					double path_cost = dist[min_idx] + edge_cost;
					
					if (!visited[neighbor] && path_cost < dist[neighbor]) {
						dist[neighbor] = path_cost;
						prev[neighbor] = min_idx;
						pq.push({dist[neighbor], neighbor});
					}
				}
			}

			// goal path check
			if (prev[1] == -1 || dist[1] == INFINITY) {
				std::cout << "Goal not reached with planner 3: PRM " << std::endl;

				std::vector<std::vector<double>> path;
				path.push_back(std::vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs));
				path.push_back(std::vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs));
				allocatePlanMemory(plan, planlength, path, numofDOFs);
				
				pathCost = euclidean_distance(armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs);
				numVertices = nodes.size();

			} else {
				// reconstruct path
				std::vector<int> path_indices;
				int current = 1; 

				while (current != -1) {
					path_indices.push_back(current);
					current = prev[current];
				}

				std::reverse(path_indices.begin(), path_indices.end());

				std::vector<std::vector<double>> path;
				for (int idx : path_indices) {
					std::vector<double> config(numofDOFs);
					for (int j = 0; j < numofDOFs; j++) {
						config[j] = nodes[idx].joint_comb[j];
					}
					path.push_back(config);
				}

				numVertices = nodes.size();

				std::cout << "Path before shortcut: " << path.size() << " points" << std::endl;

				if (shortcut) {
					shortcutPath(path, numofDOFs, map, x_size, y_size, stepsize);
					std::cout << "Path after shortcut: " << path.size() << " points" << std::endl;
				}
				
				allocatePlanMemory(plan, planlength, path, numofDOFs);

				std::cout << "Path length: " << path.size() << std::endl;
			}
		}

		for (int i = 0; i < numofDOFs; ++i) {
			(*plan)[*planlength - 1][i] = armgoal_anglesV_rad[i];
		}

		// cleanup
		for (size_t i = 0; i < nodes.size(); i++) {
			delete[] nodes[i].joint_comb;
		}
	}
		

	else {
		
	}

	// calculate planning time
	auto endTime = std::chrono::high_resolution_clock::now();
	auto planTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
	std::cout << "Planning time: " << planTime << " ms" << std::endl;

	// calculate path cost
	for (int i = 1; i < *planlength; i++) {
		pathCost += euclidean_distance((*plan)[i-1], (*plan)[i], numofDOFs);
	}
	std::cout << "Path cost: " << pathCost << std::endl;

	// calculate number of vertices
	std::cout << "Number of vertices: " << numVertices << std::endl;


    return;
}


/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
	planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, whichPlanner);

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as the 
	//// grading script will not work.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
