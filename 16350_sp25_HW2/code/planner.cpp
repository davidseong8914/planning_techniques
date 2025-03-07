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

./a.out undergrad/map5.txt
python visualize.py undergrad/map5.txt
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

struct Node {
	double* joint_comb;
	int parent;
	double cost;
};

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

    if (magnitude <= stepsize) {
        // if closer than stepsize, directly to target
        for (int i = 0; i < numofDOFs; i++) {
            q_new.joint_comb[i] = q_rand.joint_comb[i];
        }
		q_new.cost = nodes[neighbor_node_idx].cost + (magnitude <= stepsize ? magnitude : stepsize);

    } else {
        // stepsize towards target
        for (int i = 0; i < numofDOFs; i++) {
            q_new.joint_comb[i] = nodes[neighbor_node_idx].joint_comb[i] + (direction_vector[i] / magnitude) * stepsize;
        }
		q_new.cost = nodes[neighbor_node_idx].cost + (magnitude <= stepsize ? magnitude : stepsize);

    }

    if (IsValidArmConfiguration(q_new.joint_comb, numofDOFs, map, x_size, y_size)) {
        nodes.push_back(q_new);
        if (magnitude <= stepsize) {
            return REACHED;
        }
        return ADVANCED;
    } else {
        delete[] q_new.joint_comb;
        return TRAPPED;
    }
}

ExtendStatus connect(std::vector<Node>& nodes, const Node& q_target, double stepsize, int numofDOFs, double* map, int x_size, int y_size) {
    ExtendStatus status = ADVANCED;
    
    while (status == ADVANCED) {
        status = extend(nodes, q_target, stepsize, numofDOFs, map, x_size, y_size);
    }
    
    return status;
}

bool isValidStraightLinePath(double* config1, double* config2, int numofDOFs, double* map, int x_size, int y_size, double step_size = 0.1) {
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

// shortcut post processing for RRT-connect
void shortcutPath(std::vector<std::vector<double>>& path, int numofDOFs, double* map, int x_size, int y_size) {
    if (path.size() <= 2) return; // Nothing to shortcut
    
    // Save exact copies of start and goal configurations
    std::vector<double> exact_start = path.front();
    std::vector<double> exact_goal = path.back();
    
    // First pass - remove unnecessary intermediate points
    std::vector<std::vector<double>> newPath;
    newPath.push_back(exact_start);
    
    // P = start point
    int P = 0;
    // P1 = point P+1 along the path
    int P1 = 1;
    
    // While P is not the second-to-last point
    while (P < path.size() - 1) {
        // Try to extend P1 as far as possible
        while (P1 + 1 < path.size() && 
               isValidStraightLinePath(&path[P][0], &path[P1+1][0], numofDOFs, map, x_size, y_size)) {
            P1 = P1 + 1;
        }
        
        // Add the segment [P,P1] to the new path
        if (P1 < path.size() - 1 || P1 == path.size() - 1) {
            newPath.push_back(path[P1]);
        }
        
        // Move P to P1
        P = P1;
        // Move P1 to the next point
        P1 = P + 1;
        
        // If P1 goes beyond the path, break
        if (P1 >= path.size()) {
            break;
        }
    }
    
    // Make sure the goal is the last point
    if (newPath.back() != exact_goal) {
        newPath.push_back(exact_goal);
    }
    
    // Second pass - smooth transitions by adding interpolated points
    // if there are large angle changes between consecutive configurations
    std::vector<std::vector<double>> smoothedPath;
    smoothedPath.push_back(newPath[0]);
    
    for (int i = 1; i < newPath.size(); i++) {
        // Check if there's a large jump between configurations
        double dist = 0.0;
        for (int j = 0; j < numofDOFs; j++) {
            double diff = newPath[i][j] - newPath[i-1][j];
            dist += diff * diff;
        }
        dist = sqrt(dist);
        
        // if distance between points is larger than 1.0 (arbitrary)
        if (dist > 1.0) {
            int num_steps = std::max(2, (int)(dist / 0.5));
            
            for (int step = 1; step < num_steps; step++) {
                double ratio = (double)step / num_steps;
                std::vector<double> intermediate(numofDOFs);
                
                for (int j = 0; j < numofDOFs; j++) {
                    intermediate[j] = newPath[i-1][j] + ratio * (newPath[i][j] - newPath[i-1][j]);
                }
                
                if (IsValidArmConfiguration(&intermediate[0], numofDOFs, map, x_size, y_size)) {
                    smoothedPath.push_back(intermediate);
                }
            }
        }
        
        smoothedPath.push_back(newPath[i]);
    }
    
    // Replace the original path with the smoothed path
    path = smoothedPath;
}


// for RRT*
std::vector<int> near_neighbors(const std::vector<Node>& nodes, int node_idx, double radius, int numofDOFs) {
    std::vector<int> neighbors;
    for (int i = 0; i < nodes.size(); i++) {
        if (euclidean_distance(nodes[i].joint_comb, nodes[node_idx].joint_comb, numofDOFs) <= radius) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

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
    
    // Update parent and cost
    nodes[new_node_idx].parent = min_cost_idx;
    nodes[new_node_idx].cost = min_cost;
    
    return min_cost_idx;
}

void rewire(std::vector<Node>& nodes, int new_node_idx, const std::vector<int>& near_indices, 
           double* map, int x_size, int y_size, int numofDOFs) {
    for (int near_idx : near_indices) {
        // Skip parent
        if (near_idx == nodes[new_node_idx].parent) continue;
        
        double edge_cost = euclidean_distance(nodes[new_node_idx].joint_comb, nodes[near_idx].joint_comb, numofDOFs);
        double potential_cost = nodes[new_node_idx].cost + edge_cost;
        
        if (potential_cost < nodes[near_idx].cost && 
            isValidStraightLinePath(nodes[new_node_idx].joint_comb, nodes[near_idx].joint_comb, numofDOFs, map, x_size, y_size)) {
            // Rewire: change parent and update cost
            nodes[near_idx].parent = new_node_idx;
            nodes[near_idx].cost = potential_cost;
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
	int K = 100000;
	// how often to generate biased node
	int bias_check = 30;
	// angle step size
	double epsilon = 0.2; // epsilisimo
	// double epsilon = PI/20;

	// initial  node
	Node q_init;
	// memory for joint angles
	q_init.joint_comb = new double[numofDOFs];
	// set initial configuration
	for (int i = 0; i < numofDOFs; i++) {
		q_init.joint_comb[i] = armstart_anglesV_rad[i];
	}
	//set index
	q_init.parent = -1;

	// final node
	Node q_goal;
	q_goal.joint_comb = new double[numofDOFs];
	for (int i = 0; i < numofDOFs; i++) {
		q_goal.joint_comb[i] = armgoal_anglesV_rad[i];
	}

	// generate random angle between 0 - 2pi
	std::random_device rd;  // Used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); // Standard mersenne_twister_engine
	std::uniform_real_distribution<double> distribution(0.0, 2.0 * PI);

	// RRT : planner id = 0
	if (whichPlanner == 0) {
		nodes.push_back(q_init);
		bool goal_reached = false;

		// random sampling (need to make it biased towards goal)
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
		
		// Convert path indices to configuration vectors for smoothing
		std::vector<std::vector<double>> path;
		for (int idx : RRT_path) {
			std::vector<double> config(numofDOFs);
			for (int j = 0; j < numofDOFs; j++) {
				config[j] = nodes[idx].joint_comb[j];
			}
			path.push_back(config);
		}
		
		// Make sure start and end points are exact
		path[0] = std::vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
		path[path.size()-1] = std::vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
		
		// Apply shortcutting and smoothing
		shortcutPath(path, numofDOFs, map, x_size, y_size);
		
		// Set plan length and allocate memory
		*planlength = path.size();
		*plan = (double**) malloc((*planlength) * sizeof(double*));
		
		// Fill the plan with the path
		for (int i = 0; i < *planlength; i++) {
			(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = path[i][j];
			}
		}
		
		std::cout << "Path length: " << path.size() << std::endl;
		
		// cleanup
		for (size_t i = 0; i < nodes.size(); i++) {
			delete[] nodes[i].joint_comb;
		}
	}
	
	// RRT-Connect : planner id = 1
	else if (whichPlanner == 1) {
		// Set parent for initial nodes
		q_init.parent = -1;
		q_goal.parent = -1;
		
		// initialize trees
		nodes_init.push_back(q_init); // tree A (start)
		nodes_goal.push_back(q_goal); // tree B (goal)
		
		// track connection points
		int connect_init_idx = -1;
		int connect_goal_idx = -1;
		bool connection = false;
		
		// random sampling
		for (int k = 0; k < K && !connection; k++) {
			// Generate random configuration
			Node q_rand;
			q_rand.joint_comb = new double[numofDOFs];
			
			// Occasionally bias towards goal
			if (k % bias_check == 0) {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = q_goal.joint_comb[i];
				}
			} else {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = distribution(gen);
				}
			}
			
			// Extend tree A toward random sample
			ExtendStatus status_init = extend(nodes_init, q_rand, epsilon, numofDOFs, map, x_size, y_size);
			
			if (status_init != TRAPPED) {
				// Create node for latest addition to tree A
				Node q_new;
				q_new.joint_comb = new double[numofDOFs];
				for (int i = 0; i < numofDOFs; i++) {
					q_new.joint_comb[i] = nodes_init.back().joint_comb[i];
				}
				
				// Try to connect tree B to this new node
				ExtendStatus status_goal = connect(nodes_goal, q_new, epsilon, numofDOFs, map, x_size, y_size);
				
				if (status_goal == REACHED) {
					// Trees connected!
					connection = true;
					connect_init_idx = nodes_init.size() - 1;
					connect_goal_idx = nodes_goal.size() - 1;
				}
				
				delete[] q_new.joint_comb;
			}
			
			// If no connection, swap trees for next iteration
			if (!connection) {
				std::swap(nodes_init, nodes_goal);
			}
			
			delete[] q_rand.joint_comb;
		}
		
		if (connection) {
			std::cout << "Goal reached with planner 1: RRT-Connect" << std::endl;
			
			// Build path from both trees
			std::vector<std::vector<double>> path;
			
			// Save exact start and goal configurations
			std::vector<double> exact_start(numofDOFs);
			std::vector<double> exact_goal(numofDOFs);
			
			for (int i = 0; i < numofDOFs; i++) {
				exact_start[i] = armstart_anglesV_rad[i];
				exact_goal[i] = armgoal_anglesV_rad[i];
			}
			
			// Always start with the exact start configuration
			path.push_back(exact_start);
			
			// Trace path from start to connection point
			if (connect_init_idx >= 0) {
				std::vector<int> init_path;
				int current = connect_init_idx;
				
				while (current != -1 && current != 0) { // Skip the start node since we added exact_start
					init_path.push_back(current);
					current = nodes_init[current].parent;
				}
				
				// Add init path in reverse order (near start to connection)
				for (int i = init_path.size() - 1; i >= 0; i--) {
					std::vector<double> config(numofDOFs);
					for (int j = 0; j < numofDOFs; j++) {
						config[j] = nodes_init[init_path[i]].joint_comb[j];
					}
					path.push_back(config);
				}
			}
			
			// Trace path from connection point to goal
			if (connect_goal_idx >= 0) {
				std::vector<int> goal_path;
				int current = connect_goal_idx;
				
				while (current != -1 && current != 0) { // Skip the goal node since we'll add exact_goal
					goal_path.push_back(current);
					current = nodes_goal[current].parent;
				}
				
				// Add goal path in forward order (connection to near goal)
				for (int i = 1; i < goal_path.size(); i++) {
					std::vector<double> config(numofDOFs);
					for (int j = 0; j < numofDOFs; j++) {
						config[j] = nodes_goal[goal_path[i]].joint_comb[j];
					}
					path.push_back(config);
				}
			}
			
			// Always end with the exact goal configuration
			path.push_back(exact_goal);

			shortcutPath(path, numofDOFs, map, x_size, y_size);
			
			// Set plan length and allocate memory
			*planlength = path.size();
			*plan = (double**) malloc((*planlength) * sizeof(double*));
			
			// Fill the plan with the path
			for (int i = 0; i < *planlength; i++) {
				(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[i][j] = path[i][j];
				}
			}

			std::cout << "Path length: " << path.size() << std::endl;

		} else {
			std::cout << "Failed to connect trees" << std::endl;
			*plan = NULL;
			*planlength = 0;
		}
		
		// Cleanup
		for (size_t i = 0; i < nodes_init.size(); i++) {
			delete[] nodes_init[i].joint_comb;
		}
		for (size_t i = 0; i < nodes_goal.size(); i++) {
			delete[] nodes_goal[i].joint_comb;
		}
	}

	// RRT* : planner id = 2
	else if (whichPlanner == 2) {
		q_init.cost = 0.0;
		nodes.push_back(q_init);
		bool goal_reached = false;

		// radius for nearest neighbor = epsilon ? gamma
		double gamma = 1.0;

		for (int k = 0; k < K && !goal_reached; k++) {
			Node q_rand;
			q_rand.joint_comb = new double[numofDOFs];

			if (k % bias_check == 0) {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = q_goal.joint_comb[i];
				}
			} else {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = distribution(gen);
				}
			}

			// Extend tree toward random configuration
			int old_size = nodes.size();
			ExtendStatus status = extend(nodes, q_rand, epsilon, numofDOFs, map, x_size, y_size);
			delete[] q_rand.joint_comb;
			
			if (status != TRAPPED) {
				// Calculate radius based on number of nodes
				double radius = std::min(gamma * std::pow(std::log(nodes.size()) / nodes.size(), 1.0/numofDOFs), epsilon * 3.0);
				
				// Find neighbors within radius
				std::vector<int> near_indices = near_neighbors(nodes, nodes.size()-1, radius, numofDOFs);
				
				// Choose best parent
				choose_parent(nodes, nodes.size()-1, near_indices, map, x_size, y_size, numofDOFs);
				
				// Rewire the tree
				rewire(nodes, nodes.size()-1, near_indices, map, x_size, y_size, numofDOFs);
			}
			
			// Check for goal
			if (k % bias_check == 0) {
				double dist_to_goal = euclidean_distance(nodes.back().joint_comb, q_goal.joint_comb, numofDOFs);
				if (dist_to_goal < epsilon && isValidStraightLinePath(nodes.back().joint_comb, q_goal.joint_comb, numofDOFs, map, x_size, y_size)) {
					q_goal.parent = nodes.size() - 1;
					q_goal.cost = nodes.back().cost + dist_to_goal;
					nodes.push_back(q_goal);
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
			std::cout << "Goal reached with planner 2: RRT* " << std::endl;
		} else {
		// goal not reached -> go to nearest 
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
		
		// Convert path indices to configuration vectors for smoothing
		std::vector<std::vector<double>> path;
		for (int idx : RRT_path) {
			std::vector<double> config(numofDOFs);
			for (int j = 0; j < numofDOFs; j++) {
				config[j] = nodes[idx].joint_comb[j];
			}
			path.push_back(config);
		}
		
		// Make sure start and end points are exact
		path[0] = std::vector<double>(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
		path[path.size()-1] = std::vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
		
		// Apply shortcutting and smoothing
		shortcutPath(path, numofDOFs, map, x_size, y_size);
		
		// Set plan length and allocate memory
		*planlength = path.size();
		*plan = (double**) malloc((*planlength) * sizeof(double*));
		
		// Fill the plan with the path
		for (int i = 0; i < *planlength; i++) {
			(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
			for (int j = 0; j < numofDOFs; j++) {
				(*plan)[i][j] = path[i][j];
			}
		}
		
		std::cout << "Path length: " << path.size() << std::endl;
		
		// cleanup
		for (size_t i = 0; i < nodes.size(); i++) {
			delete[] nodes[i].joint_comb;
		}
	}

	//PRM : planner id = 3
	else if (whichPlanner == 3) {

	}

	else {

	}

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
