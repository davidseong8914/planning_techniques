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
    } else {
        // stepsize towards target
        for (int i = 0; i < numofDOFs; i++) {
            q_new.joint_comb[i] = nodes[neighbor_node_idx].joint_comb[i] + (direction_vector[i] / magnitude) * stepsize;
        }
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
	int bias_check = 10;
	// angle step size
	double epsilon = 0.5;
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
			// formatting
			*planlength = RRT_path.size();
			std::cout << RRT_path.size();

			*plan = (double**) malloc((*planlength) * sizeof(double*));
			for (int i = 0; i < *planlength; i++) {
				(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[i][j] = nodes[RRT_path[i]].joint_comb[j];
				}
			}

			// cleanup
			for (size_t i = 0; i < nodes.size(); i++) {
				delete[] nodes[i].joint_comb;
			}
	}
	
	// RRT-Connect : planner id = 1
	else if (whichPlanner == 1) {
		// initialize trees
		nodes_init.push_back(q_init); // tree A
		nodes_goal.push_back(q_goal); // tree B
		// bool tree_switch = true;

		// keep track of trees A and B being connected
		int init_idx = 0;
		int goal_idx = 0;
		bool connection = false;
		bool swapped = false;

		// random sampling (need to make it biased towards goal)
		for (int k = 0; k < K && connection == false; k++) {
			Node q_rand;
			q_rand.joint_comb = new double [numofDOFs];
			if (k % bias_check == 0) {
				if (swapped == false) {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = q_goal.joint_comb[i];
				} }
				else {
					for (int i = 0; i < numofDOFs; i++) {
						q_rand.joint_comb[i] = q_init.joint_comb[i];
					}
				}
			} else {
				for (int i = 0; i < numofDOFs; i++) {
					q_rand.joint_comb[i] = distribution(gen);
				}
			}

			if (extend(nodes_init, q_rand, epsilon, numofDOFs, map, x_size, y_size) != TRAPPED) {
				// setting last ndoe in init tree as target for goal tree
				Node connect_target;
				connect_target.joint_comb = new double[numofDOFs];
				for (int i=0; i<numofDOFs; i++) {
					connect_target.joint_comb[i] = nodes_init.back().joint_comb[i];
				}





				ExtendStatus status = ADVANCED;
				while (status == ADVANCED && !connection) {
					status = extend(nodes_goal, connect_target, epsilon, numofDOFs, map, x_size, y_size);
					if (status == REACHED) {
						connection = true;
						init_idx = nodes_init.size() - 1;
						goal_idx = nodes_goal.size() - 1;
						break;
					}
					// update target
					for (int i = 0; i < numofDOFs; i++) {
						connect_target.joint_comb[i] = nodes_goal.back().joint_comb[i];
					}
				}




				delete[] connect_target.joint_comb;
			}

			// if not connected -> swap trees and repeat
			if (connection == false) {
				std::vector<Node> temp = nodes_init;
				nodes_init = nodes_goal;
				nodes_goal = temp;
				swapped = !swapped;
			}

			delete[] q_rand.joint_comb;
		}

		// if connected
		if (connection) {
			std::cout << "Connected \n";
			std::vector<int> init_path;
			std::vector<int> goal_path;

			// untangling swaps
			if (swapped) {
				std::swap(init_idx, goal_idx);
				std::swap(nodes_init, nodes_goal);
			}

			// init path
			int current = init_idx;
			while (current != -1) {
				init_path.push_back(current);
				current = nodes_init[current].parent;
			}
			std::reverse(init_path.begin(), init_path.end());

			// goal path
			current = goal_idx;
			while (current != -1) {
				goal_path.push_back(current);
				current = nodes_goal[current].parent;
			}

			// Calculate total path length
			*planlength = init_path.size() + goal_path.size();
			
			// Allocate memory for the path
			*plan = (double**) malloc((*planlength) * sizeof(double*));
			
			// Fill in path from start to connection point
			for (int i = 0; i < init_path.size(); i++) {
				(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[i][j] = nodes_init[init_path[i]].joint_comb[j];
				}
			}
        
			// Fill in path from connection point to goal
			for (int i = 0; i < goal_path.size(); i++) {
				int plan_idx = init_path.size() + i;
				(*plan)[plan_idx] = (double*) malloc(numofDOFs * sizeof(double));
				for (int j = 0; j < numofDOFs; j++) {
					(*plan)[plan_idx][j] = nodes_goal[goal_path[i]].joint_comb[j];
				}
			}
		} else {
			std::cout << "Trees have not been connected" << std::endl;
			// Set empty plan
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

	}

	//PRM : planner id = 3
	else if (whichPlanner == 3) {

	}

	else {
		//no plan by default
		*plan = NULL;
		*planlength = 0;
			
		//for now just do straight interpolation between start and goal checking for the validity of samples
		double distance = 0;
		int i,j;
		for (j = 0; j < numofDOFs; j++){
			if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
				distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
		}
		int numofsamples = (int)(distance/(PI/20));
		if(numofsamples < 2){
			printf("The arm is already at the goal\n");
			return;
		}
		int countNumInvalid = 0;
		*plan = (double**) malloc(numofsamples*sizeof(double*));
		for (i = 0; i < numofsamples; i++){
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
			for(j = 0; j < numofDOFs; j++){
				(*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
			}
			if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
				++countNumInvalid;
			}
		}
		printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
		*planlength = numofsamples;
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
