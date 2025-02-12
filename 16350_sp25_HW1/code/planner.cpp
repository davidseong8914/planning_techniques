/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/

/*
Run command:
g++ -std=c++17 runtest.cpp planner.cpp
./a.out undergrad/map5.txt
python visualize.py undergrad/map5.txt

right now catches but doesn't move backwards in trajectory
*/

#include "planner.h"
#include <math.h>
#include <iostream>
#include <queue>
#include <vector>
#include <map>
#include <set>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


bool gridValid(int x, int y, int x_size, int y_size, int* map, int collision_thresh) {
    if (
        x >= 0 && x < x_size &&
        y >= 0 && y < y_size &&
        map[GETMAPINDEX(x, y, x_size, y_size)] >= 0 &&
        map[GETMAPINDEX(x, y, x_size, y_size)] < collision_thresh
    ) {
        return true;
    } else {
        return false;
    }
}

double euclidean_heuristic(int x, int y, int target_x, int target_y) {
    return sqrt((x-target_x)*(x-target_x) + (y-target_y)*(y-target_y));
}

bool isInClosed(int x, int y, std::vector<std::pair<int, int>> CLOSED) {
    for (auto it = CLOSED.begin(); it != CLOSED.end(); it++) {
        if (it->first == x && it->second == y) {
            return true;
        }
    }
    return false;
}

// input target grid coordinate, outputs backward A* heuristic map
// start from the target, expand in all directions until s_start is found
// if s_start found, move to that grid
std::map<std::pair<int, int>, double> heuristic_map(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int targetX,
    int targetY,
    int robotposeX,
    int robotposeY,
    int* target_traj,
    int target_steps
) {
    std::map<std::pair<int, int>, double> distances;
    std::set<std::pair<int, int>> visited;

    std::priority_queue<
        std::pair<double, std::pair<int, int>>,
        std::vector<std::pair<double, std::pair<int, int>>>,
        std::greater<std::pair<double, std::pair<int, int>>>
    > OPEN;

    // Find valid target position by backtracking through trajectory
    int validTargetX = targetX;
    int validTargetY = targetY;
    bool foundValid = gridValid(targetX, targetY, x_size, y_size, map, collision_thresh);
    
    if (!foundValid) {
        std::cout << "Target position invalid, searching for valid target..." << std::endl;
        for (int i = target_steps - 1; i >= 0; i--) {
            validTargetX = target_traj[i];
            validTargetY = target_traj[i + target_steps];
            if (gridValid(validTargetX, validTargetY, x_size, y_size, map, collision_thresh)) {
                foundValid = true;
                std::cout << "Found valid target at index " << i << ": (" 
                         << validTargetX << "," << validTargetY << ")" << std::endl;
                break;
            }
        }
        
        if (!foundValid) {
            std::cout << "Warning: No valid target found in trajectory" << std::endl;
            // Return empty map if no valid target found
            return distances;
        }
    }

    // Initialize with valid target position
    distances[{validTargetX, validTargetY}] = 0;
    OPEN.push({0, {validTargetX, validTargetY}});

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    while (!OPEN.empty()) {
        auto current = OPEN.top().second;
        double current_g = distances[current];
        OPEN.pop();

        // Skip if already visited
        if (visited.count(current) > 0) {
            continue;
        }
        visited.insert(current);

        // checks neighboring grids
        for(int i = 0; i < NUMOFDIRS; i++) {
            int newx = current.first + dX[i];
            int newy = current.second + dY[i];
            std::pair<int, int> neighbor = {newx, newy};

            // checks if grid is valid and not visited
            if (gridValid(newx, newy, x_size, y_size, map, collision_thresh) && 
                visited.count(neighbor) == 0) {
                // std::cout << "Valid grid and not visited" << std::endl;
                
                double newg = current_g + 1;
                
                // Only update distance if it's better than existing
                if (distances.find(neighbor) == distances.end() || newg < distances[neighbor]) {
                    distances[neighbor] = newg;
                    OPEN.push({newg, neighbor});
                }
            }
        }
    }

    return distances; // returns map of coordinates, distances
}


// expand using heuristic map
// if on robot trajectory, follow trajectory backwards
void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
)
{
    
    // weighted A*
    int epsilon = 50; // with 5 it didn't choose the most optimal path

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // current target position
    // int goalposeX = target_traj[curr_time-1];
    // int goalposeY = target_traj[curr_time-1 + target_steps];
    
    // set target to end of the target trajectory
    int goalposeX = target_traj[target_steps-1]; 
    int goalposeY = target_traj[2*target_steps-1];

    // look 10 steps ahead if not end of target trajectory
    // int goalposeX = target_traj[std::min(curr_time + 30, target_steps-1)];
    // int goalposeY = target_traj[std::min(curr_time + 30, 2*target_steps-1)];

    static std::set<std::pair<int, int>> visited_positions;
    visited_positions.insert({robotposeX, robotposeY});
    
    // if goal trajectory is reached and within 10 steps
    if (robotposeX == goalposeX && robotposeY == goalposeY && (target_steps - curr_time) < 20) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    
    // if on target trajectory (that is in the future), follow trajectory backwards
    for (int i = target_steps - 1; i > 0; i--) {
        // if on target trajectory and within 10 steps stay still
        if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && (i - curr_time) < 20) {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
        } else if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && (i > curr_time)) {
        // if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && (i > curr_time)) {
                std::cout << "On target trajectory, following backwards" << std::endl;
                std::cout << target_traj[i-1] << " " << target_traj[i+target_steps-1] << std::endl;
                action_ptr[0] = target_traj[i - 1];
                action_ptr[1] = target_traj[i + target_steps - 1];
                return;
        }
    }

    // Initialize backward A* heuristic map
    std::map<std::pair<int, int>, double> h_map = heuristic_map(map, collision_thresh, x_size, y_size, goalposeX, goalposeY, robotposeX, robotposeY, target_traj, target_steps);
    
    std::cout << "Heuristic map size: " << h_map.size() << std::endl;
    std::cout << "Robot position: (" << robotposeX << "," << robotposeY << ")" << std::endl;
    std::cout << "Goal position: (" << goalposeX << "," << goalposeY << ")" << std::endl;
    
    // current position
    std::pair<int, int> current = std::make_pair(robotposeX, robotposeY);
    
    double best_f = INFINITY;
    std::pair<int, int> best_next;
    bool found_valid_move = false;

    // evaluate 8 neighbor grids
    for (int i = 0; i < NUMOFDIRS; i++) {
        int newx = current.first + dX[i];
        int newy = current.second + dY[i];
        // std::cout << "\nChecking neighbor " << i << ": (" << newx << "," << newy << ")" << std::endl;
        
        // // Skip if we've already visited this position
        // if (visited_positions.count({newx, newy}) > 0) {
        //     std::cout << "Skipping previously visited position" << std::endl;
        //     continue;
        // }
        
        bool valid = gridValid(newx, newy, x_size, y_size, map, collision_thresh);
        std::cout << "Grid valid: " << (valid ? "yes" : "no") << std::endl;
        
        if (valid) {
            auto it = h_map.find({newx, newy});
            std::cout << "Found in h_map: " << (it != h_map.end() ? "yes" : "no") << std::endl;
            
            if (it != h_map.end()) {
                double h = it->second;
                double h_euc = euclidean_heuristic(newx, newy, goalposeX, goalposeY); // second heuristic
                double g = map[GETMAPINDEX(newx, newy, x_size, y_size)];
                double f = g + h * epsilon + h_euc * epsilon/2;
                
                std::cout << "Values | g: " << g << ", h: " << h << ", f: " << f << ", best_f: " << best_f << std::endl;
                
                if (f < best_f) {
                    best_f = f;
                    best_next = {newx, newy};
                    found_valid_move = true;
                    std::cout << "New best move!" << std::endl;
                    visited_positions.insert({newx, newy});
                }
            }
        }
    }

    // std::cout << "\nFound valid move: " << (found_valid_move ? "yes" : "no") << std::endl;
    if (found_valid_move) {
        std::cout << "Best next position: (" << best_next.first << "," << best_next.second << ")" << std::endl;
    }

    // if no valid move found, stay in place
    if (!found_valid_move) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        std::cout << "No valid moves found, staying in place." << std::endl;
        return;
    }

    // move to the best next position
    action_ptr[0] = best_next.first;
    action_ptr[1] = best_next.second;

    std::cout << "Current: (" << robotposeX << "," << robotposeY 
              << ") Next: (" << action_ptr[0] << "," << action_ptr[1] 
              << ") Goal: (" << goalposeX << "," << goalposeY << ")" << std::endl;
    return;
}



