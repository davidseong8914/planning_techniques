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
#include <algorithm>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


bool gridValid(int x, int y, int x_size, int y_size, int* map, int collision_thresh) {
    // Check bounds first (1-based coordinates)
    if (x < 1 || x > x_size || y < 1 || y > y_size) {
        return false;
    }
    
    // Then check collision (convert to 0-based for array access)
    int idx = (y-1)*x_size + (x-1);  // Manual index calculation for 1-based coords
    if (map[idx] < 0 || map[idx] >= collision_thresh) {
        return false;
    }
    
    return true;
}

double euclidean_heuristic(int x, int y, int target_x, int target_y) {
    return sqrt((x-target_x)*(x-target_x) + (y-target_y)*(y-target_y));
}

bool isInClosed(int x, int y, std::set<std::pair<int, int>> CLOSED) {
    return CLOSED.count({x, y}) > 0;
}

// bool isInClosed(int x, int y, std::vector<std::pair<int, int>> CLOSED) {
//     for (auto it = CLOSED.begin(); it != CLOSED.end(); it++) {
//         if (it->first == x && it->second == y) {
//             return true;
//         }
//     }
//     return false;
// }

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
    static std::map<std::pair<int, int>, double> cached_map;
    static int cached_targetX = -1;
    static int cached_targetY = -1;
    
    // Only recalculate if target has moved significantly
    if (cached_targetX == targetX && cached_targetY == targetY) {
        return cached_map;
    }
    
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

    cached_map = distances;
    cached_targetX = validTargetX;
    cached_targetY = validTargetY;
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
    // Find min and max values in map
    int min_val = map[0];
    int max_val = map[0];
    for(int i = 0; i < x_size * y_size; i++) {
        if(map[i] < min_val) min_val = map[i];
        if(map[i] > max_val) max_val = map[i];
    }
    // int adaptive_epsilon = (max_val - min_val) / 2; // worked for maps 1,2, 3,4, 5 (X)
    int adaptive_epsilon = (max_val - min_val) / 1.2; 
    // worked for map 1, 2 (nop opt), 3 (not opt), 4 (not opt), 5, 6, 7 (might be tricky), 8, 9 (could account for the steps but this will take more time to implement), 10
    
    // weighted A*
    int epsilon = adaptive_epsilon; // with 5 it didn't choose the most optimal path

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
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
        std::cout << "Goal trajectory reached, staying still" << std::endl;
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    
    // if on target trajectory (that is in the future), follow trajectory backwards
    for (int i = target_steps - 1; i > 0; i--) {
        // if on target trajectory and within 10 steps stay still
        if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && 0 < (i - curr_time) &&(i - curr_time) < 20) {
            std::cout << "On target trajectory, staying still" << std::endl;
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        } 
        else if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && (i > curr_time)) {
                std::cout << "On target trajectory, following backwards" << std::endl;
                std::cout << target_traj[i-1] << " " << target_traj[i+target_steps-1] << std::endl;
                action_ptr[0] = target_traj[i - 1];
                action_ptr[1] = target_traj[i + target_steps - 1];
                return;
        }
    }

    // Initialize backward A* heuristic map
    std::map<std::pair<int, int>, double> h_map = heuristic_map(map, collision_thresh, x_size, y_size, goalposeX, goalposeY, robotposeX, robotposeY, target_traj, target_steps);
    
    // current position
    std::pair<int, int> current = std::make_pair(robotposeX, robotposeY);
    
    double best_f = INFINITY;
    std::pair<int, int> best_next;
    bool found_valid_move = false;

    // evaluate 8 neighbor grids
    for (int dir = 0; dir < NUMOFDIRS; dir++) {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];
        
        // Double check validity before considering move
        if (!gridValid(newx, newy, x_size, y_size, map, collision_thresh)) {
            continue;
        }
        
        if (visited_positions.count({newx, newy}) > 0) {
            continue;
        }
        
        auto it = h_map.find({newx, newy});
        if (it == h_map.end()) {
            continue;
        }

        double h = it->second;
        double h_euc = euclidean_heuristic(newx, newy, goalposeX, goalposeY);
        double g = map[(newy-1)*x_size + (newx-1)];  // Use 1-based coordinate conversion
        double f = g + h * epsilon + h_euc * epsilon/2;
        
        if (f < best_f) {
            best_f = f;
            best_next = {newx, newy};
            found_valid_move = true;
            visited_positions.insert({newx, newy});
        }
    }

    // Safety check before applying move
    if (found_valid_move && gridValid(best_next.first, best_next.second, x_size, y_size, map, collision_thresh)) {
        action_ptr[0] = best_next.first;
        action_ptr[1] = best_next.second;
    } else {
        // Stay in place if no valid move found
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    return;
}



