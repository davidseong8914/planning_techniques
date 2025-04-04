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

// Works for maps: 1 ~ 12

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
    
    // Check for collision
    int idx = (y-1)*x_size + (x-1);  
    if (map[idx] < 0 || map[idx] >= collision_thresh) {
        return false;
    }
    return true;
}

// euclidean distance for heuristic
double euclidean_heuristic(int x, int y, int target_x, int target_y) {
    return sqrt((x-target_x)*(x-target_x) + (y-target_y)*(y-target_y));
}

// check if grid is in closed list
bool isInClosed(int x, int y, std::set<std::pair<int, int>> CLOSED) {
    return CLOSED.count({x, y}) > 0;
}

// input target grid coordinate, outputs backward A* heuristic map (coordinates, Backward A* distance)
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
    
    // if target is invalid, find nearest valid target
    if (!foundValid) {
        for (int i = target_steps - 1; i >= 0; i--) {
            validTargetX = target_traj[i];
            validTargetY = target_traj[i + target_steps];
            if (gridValid(validTargetX, validTargetY, x_size, y_size, map, collision_thresh)) {
                foundValid = true;
                break;
            }
        }
        
        // no valid target found
        if (!foundValid) {
            std::cout << "No valid target" << std::endl;
            return distances;
        }
    }

    // Initialize with valid target position
    distances[{validTargetX, validTargetY}] = 0;
    OPEN.push({0, {validTargetX, validTargetY}});

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // Add maximum search radius to limit exploration
    int average_map_size = (x_size + y_size) / 2;
    const int boundary_buffer = average_map_size / 4;
    
    // Calculate search bounds - ALWAYS include robot position and target
    int min_x = std::min(robotposeX, validTargetX);
    int max_x = std::max(robotposeX, validTargetX);
    int min_y = std::min(robotposeY, validTargetY);
    int max_y = std::max(robotposeY, validTargetY);
    
    // Then expand by search radius - Increase the search radius
    int search_radius = std::max(boundary_buffer, 
        (int)euclidean_heuristic(robotposeX, robotposeY, validTargetX, validTargetY));
    
    min_x = std::max(1, min_x - search_radius);
    max_x = std::min(x_size, max_x + search_radius);
    min_y = std::max(1, min_y - search_radius);
    max_y = std::min(y_size, max_y + search_radius);

    while (!OPEN.empty()) {
        auto current = OPEN.top().second;
        int x = current.first;
        int y = current.second;
        
        // Skip if outside search bounds
        if (x < min_x || x > max_x || y < min_y || y > max_y) {
            OPEN.pop();
            continue;
        }
        
        double current_g = distances[current];
        OPEN.pop();

        visited.insert(current);

        // checks neighboring grids
        for(int i = 0; i < NUMOFDIRS; i++) {
            int newx = x + dX[i];
            int newy = y + dY[i];
            
            // Skip if outside search bounds
            if (newx < min_x || newx > max_x || newy < min_y || newy > max_y) {
                continue;
            }
            
            // checks if grid is valid and not visited
            if (gridValid(newx, newy, x_size, y_size, map, collision_thresh) && 
                visited.count({newx, newy}) == 0) {                
                double newg = current_g + 1;
                
                // Only update distance if it's better than existing
                if (distances.find({newx, newy}) == distances.end() || newg < distances[{newx, newy}]) {
                    distances[{newx, newy}] = newg;
                    OPEN.push({newg, {newx, newy}});
                }
            }
        }
    }

    return distances;
}

// Static cache for the heuristic map at planner level
static std::map<std::pair<int, int>, double> cached_h_map;
static bool h_map_initialized = false;
static int mode = 1; 

// planner
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
    // Calculate map area ratio
    double map_area = x_size * y_size;
    double area_ratio = map_area / 1000000.0;  // 1000 x 1000 map
    // double area_ratio = 4.0;
    
    int goalposeX, goalposeY;
    
    // map is greater than 2 * (1000 x 1000)
    if (area_ratio > 2.0) {
        int num_modes = area_ratio;
        // Calculate current target position based on mode
        goalposeX = target_traj[(mode * target_steps)/num_modes]; 
        goalposeY = target_traj[2*target_steps - 1 - ((num_modes-mode) * target_steps)/num_modes];
        
        // Calculate Manhattan distance to current goal
        int manhattan_dist = abs(robotposeX - goalposeX) + abs(robotposeY - goalposeY);
        int steps_remaining = target_steps - curr_time;
        
        // If distance to current goal is too high, move to next quarter
        if (manhattan_dist > steps_remaining && mode < num_modes) {
            mode++;
            h_map_initialized = false;  // Reset to create new heuristic map
            std::cout << "Moving to next target (mode " << mode << ")" << std::endl;
            // Update goal for new mode
            goalposeX = target_traj[(mode * target_steps)/num_modes];
            goalposeY = target_traj[2*target_steps - 1 - ((num_modes-mode) * target_steps)/num_modes];
        }
    } else {
        goalposeX = target_traj[target_steps - 1];
        goalposeY = target_traj[2*target_steps - 1];
    }

    // Initialize heuristic map only once
    if (!h_map_initialized) {
        cached_h_map = heuristic_map(map, collision_thresh, x_size, y_size, goalposeX, goalposeY, robotposeX, robotposeY, target_traj, target_steps);
        h_map_initialized = true;
    }
    
    // Use cached heuristic map
    std::map<std::pair<int, int>, double>& h_map = cached_h_map;

    // Find min and max values in map
    int min_val = 0;
    int max_val = collision_thresh;

    int adaptive_epsilon = (max_val - min_val) / 2.0;
    // 1
    
    // weighted A*
    int epsilon = adaptive_epsilon; 

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    static std::set<std::pair<int, int>> visited_positions;
    visited_positions.insert({robotposeX, robotposeY});
    
    // if goal trajectory is reached and within 10 steps
    std::cout << "goalposeX: " << goalposeX << ", goalposeY: " << goalposeY << std::endl;
    if (robotposeX == goalposeX && robotposeY == goalposeY && (target_steps - curr_time) < 10) {
        std::cout << "On goal trajectory, staying still" << std::endl;
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

 /**/   
    // if on target trajectory (that is in the future), follow trajectory backwards
    // static int last_trajectory_index = target_steps - 1; 
    int last_trajectory_index = target_steps - 1; 


    for (int i = last_trajectory_index; i > 0; i--) {
        // if on target trajectory and within 10 steps stay still
        if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && 0 < (i - curr_time) &&(i - curr_time) < 10) {
            std::cout << "On target trajectory, staying still" << std::endl;
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        } 
        else if (robotposeX == target_traj[i] && robotposeY == target_traj[i+target_steps] && (i > curr_time)) {
            std::cout << "On target trajectory, following backwards" << std::endl;
            last_trajectory_index = i;  // Remember where we are in the trajectory
            action_ptr[0] = target_traj[i-1];
            action_ptr[1] = target_traj[i-1 + target_steps];
            return;
        }
    }

    // current position
    std::pair<int, int> current = std::make_pair(robotposeX, robotposeY);
    
    double best_f = INFINITY;
    std::pair<int, int> best_next;
    bool found_valid_move = false;

    // evaluate 8 neighbor grids
    for (int dir = 0; dir < NUMOFDIRS; dir++) {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];
        
        // if not a valid grid, continue
        if (!gridValid(newx, newy, x_size, y_size, map, collision_thresh)) {
            continue;
        }
        
        // if visited, continue
        if (visited_positions.count({newx, newy}) > 0) {
            continue;
        }
        
        // if not in heuristic map, continue
        auto it = h_map.find({newx, newy});
        if (it == h_map.end()) {
            continue;
        }

        double h = it->second;
        double h_euc = euclidean_heuristic(newx, newy, goalposeX, goalposeY);
        double g = map[(newy-1)*x_size + (newx-1)];  // Use 1-based coordinate conversion
        double f = g + h * epsilon + h_euc;
        
        if (f < best_f) {
            best_f = f;
            best_next = {newx, newy};
            found_valid_move = true;
            visited_positions.insert({newx, newy});
        }
    }

    // valid move and valid grid - move to best next grid
    if (found_valid_move && gridValid(best_next.first, best_next.second, x_size, y_size, map, collision_thresh)) {
        action_ptr[0] = best_next.first;
        action_ptr[1] = best_next.second;
    } else {
        // no valid move found - stay still
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    // Reset visited positions when switching modes or when stuck
    if (!h_map_initialized || visited_positions.size() > 100) {
        visited_positions.clear();
    }

    // unstucking mechanism
    static int last_x = -1, last_y = -1;
    static int stuck_count = 0;
    if (robotposeX == last_x && robotposeY == last_y) {
        stuck_count++;
        if (stuck_count > 5) {
            visited_positions.clear();
            stuck_count = 0;
        }
    } else {
        stuck_count = 0;
    }
    last_x = robotposeX;
    last_y = robotposeY;

    return;
}



