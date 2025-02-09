/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/

/*
Run command:
g++ -std=c++17 runtest.cpp planner.cpp
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
        x >= 0 && x <= x_size &&
        y >= 0 && y <= y_size &&
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
    int targetY
) {
    std::map<std::pair<int, int>, double> distances;
    std::set<std::pair<int, int>> visited;  // Replace vector with set for O(log n) lookup

    std::priority_queue<
        std::pair<double, std::pair<int, int>>,
        std::vector<std::pair<double, std::pair<int, int>>>,
        std::greater<std::pair<double, std::pair<int, int>>>
    > OPEN;

    // Initialize with target
    distances[{targetX, targetY}] = 0; // coordinates, coordinates
    visited.insert({targetX, targetY}); // visited coordinates
    OPEN.push({0, {targetX, targetY}}); // Open set > used for A* expansion

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    while (!OPEN.empty()) {
        auto current = OPEN.top().second; // current coordinates
        double current_g = OPEN.top().first; // current g value
        OPEN.pop();

        // cell already visited
        if (visited.count(current) > 0) {
            continue; 
        }
        else {
            visited.insert(current);
        }

        // checks neighboring grids
        for(int i = 0; i < NUMOFDIRS; i++) {
            int newx = current.first + dX[i];
            int newy = current.second + dY[i];
            std::pair<int, int> neighbor = {newx, newy};

            // checks if grid is valid and not visited
            if (gridValid(newx, newy, x_size, y_size, map, collision_thresh) && 
                visited.count(neighbor) == 0) {
                
                // g value of neighbor
                double newg = current_g + 1;
                distances[neighbor] = newg;
                OPEN.push({newg, neighbor});
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
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    // current target position
    int goalposeX = target_traj[curr_time-1];
    int goalposeY = target_traj[curr_time-1 + target_steps];

    // if goal trajectory is reached
    if (robotposeX == goalposeX && robotposeY == goalposeY) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    // Initialize backward A* heuristic map
    std::map<std::pair<int, int>, double> h_map = heuristic_map(map, collision_thresh, x_size, y_size, goalposeX, goalposeY);
    // current position
    std::pair<int, int> current = std::make_pair(robotposeX, robotposeY);
    // OPEN (considers 8 neighbors)
    std::priority_queue<
        std::pair<double, std::pair<int, int>>,
        std::vector<std::pair<double, std::pair<int, int>>>,
        std::greater<std::pair<double, std::pair<int, int>>>
    > OPEN;

    // evaluate 8 neighbor grids
    for (int i = 0; i < NUMOFDIRS; i++) {
        int newx = current.first + dX[i];
        int newy = current.second + dY[i];

        if (gridValid(newx, newy, x_size, y_size, map, collision_thresh)) {
            // Safely access h_map
            auto it = h_map.find({newx, newy});
            if (it != h_map.end()) {
                double h = it->second;
                double g = map[GETMAPINDEX(newx, newy, x_size, y_size)];
                double f = g + h;
                OPEN.push(std::make_pair(f, std::make_pair(newx, newy)));
                std::cout << "(" << newx << "," << newy << ") f=" << f 
                         << " (g=" << g << ", h=" << h << ")" << std::endl;
            }
        }
    }

    // Check for empty queue
    if (OPEN.empty()) {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    std::pair<int, int> next = OPEN.top().second;
    action_ptr[0] = next.first;
    action_ptr[1] = next.second;

    std::cout << "Current: (" << robotposeX << "," << robotposeY 
              << ") Next: (" << action_ptr[0] << "," << action_ptr[1] 
              << ") Goal: (" << goalposeX << "," << goalposeY << ")" << std::endl;
    return;
}



