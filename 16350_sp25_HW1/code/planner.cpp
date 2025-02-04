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

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

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
    
    int goalposeX = target_traj[curr_time-1];
    int goalposeY = target_traj[curr_time-1 + target_steps];

    // Initialize OPEN
    std::priority_queue<
        std::pair<double, std::pair<int, int>>,
        std::vector<std::pair<double, std::pair<int, int>>>,
        std::greater<std::pair<double, std::pair<int, int>>>
    > OPEN;

    // Initialize Closed
    std::vector<std::pair<int, int>> CLOSED;


    // Edge case where start is invalid
    if (map[GETMAPINDEX(robotposeX, robotposeY, x_size, y_size)] >= collision_thresh) {
        // stay in place
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    // evalue 8 neighbor grids
    for (int i = 0; i < NUMOFDIRS; i++) {
        int newx = robotposeX + dX[i];
        int newy = robotposeY + dY[i];
        
        if (gridValid(newx, newy, x_size, y_size, map, collision_thresh) && !isInClosed(newx, newy, CLOSED)) {
            double f = map[GETMAPINDEX(newx, newy, x_size, y_size)] + 
                      euclidean_heuristic(newx, newy, goalposeX, goalposeY);
                    //   std::cout<<f << std::endl;
            OPEN.push(std::make_pair(f, std::make_pair(newx, newy)));
            std::cout << "OPEN: " << newx << ", " << newy << " " << euclidean_heuristic(newx, newy, goalposeX, goalposeY) << std::endl;

        }
    }

    // if on trajectory
    if (robotposeX == targetposeX && robotposeY == targetposeY) {
        action_ptr[0] = targetposeX;
        action_ptr[1] = targetposeY;
        return;
    } else {
        // Take the best neighbor
        action_ptr[0] = OPEN.top().second.first;
        action_ptr[1] = OPEN.top().second.second;
    }

    std::cout << "action_ptr: " << action_ptr[0] << ", " << action_ptr[1] << std::endl;
    CLOSED.push_back(std::make_pair(action_ptr[0], action_ptr[1]));


    return;
}

