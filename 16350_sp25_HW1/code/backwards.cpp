#include "planner.h"
#include <math.h>
#include <iostream>
#include <queue>
#include <vector>
#include "planner.h"
#define NUMOFDIRS 8

void backwards(
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
    ) {

        std::cout << map;
    
}