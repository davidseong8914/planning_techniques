/*
 * config_finder.cpp
 * 
 * This program finds valid arm configurations for a given map by testing
 * many different combinations of joint angles and checking if they are valid.
 * 
 * Compile with: g++ -std=c++17 config_finder.cpp -o config_finder.out
 * Run with: ./config_finder.out <map_file> <num_DOFs> <num_configs_to_find>
 */

// g++ -std=c++17 config_finder.cpp -o config_finder.out
// ./config_finder.out map2.txt 5 10

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <tuple>
#include <regex>
#include <chrono>
#include <set>
#include <iomanip>

// Constants
#define PI 3.141592654
#define LINKLENGTH_CELLS 10
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

// Bresenham algorithm parameters - define this first before function declarations
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

// Function prototypes from planner.cpp
void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);
void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);
void get_current_point(bresenham_param_t *params, int *x, int *y);
int get_next_point(bresenham_param_t *params);
int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map, int x_size, int y_size);
int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size);

// Load map from file
std::tuple<double*, int, int> loadMap(std::string filepath) {
    std::FILE *f = fopen(filepath.c_str(), "r");
    if (!f) {
        printf("Opening file failed! \n");
        throw std::runtime_error("Opening map file failed!");
    }
    int height, width;
    if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
        throw std::runtime_error("Invalid loadMap parsing map metadata");
    }
    
    double* map = new double[height*width];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            char c;
            do {
                if (fscanf(f, "%c", &c) != 1) {
                    throw std::runtime_error("Invalid parsing individual map data");
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
    return std::make_tuple(map, width, height);
}

// Generate a random arm configuration
std::vector<double> generateRandomConfig(int numofDOFs, std::mt19937& gen) {
    std::uniform_real_distribution<> dis(-PI, PI);
    std::vector<double> config(numofDOFs);
    for (int i = 0; i < numofDOFs; i++) {
        config[i] = dis(gen);
    }
    return config;
}

// Check if configuration pair (start and goal) is valid
bool isValidConfigPair(double* map, int x_size, int y_size, 
                      const std::vector<double>& start, 
                      const std::vector<double>& goal, 
                      int numofDOFs) {
    // Convert vectors to arrays for IsValidArmConfiguration
    double* startArr = new double[numofDOFs];
    double* goalArr = new double[numofDOFs];
    
    for (int i = 0; i < numofDOFs; i++) {
        startArr[i] = start[i];
        goalArr[i] = goal[i];
    }
    
    bool isValid = IsValidArmConfiguration(startArr, numofDOFs, map, x_size, y_size) && 
                  IsValidArmConfiguration(goalArr, numofDOFs, map, x_size, y_size);
    
    delete[] startArr;
    delete[] goalArr;
    
    return isValid;
}

// Format configuration as string
std::string formatConfig(const std::vector<double>& config) {
    std::stringstream ss;
    for (size_t i = 0; i < config.size(); i++) {
        ss << std::fixed << std::setprecision(2) << config[i];
        if (i < config.size() - 1) {
            ss << ",";
        }
    }
    return ss.str();
}

// Calculate Euclidean distance between configurations
double configDistance(const std::vector<double>& config1, const std::vector<double>& config2) {
    double sum = 0.0;
    for (size_t i = 0; i < config1.size(); i++) {
        double diff = config1[i] - config2[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// Create a diverse set of configurations by ensuring they're different enough
std::vector<std::vector<double>> generateDiverseConfigs(
    double* map, int x_size, int y_size, int numofDOFs, int numToFind) {
    
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Store valid configurations
    std::vector<std::vector<double>> validConfigs;
    
    // Distance threshold for diversity
    const double minDistance = 0.5;
    
    // Maximum attempts
    const int maxAttempts = 100000;
    int attempts = 0;
    
    std::cout << "Searching for " << numToFind << " valid configurations..." << std::endl;
    
    while (validConfigs.size() < numToFind && attempts < maxAttempts) {
        attempts++;
        
        if (attempts % 1000 == 0) {
            std::cout << "Attempts: " << attempts << ", Found: " << validConfigs.size() << std::endl;
        }
        
        // Generate random configuration
        std::vector<double> config = generateRandomConfig(numofDOFs, gen);
        
        // Skip if too similar to existing configs
        bool tooSimilar = false;
        for (const auto& existingConfig : validConfigs) {
            if (configDistance(config, existingConfig) < minDistance) {
                tooSimilar = true;
                break;
            }
        }
        if (tooSimilar) continue;
        
        // Check if valid
        double* configArr = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; i++) {
            configArr[i] = config[i];
        }
        
        if (IsValidArmConfiguration(configArr, numofDOFs, map, x_size, y_size)) {
            validConfigs.push_back(config);
            std::cout << "Found valid config #" << validConfigs.size() << ": " 
                      << formatConfig(config) << std::endl;
        }
        
        delete[] configArr;
    }
    
    if (validConfigs.size() < numToFind) {
        std::cout << "Warning: Only found " << validConfigs.size() 
                  << " valid configurations after " << attempts << " attempts." << std::endl;
    }
    
    return validConfigs;
}

// Main function
int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_DOFs> <num_configs_to_find>" << std::endl;
        return 1;
    }
    
    std::string mapFile = argv[1];
    int numofDOFs = std::stoi(argv[2]);
    int numToFind = std::stoi(argv[3]);
    
    std::cout << "Loading map from " << mapFile << "..." << std::endl;
    
    // Load map
    double* map;
    int x_size, y_size;
    std::tie(map, x_size, y_size) = loadMap(mapFile);
    
    std::cout << "Map loaded successfully. Size: " << x_size << "x" << y_size << std::endl;
    
    // Find valid configurations
    auto validConfigs = generateDiverseConfigs(map, x_size, y_size, numofDOFs, numToFind);
    
    // Generate valid configuration pairs
    std::cout << "\nGenerating valid configuration pairs:" << std::endl;
    
    int numPairs = std::min(5, (int)validConfigs.size() / 2);
    
    for (int i = 0; i < numPairs; i++) {
        int idx1 = i;
        int idx2 = validConfigs.size() - 1 - i;
        
        std::string start = formatConfig(validConfigs[idx1]);
        std::string goal = formatConfig(validConfigs[idx2]);
        
        std::cout << "\nTest Case " << (i+1) << ":" << std::endl;
        std::cout << "Start: " << start << std::endl;
        std::cout << "Goal: " << goal << std::endl;
        
        // For each planner type (0-3)
        for (int planner = 0; planner <= 3; planner++) {
            std::string cmd = "./planner.out " + mapFile + " " + std::to_string(numofDOFs) + 
                             " " + start + " " + goal + " " + std::to_string(planner) + 
                             " output_" + std::to_string(i+1) + "_planner" + std::to_string(planner) + ".txt";
            
            std::cout << "Command (Planner " << planner << "): " << cmd << std::endl;
        }
    }
    
    // Clean up
    delete[] map;
    
    return 0;
}

// Implementation of functions from planner.cpp
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

int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map, int x_size, int y_size) {
    bresenham_param_t params;
    int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;
    
    //make sure the line segment is inside the environment
    if(x0 < 0 || x0 >= x_size ||
        x1 < 0 || x1 >= x_size ||
        y0 < 0 || y0 >= y_size ||
        y1 < 0 || y1 >= y_size)
        return 0;

    ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
    ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //iterate through the points on the segment
    get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
    do {
        get_current_point(&params, &nX, &nY);
        if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
    } while (get_next_point(&params));

    return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size) {
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