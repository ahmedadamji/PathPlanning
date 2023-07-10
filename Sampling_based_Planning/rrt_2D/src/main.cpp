#include "Plotting.h"
#include "Env.h"
#include "AStar.h"
#include <opencv2/opencv.hpp>

int main()
{
    std::pair<int, int> xI = {5, 5}; // Define xI directly.
    std::pair<int, int> xG = {35, 25}; // Define xG directly.

    // Initialize environment with its range and starting and ending points.
    Env env(51, 31, xI, xG);

    // Setting the cell size for plotting.
    int cell_size = 15;
    
    // Initialize a Plotting object.
    Plotting plot(env, cell_size);

    // Update the obstacle set in the Plotting object.
    plot.update_obs(env.get_obs());

    // Plot grid.
    plot.plot_grid();

    // Initialize and run the A* algorithm.
    // AStar astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    AStar astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    AStar::PointVectorPointSetPair pathVisitedPair = astar.searching();

    return 0;
}
