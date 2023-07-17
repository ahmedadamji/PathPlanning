#include "Plotting.h"
#include "Env.h"
#include "Utils.h"
#include "Node.h"
#include "RRT.h"
#include <opencv2/opencv.hpp>

int main()
{
    std::pair<int, int> xI = {2, 2}; // Define xI directly.
    std::pair<int, int> xG = {49, 24}; // Define xG directly.

    // Initialize environment with its range and starting and ending points.
    Env env(51, 31, xI, xG);

    // Initialize a Utils object.
    Utils utils(env);

    // Setting the cell size for plotting.
    int cell_size = 15;
    
    // Initialize a Plotting object.
    Plotting plot(env, utils, cell_size);

    // Update the obstacle set in the Plotting object.
    plot.update_obs();

    // Plot grid.
    plot.plot_grid();

    // // Show the plot.
    // plot.show_image("Sample");
    // cv::waitKey(0);

    // Initialize and run the RRT algorithm.
    RRT rrt(env, plot, utils, env.get_xI(), env.get_xG(), 0.5, 0.05, 10000);
    RRT::PointVector path = rrt.Planning();


    return 0;
}
