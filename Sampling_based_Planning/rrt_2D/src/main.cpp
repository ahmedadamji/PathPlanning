#include "Plotting.h"
#include "Env.h"
#include "Utils.h"
#include "Node.h"
#include "RRT.h"
#include "RRT_Connect.h"
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

    // // Initialize and run the RRT algorithm.
    // double step_len, goal_sample_rate, iter_max;
    // step_len = 0.5;
    // goal_sample_rate = 0.05;
    // iter_max = 10000;
    // RRT rrt(env, plot, utils, env.get_xI(), env.get_xG(), step_len, goal_sample_rate, iter_max);
    // RRT::PointVector path = rrt.Planning();

    // Initialize and run the RRT algorithm.
    double step_len, goal_sample_rate, iter_max;
    step_len = 0.8;
    goal_sample_rate = 0.05;
    iter_max = 5000;
    RRT_Connect rrt_connect(env, plot, utils, env.get_xI(), env.get_xG(), step_len, goal_sample_rate, iter_max);
    RRT_Connect::PointVector path = rrt_connect.Planning();
    // Save the recorded frames as a GIF with a delay of 1 milliseconds between frames.
    plot.save_as_gif("rrt_connect.gif", 1);


    return 0;
}
