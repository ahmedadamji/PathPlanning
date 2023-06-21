#include "Dummy.h"
#include <opencv2/opencv.hpp>

void Dummy::run(Env &env, Plotting &plot) {
    // Create dummy visited points and path.
    std::set<std::pair<int, int>> visited;
    std::set<std::pair<int, int>> path;

    std::set<std::pair<int, int>> obstacles = env.get_obs();
    std::pair<int, int> xI = env.get_xI();
    std::pair<int, int> xG = env.get_xG();

    double expand_search = 1;

    for(int i = xI.first; i < xG.first; i++){
        if (i != xI.first) {
            expand_search_and_plot(i, xI.second, expand_search, visited, path, obstacles, xI, xG, plot, true);
        }
    }

    for(int i = xI.second; i < xG.second; i++){
        expand_search_and_plot(xG.first, i, expand_search, visited, path, obstacles, xI, xG, plot, false);
    }

    // Plot visited points and path.
    display_plots(plot, visited, path);
    cv::waitKey(0);
}

void Dummy::display_plots(Plotting &plot,
                   std::set<std::pair<int, int>> &visited,
                   std::set<std::pair<int, int>> &path)
{
    plot.plot_visited(visited);
    plot.plot_path(path);
    plot.show_image();
}

void Dummy::expand_search_and_plot(int x, int y, double &expand_search, 
                                std::set<std::pair<int, int>> &visited, 
                                std::set<std::pair<int, int>> &path,
                                std::set<std::pair<int, int>> &obstacles, 
                                std::pair<int, int> &xI, 
                                std::pair<int, int> &xG, 
                                Plotting &plot,
                                bool is_horizontal)
{
    std::pair<int, int> point = {x, y};

    for (int dx = -int(expand_search); dx <= int(expand_search); dx++) {
        for (int dy = -int(expand_search); dy <= int(expand_search); dy++) {
            int nx = point.first + (is_horizontal ? 0 : dx);
            int ny = point.second + (is_horizontal ? dy : 0);
            std::pair<int, int> visited_point = {nx, ny};
            if((obstacles.find(visited_point) == obstacles.end())
                && ((visited_point != xI )&& (visited_point != xG ))){
                visited.insert(visited_point);
            }
        }
    }

    if (obstacles.find(point) == obstacles.end()) {
        path.insert(point);
        display_plots(plot, visited, path);
        cv::waitKey(100); // Pause for a short time
    }

    expand_search += 0.4;
}
