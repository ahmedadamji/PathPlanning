#ifndef PLOTTING_H
#define PLOTTING_H

#include <opencv2/opencv.hpp>
#include <set>
#include <vector>

class Plotting {
public:
    Plotting(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG, std::set<std::pair<int, int>> obs, int cell_size);
    void update_obs(std::set<std::pair<int, int>> new_obs);
    void plot_grid();
    void plot_visited(std::set<std::pair<int, int>> visited);
    void plot_path(std::set<std::pair<int, int>> path);
    void show_image();
    
    std::pair<int, int> xI, xG;
    std::set<std::pair<int, int>> obs;
    int cell_size;  // Size of each cell in pixels.
    cv::Mat image;
};

#endif  // PLOTTING_H
