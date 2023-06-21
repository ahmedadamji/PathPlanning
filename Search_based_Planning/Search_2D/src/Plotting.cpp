// In Plotting.cpp
#include <opencv2/opencv.hpp>
#include "Plotting.h"

Plotting::Plotting(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG, std::set<std::pair<int, int>> obs, int cell_size) {
    this->xI = xI;
    this->xG = xG;
    this->cell_size = cell_size;
    this->obs = obs;
    this->image = cv::Mat::zeros(x_range*cell_size, y_range*cell_size, CV_8UC3);
}

void Plotting::update_obs(std::set<std::pair<int, int>> new_obs) {
    obs = new_obs;
}

void Plotting::plot_grid() {
    for (auto const& obs_point : obs) {
        // obs_point.first*cell_size and obs_point.second*cell_size are the top-left corner of the rectangle that is being drawn.
        // This is where the rectangle begins. Multiplying by cell_size scales the coordinates to the size of the cells in the grid.
        // (obs_point.first+1)*cell_size - 2 and (obs_point.second+1)*cell_size - 2 are the bottom-right corner of the rectangle.
        // This is where the rectangle ends.
        // The "+1" is used to move to the next cell, and the "-2" is used to leave a little space between cells, forming a grid.
        cv::rectangle(image, 
                      cv::Point(obs_point.first*cell_size, obs_point.second*cell_size), 
                      cv::Point((obs_point.first+1)*cell_size - 2, (obs_point.second+1)*cell_size - 2), 
                      cv::Scalar(255, 255, 255), 
                      -1); // Obstacle color
    }
    cv::rectangle(image, 
                  cv::Point(xI.first*cell_size, xI.second*cell_size), 
                  cv::Point((xI.first+1)*cell_size - 2, (xI.second+1)*cell_size - 2), 
                  cv::Scalar(255, 0, 0), 
                  -1); // Start point color
    cv::rectangle(image, 
                  cv::Point(xG.first*cell_size, xG.second*cell_size), 
                  cv::Point((xG.first+1)*cell_size - 2, (xG.second+1)*cell_size - 2), 
                  cv::Scalar(0, 255, 0), 
                  -1); // End point color
}

void Plotting::plot_visited(std::set<std::pair<int, int>> visited) {
    for(auto const& visit_point : visited) {
        cv::rectangle(image, 
                      cv::Point(visit_point.first*cell_size, visit_point.second*cell_size), 
                      cv::Point((visit_point.first+1)*cell_size - 2, (visit_point.second+1)*cell_size - 2), 
                      cv::Scalar(120, 120, 120), 
                      -1); // Visited point color
    }
}

void Plotting::plot_path(std::set<std::pair<int, int>> path) {
    for(auto const& path_point : path) {
        cv::rectangle(image, 
                      cv::Point(path_point.first*cell_size, path_point.second*cell_size), 
                      cv::Point((path_point.first+1)*cell_size - 2, (path_point.second+1)*cell_size - 2), 
                      cv::Scalar(0, 0, 255), 
                      -1); // Path color
    }
}


void Plotting::show_image() {
    this->plot_grid();
    cv::namedWindow("Path", cv::WINDOW_NORMAL);  // Create window with freedom of resizing
    cv::imshow("Path", image);
    
}
