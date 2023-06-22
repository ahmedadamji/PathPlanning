#ifndef PLOTTING_H
#define PLOTTING_H

#include <opencv2/opencv.hpp>
#include <set>
#include <vector>

class Plotting {
public:
    /**
     * @brief Construct a new Plotting object
     * 
     * @param x_range 
     * @param y_range 
     * @param xI 
     * @param xG 
     * @param obs 
     * @param cell_size 
     */
    Plotting(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG, std::set<std::pair<int, int>> obs, int cell_size);
    /**
     * @brief Update the obstacle set
     * 
     * @param new_obs 
     */
    void update_obs(std::set<std::pair<int, int>> new_obs);
    /**
     * @brief Plot the grid
     * 
     */
    void plot_grid();
    /**
     * @brief Plot the visited nodes
     * 
     * @param visited 
     */
    void plot_visited(std::set<std::pair<int, int>> visited);
    /**
     * @brief Plot the visited nodes with specified colors
     * 
     * @param visited 
     * @param color 
     */
    void plot_visited(std::set<std::pair<int, int>> visited, cv::Scalar color);
    /**
     * @brief Plot the path
     * 
     * @param path 
     */
    void plot_path(std::set<std::pair<int, int>> path);
    
    /**
     * @brief Plot the path with specified colors
     * 
     * @param path 
     * @param color 
     */
    void plot_path(std::set<std::pair<int, int>> path, cv::Scalar color);

    /**
     * @brief Get a vector of BGR colors (as cv::Scalar) based on a pre-defined list.
     *
     * The function returns a vector containing BGR color values (as cv::Scalar objects)
     * corresponding to the following color names: silver, wheat, lightskyblue, royalblue, slategray.
     *
     * @return std::vector<cv::Scalar> Vector of BGR colors as cv::Scalar objects.
     */
    static std::vector<cv::Scalar> colorListV();

    /**
     * @brief Get a vector of BGR colors (as cv::Scalar) based on a pre-defined list.
     *
     * The function returns a vector containing BGR color values (as cv::Scalar objects)
     * corresponding to the following color names: gray, orange, deepskyblue, red, magenta.
     *
     * @return std::vector<cv::Scalar> Vector of BGR colors as cv::Scalar objects.
     */
    static std::vector<cv::Scalar> colorListP();

    /**
     * @brief Show the image
     * 
     */
    void show_image();
    
    std::pair<int, int> xI, xG;
    std::set<std::pair<int, int>> obs;
    int cell_size;  // Size of each cell in pixels.
    cv::Mat image;
};

#endif  // PLOTTING_H
