#ifndef PLOTTING_H
#define PLOTTING_H

#include <opencv2/opencv.hpp>
#include <set>
#include <vector>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include "Env.h"

class Plotting {
public:
    /**
     * @brief Construct a new Plotting object
     * 
     * @param env
     * @param cell_size 
     */
    Plotting(Env &env, int cell_size);
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
    void plot_path(std::vector<std::pair<int, int>> path);
    
    /**
     * @brief Plot the path with specified colors
     * 
     * @param path 
     * @param color 
     */
    void plot_path(std::vector<std::pair<int, int>> path, cv::Scalar color);

    /**
     * @brief Show the image
     * 
     * @param windowName
     */
    void show_image(std::string windowName);
    
    /**
     * @brief Plot the path and visited nodes
     * 
     * @param windowName 
     * @param path 
     * @param visited 
     */
    void plot_animation(std::string windowName, std::set<std::pair<int, int>> visited, std::vector<std::pair<int, int>> path);

    /**
     * @brief Plot the path and visited nodes for repeated A*
     * 
     * @param windowName 
     * @param path 
     * @param visited 
     * @param repeated_count 
     * @param is_last 
     */
    void plot_animation_repeated_astar(std::string windowName, std::set<std::pair<int, int>> visited, std::vector<std::pair<int, int>> path, int repeated_count, bool is_last);

    /**
     * @brief Plot the path and visited nodes for bidirectional A*
     * 
     * @param windowName 
     * @param path 
     * @param visited 
     * @param is_forward 
     */
    void plot_animation_bidirectional_astar(std::string windowName, std::set<std::pair<int, int>> visited, std::vector<std::pair<int, int>> path, bool is_forward);
    
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
     * @brief Get the clicked point
     * 
     * @param event 
     * @param x 
     * @param y 
     * @param userdata 
     */
    static void mouse_callback(int event, int x, int y, int, void* userdata);
    
    /**
     * @brief Get the click coordinates object
     * 
     * @param windowName 
     * @return cv::Point 
     */
    cv::Point get_click_coordinates(std::string windowName);
    
    /**
     * @brief Check for input
     * 
     * This function checks for input from the user.
     * 
     */
    void checkForInput();

    
    std::pair<int, int> xI, xG;
    std::set<std::pair<int, int>> obs;
    std::vector<std::vector<int>> obs_boundary;
    std::vector<std::vector<int>> obs_rectangle;
    std::vector<std::vector<int>> obs_circle;
    int cell_size;  // Size of each cell in pixels.
    cv::Mat image;

    cv::Point clicked_point; // to store clicked pixel's coordinates
    Env _env;
    bool firstClickDone = {};

    std::vector<std::pair<int, int>> _path;


private:

    static std::atomic<bool> stopLoop;
    
    struct CallbackData {
        Plotting* plotting;
        std::string windowName;
    };


};

#endif  // PLOTTING_H
