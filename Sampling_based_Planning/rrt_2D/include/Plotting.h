#ifndef PLOTTING_H
#define PLOTTING_H

#include <opencv2/opencv.hpp>
#include <set>
#include <vector>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <Magick++.h>

#include "Env.h"
#include "Utils.h"
#include "Node.h"

class Plotting {
public:
    /**
     * @brief Construct a new Plotting object
     * 
     * @param env
     * @param cell_size 
     */
    Plotting(Env &env, Utils &utils, int cell_size);
    /**
     * @brief Update the obstacle set
     * 
     */
    void update_obs();
    /**
     * @brief Plot the grid
     * 
     */
    void plot_grid();

    /**
     * @brief Plotting the path for rrt
     * 
     * @param nodelist the path
     */
    void plot_visited(const std::vector<std::shared_ptr<Node>>& nodelist);
    
    /**
     * @brief Plotting the path for rrt to visualize nodes from two different vectors V1 and V2.
     * 
     * @param V1 The first vector of nodes
     * @param V2 The second vector of nodes
     */
    void plot_visited_connect(const std::vector<std::shared_ptr<Node>> &V1, const std::vector<std::shared_ptr<Node>> &V2);

    
    /**
     * @brief Plot the path
     * 
     * @param path 
     */
    void plot_path(const std::vector<std::pair<double, double>>& path);

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
     * @param nodelist 
     * @param path 
     */
    void plot_animation(std::string windowName, const std::vector<std::shared_ptr<Node>> &nodelist, const std::vector<std::pair<double, double>> &path);

    /**
     * @brief Plot the path and visited nodes for rrt to visualize nodes from two different vectors V1 and V2.
     * 
     * @param windowName 
     * @param V1 
     * @param V2 
     * @param path 
     */
    void plot_animation_connect(std::string windowName, const std::vector<std::shared_ptr<Node>> &V1, const std::vector<std::shared_ptr<Node>> &V2, const std::vector<std::pair<double, double>> &path);

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
    
    /**
     * @brief Show the image
     * 
     * @param windowName 
     */
    void imageShow(std::string windowName);
    
    /**
     * @brief Save the recorded frames as a gif
     * 
     * @param filename 
     */
    void save_as_gif(const std::string& filename);

    
    std::pair<int, int> xI, xG;
    std::set<std::pair<int, int>> obs;
    std::vector<std::vector<int>> obs_boundary;
    std::vector<std::vector<int>> obs_rectangle;
    std::vector<std::vector<int>> obs_circle;
    int cell_size;  // Size of each cell in pixels.
    cv::Mat image; // The image to be plotted.
    std::vector<cv::Mat> frames; // to store the frames for saving as a video

    cv::Point clicked_point; // to store clicked pixel's coordinates
    Env _env;
    Utils _utils;
    bool firstClickDone = {};

    std::vector<std::pair<double, double>> _path;


private:

    static std::atomic<bool> stopLoop;
    
    struct CallbackData {
        Plotting* plotting;
        std::string windowName;
    };


};

#endif  // PLOTTING_H
