// In Plotting.cpp
#include <opencv2/opencv.hpp>
#include "Plotting.h"

Plotting::Plotting(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG, std::set<std::pair<int, int>> obs, int cell_size)
{
    this->xI = xI;
    this->xG = xG;
    this->cell_size = cell_size;
    this->obs = obs;
    this->image = cv::Mat::zeros(x_range * cell_size, y_range * cell_size, CV_8UC3);
}

void Plotting::update_obs(std::set<std::pair<int, int>> new_obs)
{
    obs = new_obs;
}

void Plotting::plot_grid()
{
    for (auto const &obs_point : obs)
    {
        // obs_point.first*cell_size and obs_point.second*cell_size are the top-left corner of the rectangle that is being drawn.
        // This is where the rectangle begins. Multiplying by cell_size scales the coordinates to the size of the cells in the grid.
        // (obs_point.first+1)*cell_size - 2 and (obs_point.second+1)*cell_size - 2 are the bottom-right corner of the rectangle.
        // This is where the rectangle ends.
        // The "+1" is used to move to the next cell, and the "-2" is used to leave a little space between cells, forming a grid.
        cv::rectangle(image,
                      cv::Point(obs_point.first * cell_size, obs_point.second * cell_size),
                      cv::Point((obs_point.first + 1) * cell_size - 2, (obs_point.second + 1) * cell_size - 2),
                      cv::Scalar(255, 255, 255),
                      -1); // Obstacle color
    }
    cv::rectangle(image,
                  cv::Point(xI.first * cell_size, xI.second * cell_size),
                  cv::Point((xI.first + 1) * cell_size - 2, (xI.second + 1) * cell_size - 2),
                  cv::Scalar(255, 0, 0),
                  -1); // Start point color
    cv::rectangle(image,
                  cv::Point(xG.first * cell_size, xG.second * cell_size),
                  cv::Point((xG.first + 1) * cell_size - 2, (xG.second + 1) * cell_size - 2),
                  cv::Scalar(0, 255, 0),
                  -1); // End point color
}

void Plotting::plot_visited(std::set<std::pair<int, int>> visited)
{
    for (auto const &visit_point : visited)
    {
        cv::rectangle(image,
                      cv::Point(visit_point.first * cell_size, visit_point.second * cell_size),
                      cv::Point((visit_point.first + 1) * cell_size - 2, (visit_point.second + 1) * cell_size - 2),
                      cv::Scalar(120, 120, 120),
                      -1); // Visited point color
    }
}

void Plotting::plot_visited(std::set<std::pair<int, int>> visited, cv::Scalar color)
{
    for (auto const &visit_point : visited)
    {
        cv::rectangle(image,
                      cv::Point(visit_point.first * cell_size, visit_point.second * cell_size),
                      cv::Point((visit_point.first + 1) * cell_size - 2, (visit_point.second + 1) * cell_size - 2),
                      color,
                      -1); // Visited point color
    }
}

void Plotting::plot_path(std::vector<std::pair<int, int>> path)
{
    std::reverse(path.begin(), path.end()); // Reverse the path
    std::pair<int, int> last_point = path[0];
    for (auto const &path_point : path)
    {
        cv::line(image,
                 cv::Point(last_point.first * cell_size + cell_size / 2, last_point.second * cell_size + cell_size / 2),
                 cv::Point(path_point.first * cell_size + cell_size / 2, path_point.second * cell_size + cell_size / 2),
                 cv::Scalar(0, 0, 255),
                 3); // Path color
        last_point = path_point;
    }
}

void Plotting::plot_path(std::vector<std::pair<int, int>> path, cv::Scalar color)
{
    std::reverse(path.begin(), path.end()); // Reverse the path
    std::pair<int, int> last_point = path[0];
    for (auto const &path_point : path)
    {
        cv::line(image,
                 cv::Point(last_point.first * cell_size + cell_size / 2, last_point.second * cell_size + cell_size / 2),
                 cv::Point(path_point.first * cell_size + cell_size / 2, path_point.second * cell_size + cell_size / 2),
                 color,
                 3); // Path color
        last_point = path_point;
    }
}

void Plotting::show_image(std::string windowName)
{
    this->plot_grid();
    cv::namedWindow(windowName, cv::WINDOW_NORMAL); // Create window with freedom of resizing
    cv::imshow(windowName, image);
}

void Plotting::plot_animation(std::string windowName, std::set<std::pair<int, int>> visited, std::vector<std::pair<int, int>> path)
{

    this->plot_visited(visited);

    if (!path.empty())
    {
        // for (const auto& path_step : path) {
        //     if (path_step == xG) {
        //         break;
        //     }
        //     this->plot_path({path_step});
        //     this->show_image(windowName);
        //     cv::waitKey(12);
        // }
        this->plot_path(path);
        this->show_image(windowName);
        cv::waitKey(0);
    }
    else
    {
        this->show_image(windowName);
        cv::waitKey(1);
    }
}

void Plotting::plot_animation_repeated_astar(std::string windowName, std::set<std::pair<int, int>> visited, std::vector<std::pair<int, int>> path, int repeated_count, bool is_last)
{

    this->plot_visited(visited, this->colorListV()[repeated_count]);

    if (!path.empty())
    {
        // for (const auto& path_step : path) {
        //     if (path_step == xG) {
        //         break;
        //     }
        //     this->plot_path({path_step}, this->colorListP()[repeated_count]);
        //     this->show_image(windowName);
        //     cv::waitKey(12);
        // }
        this->plot_path(path, this->colorListP()[repeated_count]);
        if (is_last)
        {
            this->show_image(windowName);
            cv::waitKey(0);
        }
        else
        {
            this->show_image(windowName);
            cv::waitKey(25);
        }
    }
    else
    {
        this->show_image(windowName);
        cv::waitKey(2);
    }
}

void Plotting::plot_animation_bidirectional_astar(std::string windowName, std::set<std::pair<int, int>> visited, std::vector<std::pair<int, int>> path, bool is_forward)
{

    this->plot_visited(visited, this->colorListV()[is_forward]);

    if ((!path.empty()) && !is_forward)
    {
        this->plot_path(path);
        this->show_image(windowName);
        cv::waitKey(0);
    }
    else
    {
        this->show_image(windowName);
        cv::waitKey(2);
    }
}

std::vector<cv::Scalar> Plotting::colorListV()
{
    std::vector<cv::Scalar> cl_v;
    cl_v.push_back(cv::Scalar(255, 255, 0));   // yellow
    cl_v.push_back(cv::Scalar(0, 255, 255));   // cyan
    cl_v.push_back(cv::Scalar(225, 105, 65));  // royalblue
    cl_v.push_back(cv::Scalar(70, 130, 180));  // steelblue
    cl_v.push_back(cv::Scalar(255, 250, 205)); // lemonchiffon
    cl_v.push_back(cv::Scalar(255, 255, 255)); // white
    cl_v.push_back(cv::Scalar(192, 192, 192)); // silver
    cl_v.push_back(cv::Scalar(250, 206, 235)); // lightskyblue
    cl_v.push_back(cv::Scalar(179, 222, 245)); // wheat
    cl_v.push_back(cv::Scalar(112, 128, 144)); // slategray
    cl_v.push_back(cv::Scalar(240, 248, 255)); // aliceblue
    cl_v.push_back(cv::Scalar(250, 240, 230)); // floralwhite
    cl_v.push_back(cv::Scalar(245, 245, 220)); // beige
    cl_v.push_back(cv::Scalar(248, 248, 255)); // ghostwhite
    cl_v.push_back(cv::Scalar(255, 250, 240)); // ivory
    cl_v.push_back(cv::Scalar(245, 255, 250)); // mintcream
    return cl_v;
}



std::vector<cv::Scalar> Plotting::colorListP()
{
    std::vector<cv::Scalar> cl_p;
    cl_p.push_back(cv::Scalar(0, 0, 0));       // black
    cl_p.push_back(cv::Scalar(255, 0, 0));     // blue
    cl_p.push_back(cv::Scalar(0, 0, 255));     // red
    cl_p.push_back(cv::Scalar(0, 255, 0));     // green
    cl_p.push_back(cv::Scalar(0, 165, 255));   // orange
    cl_p.push_back(cv::Scalar(255, 191, 0));   // deepskyblue
    cl_p.push_back(cv::Scalar(128, 128, 128)); // gray
    cl_p.push_back(cv::Scalar(255, 0, 255));   // magenta
    cl_p.push_back(cv::Scalar(0, 255, 255));   // cyan
    cl_p.push_back(cv::Scalar(105, 105, 105)); // dimgray
    cl_p.push_back(cv::Scalar(25, 25, 112));   // midnightblue
    cl_p.push_back(cv::Scalar(85, 107, 47));   // darkolivegreen
    cl_p.push_back(cv::Scalar(139, 0, 0));     // darkred
    cl_p.push_back(cv::Scalar(0, 100, 0));     // darkgreen
    cl_p.push_back(cv::Scalar(139, 0, 139));   // darkmagenta
    cl_p.push_back(cv::Scalar(0, 139, 139));   // darkcyan
    cl_p.push_back(cv::Scalar(70, 130, 180));  // steelblue
    return cl_p;
}

