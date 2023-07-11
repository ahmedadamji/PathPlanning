// In Plotting.cpp
#include <opencv2/opencv.hpp>
#include "Plotting.h"

Plotting::Plotting(Env &env, Utils &utils, int cell_size): _env(env), _utils(utils)
{
    this->xI = env.get_xI();
    this->xG = env.get_xG();
    this->cell_size = cell_size;
    this->obs_boundary = env.get_obs_boundary();
    this->obs_rectangle = env.get_obs_rectangle();
    this->obs_circle = env.get_obs_circle();
    this->image = cv::Mat::zeros(env.y_range * cell_size, env.x_range * cell_size, CV_8UC3);
}

void Plotting::update_obs()
{
    this->obs_boundary = this->_env.get_obs_boundary();
    this->obs_rectangle = this->_env.get_obs_rectangle();
    this->obs_circle = this->_env.get_obs_circle();
}

void Plotting::plot_grid()
{

    // The -2 is used to leave a little space between cells, forming a grid.
    // Plotting the Boundary of the environment
    for (auto const &obs_boundary_point : this->obs_boundary)
    {
        // The -2 is not used here because it is a boundary so doesnt have to look like a grid.
        // Each obs_boundary_point contains [ox, oy, width, height]
        cv::rectangle(image,
                      cv::Point(obs_boundary_point[0] * cell_size, obs_boundary_point[1] * cell_size),
                      cv::Point((obs_boundary_point[0] + obs_boundary_point[2]) * cell_size, (obs_boundary_point[1] + obs_boundary_point[3]) * cell_size),
                      cv::Scalar(255, 255, 255),
                      -1); // Obstacle color
    }

    // Plotting the Rectangles of the environment
    for (auto const &obs_rectangle_point : this->obs_rectangle)
    {
        // The -2 is not used here because it is a boundary so doesnt have to look like a grid.
        // Each obs_rectangle_point contains [ox, oy, width, height]
        cv::rectangle(image,
                      cv::Point(obs_rectangle_point[0] * cell_size, obs_rectangle_point[1] * cell_size),
                      cv::Point((obs_rectangle_point[0] + obs_rectangle_point[2]) * cell_size, (obs_rectangle_point[1] + obs_rectangle_point[3]) * cell_size),
                      cv::Scalar(256, 256, 256),
                      3, // Thickness of the boundary
                      cv::LINE_AA); // Anti-aliased boundary
        cv::rectangle(image,
                        cv::Point(obs_rectangle_point[0] * cell_size, obs_rectangle_point[1] * cell_size),
                        cv::Point((obs_rectangle_point[0] + obs_rectangle_point[2]) * cell_size, (obs_rectangle_point[1] + obs_rectangle_point[3]) * cell_size),
                        cv::Scalar(128, 128, 128),
                        -1); // Obstacle color
    }

    // Plotting the Circles of the environment
    for (auto const &obs_circle_point : this->obs_circle)
    {
        // The -2 is not used here because it is a boundary so doesnt have to look like a grid.
        // Each obs_circle_point contains [ox, oy, radius]
        cv::circle(image,
                    cv::Point(obs_circle_point[0] * cell_size, obs_circle_point[1] * cell_size),
                    obs_circle_point[2] * cell_size,
                    cv::Scalar(256, 256, 256),
                    3, // Thickness of the boundary
                    cv::LINE_AA); // Anti-aliased boundary
        cv::circle(image,
                    cv::Point(obs_circle_point[0] * cell_size, obs_circle_point[1] * cell_size),
                    obs_circle_point[2] * cell_size,
                    cv::Scalar(128, 128, 128),
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

void Plotting::plot_path(std::vector<std::pair<int, int>> path)
{
     _path = path;
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
    _path = path;
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
        this->plot_path(path);
        this->show_image(windowName);
        // cv::Point click_coordinates = this->get_click_coordinates(windowName);
        cv::waitKey(0);
    }
    else
    {
        this->show_image(windowName);
        cv::waitKey(1);
    }
}

std::vector<cv::Scalar> Plotting::colorListV()
{
    std::vector<cv::Scalar> cl_v;
    cl_v.push_back(cv::Scalar(255, 255, 153));   // light yellow
    cl_v.push_back(cv::Scalar(153, 255, 255));   // light cyan
    cl_v.push_back(cv::Scalar(255, 184, 134));   // light royalblue
    cl_v.push_back(cv::Scalar(147, 183, 208));   // light steelblue
    cl_v.push_back(cv::Scalar(255, 204, 102));   // light orange
    cl_v.push_back(cv::Scalar(153, 255, 153));   // light lime
    cl_v.push_back(cv::Scalar(224, 224, 224));   // light silver
    cl_v.push_back(cv::Scalar(177, 109, 236));   // light blueviolet
    cl_v.push_back(cv::Scalar(102, 204, 102));   // light forestgreen
    cl_v.push_back(cv::Scalar(154, 169, 179));   // light slategray
    cl_v.push_back(cv::Scalar(102, 102, 255));   // light red
    cl_v.push_back(cv::Scalar(179, 102, 179));   // light purple
    cl_v.push_back(cv::Scalar(102, 179, 102));   // light green
    cl_v.push_back(cv::Scalar(102, 102, 179));   // light darkblue
    cl_v.push_back(cv::Scalar(179, 102, 102));   // light maroon
    cl_v.push_back(cv::Scalar(102, 179, 179));   // light teal
    cl_v.push_back(cv::Scalar(255, 192, 203));   // light pink
    cl_v.push_back(cv::Scalar(245, 222, 179));   // light wheat
    cl_v.push_back(cv::Scalar(240, 248, 255));   // light aliceblue
    cl_v.push_back(cv::Scalar(240, 230, 140));   // light khaki
    cl_v.push_back(cv::Scalar(250, 250, 210));   // light lemonchiffon
    cl_v.push_back(cv::Scalar(176, 224, 230));   // light powderblue
    cl_v.push_back(cv::Scalar(255, 228, 181));   // light moccasin
    cl_v.push_back(cv::Scalar(238, 232, 170));   // light palegoldenrod
    cl_v.push_back(cv::Scalar(230, 230, 250));   // light lavender
    cl_v.push_back(cv::Scalar(255, 239, 213));   // light papayawhip
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
    cl_p.push_back(cv::Scalar(72, 61, 139));     // darkslateblue
    cl_p.push_back(cv::Scalar(85, 26, 139));     // purple4
    cl_p.push_back(cv::Scalar(188, 143, 143));   // rosybrown
    cl_p.push_back(cv::Scalar(255, 165, 0));     // orange
    cl_p.push_back(cv::Scalar(199, 21, 133));    // mediumvioletred
    cl_p.push_back(cv::Scalar(100, 149, 237));   // cornflowerblue
    cl_p.push_back(cv::Scalar(255, 215, 0));     // gold
    cl_p.push_back(cv::Scalar(30, 144, 255));    // dodgerblue
    cl_p.push_back(cv::Scalar(153, 50, 204));    // darkorchid
    cl_p.push_back(cv::Scalar(143, 188, 143));   // darkseagreen
    return cl_p;
}


void Plotting::mouse_callback(int event, int x, int y, int, void* userdata) {
    CallbackData* data = static_cast<CallbackData*>(userdata);
    Plotting* plotting = data->plotting;
    std::string windowName = data->windowName;

    // If the first click has already been processed, just return.
    if (plotting->firstClickDone) {
        return;
    }

    if (event == cv::EVENT_LBUTTONDOWN) {
        // Convert the clicked point to the corresponding cell in the grid.
        int cell_x = x / plotting->cell_size;
        int cell_y = y / plotting->cell_size;
        plotting->clicked_point = cv::Point(cell_x, cell_y);
        std::cout << "Clicked cell: (" << cell_x << ", " << cell_y << ")" << std::endl;

        // Set the flag to true after processing the first click.
        plotting->firstClickDone = true;
    }
}

cv::Point Plotting::get_click_coordinates(std::string windowName) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL); // Create window with freedom of resizing

    CallbackData data;
    data.plotting = this;
    data.windowName = windowName;

    // // Start a thread which listens to keyboard input
    // std::thread inputThread(&Plotting::checkForInput, this);

    // while (!this->stopLoop)
    // {
    this->firstClickDone = false; // Reset the flag before starting the callback

    cv::setMouseCallback(windowName, &Plotting::mouse_callback, &data);

    while (!this->firstClickDone) {
        cv::imshow(windowName, image);
        cv::waitKey(1);

        if (this->stopLoop) {
            break;
        }
    }
    
    // Check if the clicked coordinates are part of the obstacle space.
    std::pair<int, int> coordinates = std::make_pair(this->clicked_point.x, this->clicked_point.y);
    Node node_coordinates(coordinates);
    if (this->_utils.check_collision(node_coordinates))
    {
        // Check if the clicked coordinates are the start or end point.
        if ((this->clicked_point.x == this->xI.first && this->clicked_point.y == this->xI.second) ||
            (this->clicked_point.x == this->xG.first && this->clicked_point.y == this->xG.second))
        {
            std::cout << "Clicked point is the start or end point!" << std::endl;
            // continue;
        }

        // // Check if the clicked coordinates are part of the path.
        // if (std::find(this->_path.begin(), this->_path.end(), coordinates) != this->_path.end())
        // {
        //     std::cout << "Clicked point is part of the path!" << std::endl;
        //     continue;
        // }

        else {

            // If the clicked coordinates are not part of the obstacle space, then add them to the obstacle space.
            this->_env.add_obs_circle(std::make_pair(this->clicked_point.x, this->clicked_point.y), 1);

            // Add the new obstacle to the Plotting object.
            this->update_obs();

            // Update the image based on the new obstacles.
            this->show_image(windowName);

            cv::waitKey(1); // Wait until a key is pressed

            std::cout << "New obstacle added!" << std::endl;

            // if (cv::waitKey(0) >= 0) {  // If a key is pressed, break from the loop
            //     break;
            // }
        }
    }
    else
    {
        // If the clicked coordinates are part of the obstacle space, then remove them from the obstacle space and update the image.
        this->_env.remove_obs_circle(std::make_pair(this->clicked_point.x, this->clicked_point.y));

        // Remove the obstacle from the Plotting object.
        this->update_obs();

        // Erase the clicked point from the image.
        cv::rectangle(image,
                        cv::Point(this->clicked_point.x * cell_size, this->clicked_point.y * cell_size),
                        cv::Point((this->clicked_point.x + 1) * cell_size - 2, (this->clicked_point.y + 1) * cell_size - 2),
                        cv::Scalar(0, 0, 0),
                        -1); // Obstacle color

        // Update the image based on the new obstacles.
        this->show_image(windowName);

        cv::waitKey(1); // Wait until a key is pressed

        std::cout << "Obstacle removed!" << std::endl;
    }
    // }  
    
    // cv::imshow(windowName, image);

    // // Print the updated obstacle space size.
    // std::cout << "Obstacle space size: " << this->_env.get_obs().size() << std::endl;

    // inputThread.join(); // Make sure to join the thread
    return this->clicked_point;  
}

std::atomic<bool> Plotting::stopLoop(false);

void Plotting::checkForInput()
{
    // This code is used to check if a key has been pressed on the keyboard in UNIX based systems

    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

    char c;
    while (!this->stopLoop) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            std::cout << "Input received: " << c << std::endl;
            this->stopLoop = true;
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
}