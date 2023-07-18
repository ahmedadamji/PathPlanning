#include "RRT.h"
#include "Plotting.h"
#include "Env.h"
#include "Utils.h"
#include "Node.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <unordered_set>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <algorithm>
#include <stack>
#include <math.h>
#include <random>
#include <memory>

// As a reminder, objects defined in the header with & are references, and should be initialized at the point of declaration. And once initialized, they cannot be changed to refer to another object.
// This is why objects defined as & need to be initialized in the constructor's initialization list. Otherwise, they will be initialized with the default constructor, and then assigned a new value in the constructor's body.
// It is also to be noted that the argument inside the object must be a reference as well. Not the parameter passed to the constructor.
// For example:
// in RRT.h: Env &_env;
// in RRT.cpp: RRT::RRT(Env &env) : _env(env) {}
// This is the same as:
// in RRT.h: Env _env;
// in RRT.cpp: RRT::RRT(Env &env) { _env = env; } // This is only possible if Env has a default constructor. Which looks like: Env() {}
RRT::RRT(Env &env, Plotting &plot, Utils &utils, Point xI, Point xG, double step_len, double goal_sample_rate, double iter_max)
    : _env(env), _plot(plot), _utils(utils), _xI(xI), _xG(xG), _step_len(step_len), _goal_sample_rate(goal_sample_rate), _iter_max(iter_max)
{
    this->_xI_node = Node(xI);
    this->_xG_node = Node(xG);
    this->_vertex.push_back(std::make_shared<Node>(this->_xI_node));
    this->_x_range = env.x_range;
    this->_y_range = env.y_range;
    this->obs_boundary = env.get_obs_boundary();
    this->obs_rectangle = env.get_obs_rectangle();
    this->obs_circle = env.get_obs_circle();

    // _uSet = _env.motions;

}

RRT::PointVector RRT::Planning()
{
    std::string windowName = "RRT";
    for (int iter = 0; iter < this->_iter_max; iter++)
    {
        std::shared_ptr<Node> node_rand = std::make_shared<Node>(generate_random_node());
        std::shared_ptr<Node> node_nearest = std::make_shared<Node>(nearest_neighbor(this->_vertex, *node_rand));
        std::shared_ptr<Node> node_new = new_state(node_nearest, *node_rand);

        if (!(this->_utils.check_collision(*node_nearest, *node_new)))
        {
            this->_vertex.push_back(node_new);
            double dist = this->_utils.get_dist(*node_new, this->_xG_node);

            if ((dist <= this->_step_len) && !(this->_utils.check_collision(*node_new, this->_xG_node)))
            {
                node_new = this->new_state(node_new, this->_xG_node);
                _path = extract_path(*node_new);
                this->_plot.plot_animation(windowName, this->_vertex, _path);
                return _path;
            }
        }
        this->_plot.plot_animation(windowName, this->_vertex, _path);
    }

    return {};

}

Node RRT::generate_random_node()
{
    double delta = this->_utils.delta;

    // Create a random number generator engine
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    // Generate a random number in the range [0.0, 1.0)
    double randomValue = dis(gen);

    if (randomValue > _goal_sample_rate)
    {
        std::uniform_real_distribution<double> dis_x(0 + delta, this->_x_range - delta);
        std::uniform_real_distribution<double> dis_y(0 + delta, this->_y_range - delta);

        double randomX = dis_x(gen);
        double randomY = dis_y(gen);

        return Node(std::make_pair(randomX, randomY));
    }

    // We return the goal node instead of a random node because we want to bias the tree growth towards the goal based on the goal sample rate.
    return _xG_node;
}

Node RRT::nearest_neighbor(RRT::NodePtrVector node_list, Node n)
{
    double min_dist = INF;
    Node nearest_node;

    for (auto node : node_list)
    {
        double dist = this->_utils.get_dist(*node, n);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_node = *node;
        }
    }

    return nearest_node;
}

std::shared_ptr<Node> RRT::new_state(std::shared_ptr<Node>& node_start, Node& node_end)
{
    double dist = std::min(this->_utils.get_dist(*node_start, node_end), this->_step_len);
    double theta = this->_utils.get_angle(*node_start, node_end);
    double new_x = node_start->x + this->_step_len * std::cos(theta);
    double new_y = node_start->y + this->_step_len * std::sin(theta);
    std::shared_ptr<Node> new_node = std::make_shared<Node>(std::make_pair(new_x, new_y));
    new_node->parent = node_start;

    return new_node;
}

RRT::PointVector RRT::extract_path(Node node_end)
{
    PointVector path;
    path.push_back(this->_xG);
    Node node_curr = node_end;

    while (node_curr.parent != nullptr)
    {
        node_curr = *node_curr.parent;
        path.push_back(std::make_pair(node_curr.x, node_curr.y));
    }

    return path;
}

void RRT::update_obs()
{
    this->obs_boundary = this->_env.get_obs_boundary();
    this->obs_rectangle = this->_env.get_obs_rectangle();
    this->obs_circle = this->_env.get_obs_circle();
}