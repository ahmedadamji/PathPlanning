#include "RRT_Connect.h"
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


RRT_Connect::RRT_Connect(Env &env, Plotting &plot, Utils &utils, Point xI, Point xG, double step_len, double goal_sample_rate, double iter_max)
    : _env(env), _plot(plot), _utils(utils), _xI(xI), _xG(xG), _step_len(step_len), _goal_sample_rate(goal_sample_rate), _iter_max(iter_max)
{
    this->_xI_node = Node(xI);
    this->_xG_node = Node(xG);
    this->_vertex.push_back(std::make_shared<Node>(this->_xI_node));
    this->_V1.push_back(std::make_shared<Node>(this->_xI_node));
    this->_V2.push_back(std::make_shared<Node>(this->_xG_node));
    this->_x_range = env.x_range;
    this->_y_range = env.y_range;
    this->obs_boundary = env.get_obs_boundary();
    this->obs_rectangle = env.get_obs_rectangle();
    this->obs_circle = env.get_obs_circle();

    // _uSet = _env.motions;

}

RRT_Connect::PointVector RRT_Connect::Planning()
{
    std::string windowName = "RRT_Connect";
    for (int iter = 0; iter < this->_iter_max; iter++)
    {
        std::shared_ptr<Node> node_rand = std::make_shared<Node>(generate_random_node());
        std::shared_ptr<Node> node_nearest = nearest_neighbor(this->_V1, *node_rand);
        std::shared_ptr<Node> node_new = new_state(node_nearest, *node_rand);

        if (!(this->_utils.check_collision(*node_nearest, *node_new)))
        {
            this->_V1.push_back(node_new);
            std::shared_ptr<Node> node_nearest_prim = nearest_neighbor(this->_V2, *node_new);
            std::shared_ptr<Node> node_new_prim = new_state(node_nearest_prim, *node_new);

            if (!(this->_utils.check_collision(*node_nearest_prim, *node_new_prim)))
            {
                this->_V2.push_back(node_new_prim);
                
                while (true)
                {
                    // Here we are creating a new state from node_new_prim towards node_new.
                    std::shared_ptr<Node> node_new_prim2 = new_state(node_new_prim, *node_new);
                    if (!(this->_utils.check_collision(*node_new_prim2, *node_new_prim)))
                    {
                        this->_V2.push_back(node_new_prim2);
                        node_new_prim = this->change_node(node_new_prim, node_new_prim2);
                    }
                    else
                    {
                        break;
                    }

                    if (this->is_node_same(*node_new_prim, *node_new))
                    {
                        break;
                    }
                }
            }

            if (this->is_node_same(*node_new_prim, *node_new))
            {
                this->_path = this->extract_path(*node_new, *node_new_prim);
                this->_plot.plot_animation_connect(windowName, this->_V1, this->_V2, _path);
                return this->_path;
            }
        }
        if (this->_V1.size() > this->_V2.size())
        {
            std::swap(this->_V1, this->_V2);
        }
        
        this->_plot.plot_animation_connect(windowName, this->_V1, this->_V2, _path);
    }

    return {};

}

std::shared_ptr<Node> RRT_Connect::change_node(std::shared_ptr<Node> node_new_prim, std::shared_ptr<Node> node_new_prim2)
{
    // This function is used to change the parent of node_new_prim2 to node_new_prim.
    Node node(std::make_pair(node_new_prim2->x, node_new_prim2->y));
    std::shared_ptr<Node> node_new = std::make_shared<Node>(node);
    node_new->parent = node_new_prim;
    return node_new;
}

bool RRT_Connect::is_node_same(Node node1, Node node2)
{
    return (node1.x == node2.x) && (node1.y == node2.y);
}

Node RRT_Connect::generate_random_node()
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

std::shared_ptr<Node> RRT_Connect::nearest_neighbor(RRT_Connect::NodePtrVector node_list, Node n)
{
    double min_dist = INF;
    std::shared_ptr<Node> nearest_node;

    for (auto node : node_list)
    {
        double dist = this->_utils.get_dist(*node, n);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_node = node;
        }
    }

    return nearest_node;
}


std::shared_ptr<Node> RRT_Connect::new_state(std::shared_ptr<Node>& node_start, Node& node_end)
{
    double dist = this->_utils.get_dist(*node_start, node_end);
    double theta = this->_utils.get_angle(*node_start, node_end);

    // If the distance is less than the step length, use the actual distance.
    // This will prevent the function from overshooting the target node.
    if (dist < this->_step_len) {
        double new_x = node_start->x + dist * std::cos(theta);
        double new_y = node_start->y + dist * std::sin(theta);
        std::shared_ptr<Node> new_node = std::make_shared<Node>(std::make_pair(new_x, new_y));
        // Check if this is not a collision point.
        if (!(this->_utils.check_collision(*node_start, *new_node)))
        {
            // Here I am assigning the parent of the new node as the node_start which is a shared_ptr<Node> type.
            // This would already be in the _vertex vector, so when node_start goes out of scope, the new_node will still have a parent, as it is shared with the corresponding node in the _vertex vector.
            // This is a great example of where shared_ptr<Node> is useful. This prevents undefined behavior when the address of the parent node is accessed after the original memory location has been freed.
            new_node->parent = node_start;

            return new_node;
        }

    }

    // The rest of the code remains the same
    double new_x = node_start->x + this->_step_len * std::cos(theta);
    double new_y = node_start->y + this->_step_len * std::sin(theta);
    std::shared_ptr<Node> new_node = std::make_shared<Node>(std::make_pair(new_x, new_y));
    // Here I am assigning the parent of the new node as the node_start which is a shared_ptr<Node> type.
    // This would already be in the _vertex vector, so when node_start goes out of scope, the new_node will still have a parent, as it is shared with the corresponding node in the _vertex vector.
    // This is a great example of where shared_ptr<Node> is useful. This prevents undefined behavior when the address of the parent node is accessed after the original memory location has been freed.
    new_node->parent = node_start;

    return new_node;
}


RRT_Connect::PointVector RRT_Connect::extract_path(Node node_end, Node node_end_prim)
{
    PointVector path1;
    path1.push_back(std::make_pair(node_end.x, node_end.y));
    Node node_curr = node_end;

    while (node_curr.parent != nullptr)
    {
        node_curr = *node_curr.parent;
        path1.push_back(std::make_pair(node_curr.x, node_curr.y));
    }

    PointVector path2;
    path2.push_back(std::make_pair(node_end_prim.x, node_end_prim.y));
    node_curr = node_end_prim;

    while (node_curr.parent != nullptr)
    {
        node_curr = *node_curr.parent;
        path2.push_back(std::make_pair(node_curr.x, node_curr.y));
    }

    std::reverse(path1.begin(), path1.end());
    path1.insert(path1.end(), path2.begin(), path2.end());

    return path1;
}

void RRT_Connect::update_obs()
{
    this->obs_boundary = this->_env.get_obs_boundary();
    this->obs_rectangle = this->_env.get_obs_rectangle();
    this->obs_circle = this->_env.get_obs_circle();
}