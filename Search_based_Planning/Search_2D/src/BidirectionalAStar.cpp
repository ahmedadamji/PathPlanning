#include "BidirectionalAStar.h"
#include "Plotting.h"
#include "Env.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <unordered_set>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <algorithm>
#include <math.h>

void BidirectionalAStar::init()
{
    _parent_forward[_xI] = _xI;
    _parent_backward[_xG] = _xG;
    
    _g_forward[_xI] = 0.0;
    _g_forward[_xG] = BidirectionalAStar::INF;
    _g_backward[_xG] = 0.0;
    _g_backward[_xI] = BidirectionalAStar::INF;

    _open_forward.emplace(this->fValue(_xI), _xI);
    _open_backward.emplace(this->fValue(_xG), _xG);

    _xM = _xI;
}


BidirectionalAStar::PointSetPair BidirectionalAStar::searching()
{
    BidirectionalAStar::PointSetPair pathVisitedPair;

    this->init();
    
    std::cout << "Starting Forward Search at:" << std::endl;
    std::cout << "xI: " << _xI.first << ", " << _xI.second << std::endl;

    std::cout << "Starting Backward Search at:" << std::endl;
    std::cout << "xG: " << _xG.first << ", " << _xG.second << std::endl;

    int count = 0;

    while(!_open_forward.empty() && !_open_backward.empty())
    {
        
        if (count % 10 == 0)
        {
            // Plot visited points and path.
            this->displayPlots();
            cv::waitKey(40); // Pause for a short time
        }

        count++;

        // Forward search

        _xCF = _open_forward.top().second;
        _open_forward.pop();

        if (_parent_backward.find(_xCF) != _parent_backward.end())
        {
            _xM = _xCF;
            std::cout << "Goal found!" << std::endl;
            break;
        }

        _closed_forward.insert(_xCF);
        

        BidirectionalAStar::PointVector neighbours = this->getNeighbours(_xCF);
        for(auto s_next : neighbours)
        {
            double new_cost = _g_forward[_xCF] + this->cost(_xCF, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g_forward.insert({s_next, BidirectionalAStar::INF});

            if(new_cost < _g_forward[s_next])
            {
                _g_forward[s_next] = new_cost;
                _parent_forward[s_next] = _xCF;
                _open_forward.emplace(this->fValueForward(s_next), s_next);
            }
        }

        // Backward search

        _xCB = _open_backward.top().second;
        _open_backward.pop();

        if (_parent_forward.find(_xCB) != _parent_forward.end())
        {
            _xM = _xCB;
            std::cout << "Goal found!" << std::endl;
            break;
        }

        _closed_backward.insert(_xCB);

        neighbours = this->getNeighbours(_xCB);
        for(auto s_next : neighbours)
        {
            double new_cost = _g_backward[_xCB] + this->cost(_xCB, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g_backward.insert({s_next, BidirectionalAStar::INF});

            if(new_cost < _g_backward[s_next])
            {
                _g_backward[s_next] = new_cost;
                _parent_backward[s_next] = _xCB;
                _open_backward.emplace(this->fValueBackward(s_next), s_next);
            }
        }

    }

    std::cout << "Number of open nodes in the forward set: ";
    std::cout << _open_forward.size() << std::endl;

    std::cout << "Number of visited nodes in the forward set: ";
    std::cout << _closed_forward.size() << std::endl;

    std::cout << "Number of open nodes in the backward set: ";
    std::cout << _open_backward.size() << std::endl;

    std::cout << "Number of visited nodes in the backward set: ";
    std::cout << _closed_backward.size() << std::endl;

    // Plot visited points and path.
    std::vector<int> color_forward = {0, 125, 125};
    _plot.plot_visited(_closed_forward, color_forward);
    std::vector<int> color_backward = {125, 125, 0};
    _plot.plot_visited(_closed_backward, color_backward);
    _plot.plot_path(this->extractPath());
    _plot.show_image();
    cv::waitKey(0);
    
    pathVisitedPair.first = this->extractPath();

    BidirectionalAStar::PointSet closed;
    closed.insert(_closed_forward.begin(), _closed_forward.end());
    closed.insert(_closed_backward.begin(), _closed_backward.end());
    pathVisitedPair.second = closed;
    
    return pathVisitedPair;
}

double BidirectionalAStar::heuristic(BidirectionalAStar::Point &s, BidirectionalAStar::Point &goal)
{
    // A* uses a heuristic function to estimate the cost to reach the goal from a given node.
    // The quality of the heuristic greatly affects the efficiency of the A* search.
    // A good heuristic function can guide the A* algorithm to explore less unnecessary paths
    // and find the shortest path more quickly.

    std::string heuristic_type = _heuristicType;
    if (heuristic_type == "manhattan")
    {
        return std::abs(s.first - goal.first) + std::abs(s.second - goal.second);
    }
    else
    {
        return std::sqrt(std::pow(s.first - goal.first, 2) + std::pow(s.second - goal.second, 2));
    }
}

double BidirectionalAStar::fValueForward(Point &s)
{
    double f = _g_forward[s] + this->heuristic(s, _xG);
    return f;
}

double BidirectionalAStar::fValueBackward(Point &s)
{
    double f = _g_backward[s] + this->heuristic(s, _xI);
    return f;
}

BidirectionalAStar::PointSet BidirectionalAStar::extractPath()
{
    // Extract the path from the forward search.
    BidirectionalAStar::PointSet path_forward;
    path_forward.insert(_xM);
    
    BidirectionalAStar::Point s = _xM;

    while (true)
    {
        // std::cout << "s: " << s.first << ", " << s.second << std::endl;
        s = _parent_forward[s];
        path_forward.insert(s);
        if (s == _xI)
        {
            break;
        }
    }

    // Extract the path from the backward search. 
    BidirectionalAStar::PointSet path_backward;
    s = _xM;

    while (true)
    {
        // std::cout << "s: " << s.first << ", " << s.second << std::endl;
        s = _parent_backward[s];
        path_backward.insert(s);
        if (s == _xG)
        {
            break;
        }
    }

    // Combine the two paths.
    BidirectionalAStar::PointSet path;
    path.insert(path_forward.begin(), path_forward.end());
    path.insert(path_backward.begin(), path_backward.end());

    return path;
}


void BidirectionalAStar::displayPlots()
{
    std::vector<int> color_forward = {0, 125, 125};
    _plot.plot_visited(_closed_forward, color_forward);
    std::vector<int> color_backward = {125, 125, 0};
    _plot.plot_visited(_closed_backward, color_backward);
    _plot.show_image();
}