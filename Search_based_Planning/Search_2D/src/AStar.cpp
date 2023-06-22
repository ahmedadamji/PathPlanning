#include "AStar.h"
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
#include <stack>
#include <math.h>

AStar::AStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type)
    : _env(env), _plot(plot), _xI(xI), _xG(xG), _heuristicType(heuristic_type)
{
    _uSet = _env.motions;
    _obs = _env.obs;

}

AStar::PointSetPair AStar::searching()
{
    AStar::PointSetPair pathVisitedPair;
    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = AStar::INF;
    
    std::cout << "Starting at:" << std::endl;
    std::cout << "xI: " << _xI.first << ", " << _xI.second << std::endl;

    _open.emplace(this->fValue(_xI), _xI);

    int count = 0;

    while(!_open.empty())
    {
        Point s = _open.top().second;
        _xC = s;
        _open.pop();
        _closed.insert(s);
        
        
        if (count % 25 == 0)
        {
            // Plot visited points and path.
            this->displayPlots();
            cv::waitKey(25); // Pause for a short time
        }

        count++;

        if(s == _xG)
        {           
            std::cout << "Goal found!" << std::endl; 
            break;
        }

        PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            double new_cost = _g[s] + this->cost(s, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g.insert({s_next, AStar::INF});

            if(new_cost < _g[s_next])
            {
                _g[s_next] = new_cost;
                _parent[s_next] = s;
                _open.emplace(this->fValue(s_next), s_next);
            }
        }
    }

    // Plot visited points and path.
    _plot.plot_visited(_closed);
    _plot.plot_path(this->extractPath(_parent));
    _plot.show_image();
    cv::waitKey(0);
    
    pathVisitedPair.first = this->extractPath(_parent);
    pathVisitedPair.second = _closed;
    
    return pathVisitedPair;
}

AStar::PointSetPair AStar::searchingRepeatedAStar(double &e)
{
    AStar::PointSetPair pathVisitedPair;


    return pathVisitedPair;
}

AStar::PointSetPair AStar::repeatedSearching(double &e)
{
    AStar::PointSetPair pathVisitedPair;


    return pathVisitedPair;
}

AStar::PointVector AStar::getNeighbours(AStar::Point &s)
{
    AStar::PointVector neighbours;
    AStar::Point s_next;
    
    for (auto u : _uSet)
    {
        s_next = std::make_pair(s.first + u.first, s.second + u.second);
        if ((s_next.first >= 0) && (s_next.first < _env.x_range) && (s_next.second >= 0) && (s_next.second < _env.y_range))
        {
            if (_obs.find(s_next) == _obs.end())
            {
                neighbours.push_back(s_next);
            }
        }
    }

    return neighbours;
}

double AStar::cost(AStar::Point &s_start, AStar::Point &s_goal)
{

    if (this->isCollision(s_start, s_goal))
    {
        return AStar::INF;
    }
    else
    {
        return std::sqrt(std::pow(s_goal.first - s_start.first, 2) + std::pow(s_goal.second - s_start.second, 2));
    }
}

bool AStar::isCollision(AStar::Point &s_start, AStar::Point &s_goal)
{
    if (_obs.find(s_start) != _obs.end() || _obs.find(s_goal) != _obs.end())
    {
        return true;
    }

    if (s_start.first != s_goal.first && s_start.second != s_goal.second)
    {
        AStar::Point s1;
        AStar::Point s2;

        if (s_goal.first - s_start.first == s_start.second - s_goal.second)
        {
            s1 = std::make_pair(std::min(s_start.first, s_goal.first), std::min(s_start.second, s_goal.second));
            s2 = std::make_pair(std::max(s_start.first, s_goal.first), std::max(s_start.second, s_goal.second));
        }
        else
        {
            s1 = std::make_pair(std::min(s_start.first, s_goal.first), std::max(s_start.second, s_goal.second));
            s2 = std::make_pair(std::max(s_start.first, s_goal.first), std::min(s_start.second, s_goal.second));
        }

        if (_obs.find(s1) != _obs.end() || _obs.find(s2) != _obs.end())
        {
            return true;
        }
    }

    return false;
}


double AStar::fValue(AStar::Point &s)
{
    return _g[s] + this->heuristic(s);
}

AStar::PointSet AStar::extractPath(std::map<Point, Point> &parent)
{
    AStar::PointSet path;
    
    Point s = _xG;
    path.insert(s);
    while (s != _xI)
    {
        // std::cout << "s: " << s.first << ", " << s.second << std::endl;
        s = parent[s];
        path.insert(s);
    }

    return path;
}

double AStar::heuristic(AStar::Point &s)
{
    // A* uses a heuristic function to estimate the cost to reach the goal from a given node.
    // The quality of the heuristic greatly affects the efficiency of the A* search.
    // A good heuristic function can guide the A* algorithm to explore less unnecessary paths
    // and find the shortest path more quickly.

    std::string heuristic_type = _heuristicType;
    if (heuristic_type == "manhattan")
    {
        return std::abs(s.first - _xG.first) + std::abs(s.second - _xG.second);
    }
    else
    {
        return std::sqrt(std::pow(s.first - _xG.first, 2) + std::pow(s.second - _xG.second, 2));
    }
}

void AStar::displayPlots()
{
    _plot.plot_visited(_closed);
    _plot.show_image();
}