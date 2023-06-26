#include "ARAStar.h"
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

void ARAStar::init()
{
    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = ARAStar::INF;

    _open.emplace(_xI, this->fValue(_xI));
}

ARAStar::PointVectorPointSetPair ARAStar::searching(double &e)
{
    ARAStar::PointVectorPointSetPair pathVisitedPair;
    _repeatedCount = 0;

    this->init();
    _e = e;

    ARAStar::PointVector path;
    ARAStar::PointSet closed;

    pathVisitedPair = this->improvePath();
    path = pathVisitedPair.first;
    closed = pathVisitedPair.second;

    while (this->updateEpsilon() > 1.0)
    {
        _e -= 0.4;


        // Add or replace nodes from _incons in _open
        for (const auto& pair : _incons)
        {
            double newFValue = fValue(pair.first);
            
            // If 's' already exists in _open, remove it
            if (_open.find(pair.first) != _open.end()) {
                _open.erase(pair.first);
            }
            
            // Add or replace 's' in _open
            _open[pair.first] = newFValue;
        }

        // Update fValues for all nodes in _open
        for (auto& pair : _open) {
            double newFValue = fValue(pair.first);
            if (pair.second != newFValue) {
                pair.second = newFValue;
            }
        }

        // clear _incons
        _incons.clear();
        // clear _closed
        _closed.clear();

        // Run improvePath() again
        pathVisitedPair = this->improvePath();
        path = pathVisitedPair.first;
        closed = pathVisitedPair.second;
        _plot.plot_animation_repeated_astar("ARA*", pathVisitedPair.second, pathVisitedPair.first, _repeatedCount, (this->updateEpsilon() <= 1.0));

        _repeatedCount += 1;
    }



    


    return pathVisitedPair;
}

ARAStar::PointVectorPointSetPair ARAStar::improvePath()
{
    ARAStar::PointVectorPointSetPair pathVisitedPair;
    ARAStar::PointSet visited;

    while(true)
    {
        ARAStar::PointWithPriority s_with_smallest_f = this->calcSmallestF();
        Point s = s_with_smallest_f.first;
        double f_small = s_with_smallest_f.second;


        // The condition "if self.f_value(self.s_goal) <= f_small:" checks
        // if the current f-value of the goal node, s_goal, is less than
        // or equal to the smallest f-value, f_small, among the nodes in
        // the OPEN set.

        // If the condition is true, it means that the algorithm has found
        // a path to the goal that is as good as or better than any potential
        // path it could explore in the future. This is because all future
        // paths would have to pass through a node in the OPEN set, and all
        // of those nodes have an f-value greater than the current path to
        // the goal. Therefore, it is logical to stop the search and declare
        // the current path to the goal as the best one found.

        // Note that the incons set is already merged into the open set at this point.
        if (f_small >= this->fValue(_xG)) {
            break;
        }

        _open.erase(s);
        _closed.insert(s);
        

        // if(s == _xG)
        // {           
        //     std::cout << "Goal found!" << std::endl; 
        //     break;
        // }

        PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            double new_cost = _g[s] + this->cost(s, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g.insert({s_next, ARAStar::INF});

            if(new_cost < _g[s_next])
            {
                _g[s_next] = new_cost;
                _parent[s_next] = s;

                visited.insert(s_next);
                _plot.plot_animation_repeated_astar("ARA*", visited, _path, _repeatedCount, (_e == 1.0));

                if (_closed.count(s_next) == 0) {
                    _open[s_next] = fValue(s_next);
                } else {
                    // ARA* maintains an incons list to handle inconsistent nodes.
                    // Inconsistent nodes are nodes that have been expanded under a larger ε
                    // and have a g-value less than the v-value
                    // (estimate of the cost-to-come from start to the current node).
                    _incons[s_next] = 0.0;
                }

            }
        }
    }
    // Plot visited points and path.
    pathVisitedPair.first = this->extractPath(_parent);
    pathVisitedPair.second = visited;

    // _plot.plot_animation_repeated_astar("ARA*", pathVisitedPair.second, pathVisitedPair.first, _repeatedCount, (_e == 1.0));
    
    return pathVisitedPair;
}

double ARAStar::updateEpsilon()
{
    // This function calculates the minimum
    // value of the sum of the heuristic and g value for each state in
    // the open and inconsistent lists. This minimum value is then used
    // to update ε by taking the minimum of the current ε and the ratio
    // of the g value at the goal state to the computed minimum.

    // The update rule is based on the idea that the ratio of the g value
    // at the goal state to the minimum value represents an estimate of
    // the optimal cost. As the algorithm discovers shorter paths to the
    // goal, this ratio decreases, resulting in a decrease in ε. A smaller
    // ε brings the algorithm closer to the behavior of the ordinary A*
    // algorithm, known for finding optimal paths.

    // By iteratively reducing ε and re-running the search, the ARA*
    // algorithm progressively refines its solution, approaching the
    // optimal path as more computation time is allotted.

    double v = ARAStar::INF;

    // Iterate over the map _open
    for (const auto& pair : _open) {
        v = std::min(v, ARAStar::heuristic(pair.first) + _g[pair.first]);
    }

    for (const auto& pair : _incons) {
        v = std::min(v, ARAStar::heuristic(pair.first) + _g[pair.first]);
    }

    return std::min(_e, (_g[_xG] / v));
}


ARAStar::PointWithPriority ARAStar::calcSmallestF()
{
    ARAStar::PointWithPriority s_with_smallest_f = *_open.begin();  // Start with the first pair in the map
    
    // Iterate over the map to find the pair with the smallest fValue
    for (const auto& pair : _open) {
        if (pair.second < s_with_smallest_f.second) {
            s_with_smallest_f = pair;
        }
    }

    return s_with_smallest_f;  
    
}


double ARAStar::fValue(const ARAStar::Point &s)
{
    return _g[s] + (_e * this->heuristic(s));
}



double ARAStar::heuristic(const ARAStar::Point &s)
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
