#include "Dijkstra.h"
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


Dijkstra::PointVectorPointSetPair Dijkstra::searching()
{
    Dijkstra::PointVectorPointSetPair pathVisitedPair;
    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = Dijkstra::INF;
    
    std::cout << "Starting at:" << std::endl;
    std::cout << "xI: " << _xI.first << ", " << _xI.second << std::endl;

    _open.emplace(0, _xI);

    while(!_open.empty())
    {
        Dijkstra::Point s = _open.top().second;
        _xC = s;
        _open.pop();
        _closed.insert(s);
        
        _plot.plot_animation("Dijkstra's", _closed, _path);

        if(s == _xG)
        {           
            std::cout << "Goal found!" << std::endl; 
            break;
        }

        Dijkstra::PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            double new_cost = _g[s] + this->cost(s, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g.insert({s_next, Dijkstra::INF});

            if(new_cost < _g[s_next])
            {
                _g[s_next] = new_cost;
                _parent[s_next] = s;
                _open.emplace(new_cost, s_next);
            }
        }
    }
    
    _path = this->extractPath(_parent);
    pathVisitedPair.first = _path;
    pathVisitedPair.second = _closed;

    // Plot visited points and path.
    _plot.plot_animation("Dijkstra's", _closed, _path);
    
    return pathVisitedPair;
}