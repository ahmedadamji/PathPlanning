#include "BFS.h"
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


BFS::PointVectorPointSetPair BFS::searching()
{
    BFS::PointVectorPointSetPair pathVisitedPair;
    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = BFS::INF;
    
    std::cout << "Starting at:" << std::endl;
    std::cout << "xI: " << _xI.first << ", " << _xI.second << std::endl;

    _open.push(_xI);

    while(!_open.empty())
    {
        _xC = _open.front();
        _open.pop();
        _closed.insert(_xC);
        
        _plot.plot_animation("Breadth-First Search", _closed, _path);

        if(_xC == _xG)
        {
            break;
        }

        BFS::PointVector neighbours = this->getNeighbours(_xC);

        for(auto s_next : neighbours)
        {
            // This code represents a variation of the traditional BFS algorithm. 
            // In a standard BFS, the algorithm visits all neighbors at the current depth 
            // before proceeding to nodes at the next depth level, without considering any cost associated with the nodes or edges.
            // However, in this variation, we are using a cost function to potentially change the parent node 
            // if a lower cost path is found to the node. This allows for a form of path optimization.
            // In the context of an unweighted graph, BFS does find the shortest path in terms of the number of edges. 
            // However, for weighted graphs, this variant may not guarantee the globally optimal shortest path.
            
            double new_cost = _g[_xC] + this->cost(_xC, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g.insert({s_next, BFS::INF});


            // If the new cost is less than the current cost for the node in the graph,
            // update the node's cost in the graph, set its parent to the current node,
            // and push the node onto the queue for further processing.
            if(new_cost < _g[s_next])
            {
                _g[s_next] = new_cost;
                _parent[s_next] = _xC;
                _open.push(s_next);
            }
        }
    }

    if(_xC == _xG)
    {           
        std::cout << "Goal found!" << std::endl;
    }
    else
    {
        std::cout << "Goal not found!" << std::endl;
    }

    std::cout << "Number of nodes in the open set: ";
    std::cout << _open.size() << std::endl;

    std::cout << "Number of visited nodes: ";
    std::cout << _closed.size() << std::endl;

    // Plot visited points and path.
    // _plot.plot_visited(_closed);
    // _plot.plot_path(this->extractPath(_parent));
    // _plot.show_image("Breadth-First Search");
    // cv::waitKey(0);
    _plot.plot_animation("Breadth-First Search", _closed, this->extractPath(_parent));
    
    pathVisitedPair.first = this->extractPath(_parent);
    pathVisitedPair.second = _closed;
    
    return pathVisitedPair;
}
