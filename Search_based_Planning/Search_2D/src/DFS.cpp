#include "DFS.h"
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


DFS::PointSetPair DFS::searching()
{
    DFS::PointSetPair pathVisitedPair;
    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = DFS::INF;
    
    std::cout << "Starting at:" << std::endl;
    std::cout << "xI: " << _xI.first << ", " << _xI.second << std::endl;

    _open.push(_xI);

    int count = 0;

    while(!_open.empty())
    {
        _xC = _open.top();
        _open.pop();
        _closed.insert(_xC);
        
        
        if (count % 100 == 0)
        {
            // Plot visited points and path.
            this->displayPlots();
            cv::waitKey(5); // Pause for a short time
        }

        count++;

        if(_xC == _xG)
        {
            break;
        }

        DFS::PointVector neighbours = this->getNeighbours(_xC);

        for(auto s_next : neighbours)
        {
            // This is a variation of the standard DFS algorithm, where we are introducing a cost factor 
            // to influence the selection of parent nodes for each node in the graph. In a traditional DFS, 
            // a node's parent is always the node from which it was visited and does not change, and costs are not considered.
            // However, in this variation, we use cost as a factor to potentially update a node's parent 
            // if a lower cost path is found to the node. This allows for a form of path optimization, 
            // but note that this does not guarantee finding the shortest path in all types of graphs.

            double new_cost = _g[_xC] + this->cost(_xC, s_next);

            // Insert is used as it does not overwrite the value if the key already exists.
            auto result = _g.insert({s_next, DFS::INF});


            // If the new cost is less than the current cost for the node in the graph,
            // update the node's cost in the graph, set its parent to the current node,
            // and push the node onto the stack for further processing.
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
    _plot.plot_visited(_closed);
    _plot.plot_path(this->extractPath(_parent));
    _plot.show_image();
    cv::waitKey(0);
    
    pathVisitedPair.first = this->extractPath(_parent);
    pathVisitedPair.second = _closed;
    
    return pathVisitedPair;
}
