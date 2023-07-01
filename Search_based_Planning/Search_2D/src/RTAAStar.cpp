#include "RTAAStar.h"
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

void RTAAStar::init(int N)
{
    _N = N;

    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = RTAAStar::INF;

    _xS = _xI;

    for (int i = 0; i < _env.x_range; i++)
    {
        for (int j = 0; j < _env.y_range; j++)
        {
            RTAAStar::Point s = std::make_pair(i, j);
            _h_table[s] = this->heuristic(s);
        }
    }

}

RTAAStar::PointVectorPointSetPair RTAAStar::searching(int N)
{
    /*
        The searching(int N) function is the main function that coordinates the others.
        It first calls repeatedSearching to explore the nodes,
        then it uses iteration to calculate and update the heuristic values.
        Lastly, it calls extractPathInClose to construct the path
        from the starting node to the node with an uncalculated heuristic value.
        This process continues until the goal is found.
    */
    RTAAStar::PointVectorPointSetPair pathVisitedPair;

    this->init(N);

    while(true)
    {
        RTAAStar::PointQueuePointVectorPair currentOpenClosedPair = this->repeatedSearching(_xS);
        RTAAStar::PointQueue open = currentOpenClosedPair.first;
        RTAAStar::PointVector closedVector = currentOpenClosedPair.second;
        RTAAStar::PointSet closed(closedVector.begin(), closedVector.end());

        if (closed.find(_xG) != closed.end())
        {
            _path.insert(_path.end(), closed.begin(), closed.end());
            _plot.plot_animation_repeated_astar("Real-Time Adaptive A*", closed, _path, _repeatedCount, true);
            break;
        }
        
        std::pair<RTAAStar::Point, std::map<RTAAStar::Point, double>> s_min_h_value_pair = this->computeHValue(open, closed);
        std::map<RTAAStar::Point, double> h_value = s_min_h_value_pair.second;

        // This is the node with the smallest v-value from the open set.
        // This is the node that we want to find a path to from the current start node because it is the node that will give us
        // the shortest path to the goal as well as the shortest path from the start.
        RTAAStar::Point xN = s_min_h_value_pair.first;

        for(auto& kv : h_value) {
            _h_table[kv.first] = kv.second;
        }


        std::pair<RTAAStar::Point, RTAAStar::PointVector> startPathPair = this->extractPathInClose(_xS, xN, h_value);
        _xS = startPathPair.first;
        _path.insert(_path.end(), startPathPair.second.begin(), startPathPair.second.end());
        _repeatedCount++;

        if(open.empty())
        {
            std::cout << "No path found!" << std::endl;
            break;
        }
    }

    pathVisitedPair.first = _path;
    pathVisitedPair.second = _closed;

    return pathVisitedPair;
}

std::pair<RTAAStar::Point, RTAAStar::PointVector> RTAAStar::extractPathInClose(RTAAStar::Point &xS, RTAAStar::Point &xN, std::map<Point, double> &h_value)
{
    // This function constructs a path from a given starting point xS to a node with an uncalculated heuristic value.
    // It repeatedly chooses the neighbor with the smallest heuristic value and adds it to the path until it finds a node whose heuristic value hasn't been calculated.

    std::pair<RTAAStar::Point, RTAAStar::PointVector> startPathPair;
    
    RTAAStar::PointVector path;
    RTAAStar::PointSet closed;
    path.push_back(xN);
    RTAAStar::Point s = xN;

    while(true)
    {
        std::map<RTAAStar::Point, double> h_value_neighbours;

        PointVector neighbours = this->getNeighbours(s);

        for(auto s_next : neighbours)
        {
            if (h_value.find(s_next) != h_value.end())
            {
                h_value_neighbours[s_next] = h_value[s_next];
            }
        }

        // Here we are finding the neighbour with the largest h-value because we are moving from the goal to the start.
        // This ensures that we are moving towards the start in a way that is furthest from the goal.
        // This makes sense because if if we did chose the neighbour with the smallest h-value, we would be moving towards the goal instead.
        RTAAStar::Point s_key = this->calcLargestH(h_value_neighbours).first;
        path.push_back(s_key);
        s = s_key;
        _xC = s;

        if (s == xS)
        {
            // Reverse the path to get the correct order.
            std::reverse(path.begin(), path.end());
            startPathPair = std::make_pair(xN, path);
            _plot.plot_animation_repeated_astar("Real-Time Adaptive A*", closed, path, _repeatedCount, false);
            _xS = xN;
            std::cout << "Returning path from extractPathInClose" << std::endl;
            return startPathPair;
        }

    }


}

std::pair<RTAAStar::Point, std::map<RTAAStar::Point, double>> RTAAStar::computeHValue(RTAAStar::PointQueue &open, RTAAStar::PointSet &closed)
{

    // This function computes updated heuristic values for the closed nodes based on
    // the smallest v value from the nodes in the open set.
    // This value is calculated by summing the g-value (cost function value) of the parent node,
    // the cost to move from the parent node to the current node,
    // and the heuristic value of the current node.

    
    std::map<RTAAStar::Point, double> h_value;
    std::map<RTAAStar::Point, double> v_open;

    RTAAStar::PointQueue open_copy = open;

    while(!open_copy.empty())
    {
        RTAAStar::Point s = open_copy.top().second;
        open_copy.pop();
        v_open[s] = _g_table[_parent[s]] + 1 + _h_table[s];
    }

    RTAAStar::Point s_with_smallest_v;
    double smallest_v = RTAAStar::INF;
    std::for_each(v_open.begin(), v_open.end(), [&smallest_v, &s_with_smallest_v](const auto& element) {
        if (element.second < smallest_v) {
            smallest_v = element.second;
            s_with_smallest_v = element.first;
        }
    });
    
    for (auto& item: closed)
    {
        // The h-value is the difference between the smallest v-value and the g-value of the node.
        // It is computed in this way because the g-value of the node is the cost of the shortest path from the start to the node.
        // The smallest v-value is the cost of the shortest path from the start to the goal.
        // The difference between the two is the cost of the shortest path from the node to the goal.
        // So the h-value is the cost of the shortest path from the node to the goal.
        h_value[item] = smallest_v - _g_table[item];
    }
    
    return std::make_pair(s_with_smallest_v, h_value);

}

RTAAStar::PointQueuePointVectorPair RTAAStar::repeatedSearching(RTAAStar::Point &xI)
{
    // This function conducts a search from a given starting point (xI).
    // It uses a priority queue (open) and a set of closed nodes (closed) to keep track of the nodes being explored.
    // In each iteration, it pops out a node from the queue, checks if it's the goal node, expands its neighbors,
    // and updates their cost if a better path is found.
    // The function returns when the goal node is found or it has iterated N times.

    RTAAStar::PointQueuePointVectorPair openClosedPair;
    
    RTAAStar::PointQueue open; // priority queue of open nodes

    open.emplace(_h_table[xI], xI);

    RTAAStar::PointSet closed; // list of closed nodes
    PointVector path;

    _g_table.clear();
    _parent.clear();
    
    _g_table[xI] = 0.0;
    _g_table[_xG] = RTAAStar::INF;
    
    _parent[xI] = xI;

    int count = 0;
    
    std::cout << "Starting current search iteration at: " << std::endl;
    std::cout << "xI: " << xI.first << ", " << xI.second << std::endl;

    while(!open.empty())
    {

        RTAAStar::Point s = open.top().second;
        _xC = s;
        open.pop();
        closed.insert(s);
         
        _plot.plot_animation_repeated_astar("Real-Time Adaptive A*", closed, path, _repeatedCount, false);

        count++;

        if(s == _xG)
        {   
            _closed.insert(closed.begin(), closed.end());   
            std::cout << "Goal found!" << std::endl;
            path = this->extractPath(xI, _parent);
            // PointSet pathSet(path.begin(), path.end());
            openClosedPair.first = open;
            openClosedPair.second = path;

            _plot.plot_animation_repeated_astar("Real-Time Adaptive A*", closed, path, _repeatedCount, false);

            return openClosedPair;
            
        }

        RTAAStar::PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            if(closed.find(s_next) == closed.end())
            {
                double new_cost = _g_table[s] + this->cost(s, s_next);

                // Insert is used as it does not overwrite the value if the key already exists.
                auto result = _g_table.insert({s_next, RTAAStar::INF});

                if(new_cost < _g_table[s_next])
                {
                    _g_table[s_next] = new_cost;
                    _parent[s_next] = s;
                    double g = _g_table[s_next] + _h_table[s_next];
                    open.emplace(g, s_next);
                }
            }
        }

        if (count == _N)
        {
            break;
        }
    }
    // Plot visited points and path.
    openClosedPair.first = open;
    PointVector closedVector(closed.begin(), closed.end());
    openClosedPair.second = closedVector;

    _closed.insert(closed.begin(), closed.end());

    
    
    return openClosedPair;
}


RTAAStar::PointVector RTAAStar::extractPath(RTAAStar::Point &xS, std::map<RTAAStar::Point, RTAAStar::Point> &parent)
{
    RTAAStar::PointVector path;
    
    RTAAStar::Point s = _xG;
    path.push_back(s);
    while (s != xS)
    {
        // std::cout << "s: " << s.first << ", " << s.second << std::endl;
        s = parent[s];
        path.push_back(s);
    }

    // Reverse the path to get the correct order.
    std::reverse(path.begin(), path.end());

    return path;
}

RTAAStar::PointWithPriority RTAAStar::calcSmallestH(std::map<RTAAStar::Point, double> &h_value)
{
    RTAAStar::PointWithPriority s_with_smallest_h = *h_value.begin();  // Start with the first pair in the map
    
    // Iterate over the map to find the pair with the smallest fValue
    for (const auto& pair : h_value) {
        if (pair.second < s_with_smallest_h.second) {
            s_with_smallest_h = pair;
        }
    }

    return s_with_smallest_h;  
    
}

RTAAStar::PointWithPriority RTAAStar::calcLargestH(std::map<RTAAStar::Point, double> &h_value)
{
    RTAAStar::PointWithPriority s_with_largest_h = *h_value.begin();  // Start with the first pair in the map
    
    // Iterate over the map to find the pair with the smallest fValue
    for (const auto& pair : h_value) {
        if (pair.second > s_with_largest_h.second) {
            s_with_largest_h = pair;
        }
    }

    return s_with_largest_h;  
    
}