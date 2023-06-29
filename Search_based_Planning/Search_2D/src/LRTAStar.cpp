#include "LRTAStar.h"
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

void LRTAStar::init(int N)
{
    _N = N;

    _parent[_xI] = _xI;
    _g[_xI] = 0.0;
    _g[_xG] = LRTAStar::INF;

    _xS = _xI;

    for (int i = 0; i < _env.x_range; i++)
    {
        for (int j = 0; j < _env.y_range; j++)
        {
            LRTAStar::Point s = std::make_pair(i, j);
            _h_table[s] = this->heuristic(s);
        }
    }

}

LRTAStar::PointVectorPointSetPair LRTAStar::searching(int N)
{
    /*
        The searching(int N) function is the main function that coordinates the others.
        It first calls repeatedSearching to explore the nodes,
        then it uses iteration to calculate and update the heuristic values.
        Lastly, it calls extractPathInClose to construct the path
        from the starting node to the node with an uncalculated heuristic value.
        This process continues until the goal is found.
    */
    LRTAStar::PointVectorPointSetPair pathVisitedPair;

    this->init(N);

    while(true)
    {
        LRTAStar::PointQueuePointSetPair currentOpenClosedPair = this->repeatedSearching(_xS);
        LRTAStar::PointQueue open = currentOpenClosedPair.first;
        LRTAStar::PointSet closed = currentOpenClosedPair.second;
        if (closed.find(_xG) != closed.end())
        {
            pathVisitedPair.first.insert(pathVisitedPair.first.end(), closed.begin(), closed.end());
            _plot.plot_animation_repeated_astar("Learning Real-Time A*", closed, pathVisitedPair.first, _repeatedCount, true);
            break;
        }
        std::map<LRTAStar::Point, double> h_value = this->iteration(closed);

        for(auto& kv : h_value) {
            _h_table[kv.first] = kv.second;
        }

        std::pair<LRTAStar::Point, LRTAStar::PointVector> startPathPair = this->extractPathInClose(_xS, h_value);
        _xS = startPathPair.first;
        pathVisitedPair.first.insert(pathVisitedPair.first.end(), startPathPair.second.begin(), startPathPair.second.end());
        _repeatedCount++;
    }

    return pathVisitedPair;
}

std::pair<LRTAStar::Point, LRTAStar::PointVector> LRTAStar::extractPathInClose(LRTAStar::Point &xS, std::map<Point, double> &h_value)
{
    // This function constructs a path from a given starting point xS to a node with an uncalculated heuristic value.
    // It repeatedly chooses the neighbor with the smallest heuristic value and adds it to the path until it finds a node whose heuristic value hasn't been calculated.

    std::pair<LRTAStar::Point, LRTAStar::PointVector> startPathPair;
    
    LRTAStar::PointVector path;
    LRTAStar::PointSet closed;
    path.push_back(xS);
    LRTAStar::Point s = xS;

    while(true)
    {
        std::map<LRTAStar::Point, double> h_value_neighbours;

        PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            if (h_value.find(s_next) != h_value.end())
            {
                h_value_neighbours[s_next] = h_value[s_next];
            }
            else
            {
                h_value_neighbours[s_next] = _h_table[s_next];
            }
        }

        LRTAStar::Point s_key = this->calcSmallestH(h_value_neighbours).first;
        path.push_back(s_key);
        s = s_key;

        if (h_value.find(s) == h_value.end())
        {
            startPathPair = std::make_pair(s, path);
            _plot.plot_animation_repeated_astar("Learning Real-Time A*", closed, path, _repeatedCount, false);
            return startPathPair;
        }

    }


}

std::map<LRTAStar::Point, double> LRTAStar::iteration(LRTAStar::PointSet &closed)
{
    /* 
    The 'iteration' function refines the heuristic values of the nodes in the closed set.
    Initially, heuristic values are set to infinity as the exact cost to reach the goal from each node is unknown.
    The function iterates until the heuristic values converge and stop changing significantly. 

    In each iteration, for every node, it considers its neighbours and their associated costs.
    It then updates the node's heuristic to be the minimum cost of moving to a neighbour plus the neighbour's heuristic. 

    The purpose of this iterative refinement is to make the heuristic values as accurate as possible
    based on the actual costs experienced while navigating the graph.
    This allows the algorithm to adapt its pathfinding strategy in real-time,
    making it particularly useful for scenarios where the graph is not fully known in advance.
    */


    std::map<LRTAStar::Point, double> h_value;

    // This is done because we dont want to expand the visited nodes again preferably.
    for (auto& item: closed)
    {
        h_value[item] = LRTAStar::INF;
    }

    while(true)
    {
        std::map<LRTAStar::Point, double> h_value_record;
        h_value_record = h_value;

        for (auto s: closed)
        {

            std::vector<double> f_neighbours;

            LRTAStar::PointVector neighbours = this->getNeighbours(s);
            for(auto s_next : neighbours)
            {
                if (closed.find(s_next) == closed.end())
                {
                    f_neighbours.push_back(this->cost(s, s_next) + _h_table[s_next]);
                }
                else
                {
                    f_neighbours.push_back(this->cost(s, s_next) + h_value[s_next]);
                }
                // We need to dereference the iterator to get the value, as min_element returns an iterator.
                h_value[s] = *std::min_element(f_neighbours.begin(), f_neighbours.end());
            }
        }

        if (h_value == h_value_record)
        {
            return h_value;
        }
        
    }

}

LRTAStar::PointQueuePointSetPair LRTAStar::repeatedSearching(LRTAStar::Point &xI)
{
    // This function conducts a search from a given starting point (xI).
    // It uses a priority queue (open) and a set of closed nodes (closed) to keep track of the nodes being explored.
    // In each iteration, it pops out a node from the queue, checks if it's the goal node, expands its neighbors,
    // and updates their cost if a better path is found.
    // The function returns when the goal node is found or it has iterated N times.

    LRTAStar::PointQueuePointSetPair openClosedPair;
    
    LRTAStar::PointQueue open; // priority queue of open nodes

    open.emplace(this->heuristic(xI), xI);

    LRTAStar::PointSet closed; // list of closed nodes
    PointVector path;

    std::map<LRTAStar::Point, LRTAStar::Point> parent;
    parent[xI] = xI;

    int count = 0;

    std::map<LRTAStar::Point, double> g_table;
    g_table[xI] = 0.0;
    g_table[_xG] = LRTAStar::INF;
    
    std::cout << "Starting current search iteration at: " << std::endl;
    std::cout << "xI: " << xI.first << ", " << xI.second << std::endl;

    while(!open.empty())
    {

        LRTAStar::Point s = open.top().second;
        // _xC = s;
        open.pop();
        closed.insert(s);
         
        _plot.plot_animation_repeated_astar("Learning Real-Time A*", closed, path, _repeatedCount, false);

        count++;

        if(s == _xG)
        {   
            _closed.insert(closed.begin(), closed.end());   
            std::cout << "Goal found!" << std::endl;
            path = this->extractPath(xI, parent);
            PointSet pathSet(path.begin(), path.end());
            openClosedPair.first = open;
            openClosedPair.second = pathSet;

            _plot.plot_animation_repeated_astar("Learning Real-Time A*", closed, path, _repeatedCount, false);

            return openClosedPair;
            
        }

        LRTAStar::PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            if(closed.find(s_next) == closed.end())
            {
                double new_cost = g_table[s] + this->cost(s, s_next);

                // Insert is used as it does not overwrite the value if the key already exists.
                auto result = g_table.insert({s_next, LRTAStar::INF});

                if(new_cost < g_table[s_next])
                {
                    g_table[s_next] = new_cost;
                    parent[s_next] = s;
                    double g = g_table[s_next] + _h_table[s_next];
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
    openClosedPair.second = closed;

    _closed.insert(closed.begin(), closed.end());   

    
    
    return openClosedPair;
}


LRTAStar::PointVector LRTAStar::extractPath(LRTAStar::Point &xS, std::map<LRTAStar::Point, LRTAStar::Point> &parent)
{
    LRTAStar::PointVector path;
    
    LRTAStar::Point s = _xG;
    path.push_back(s);
    while (s != xS)
    {
        // std::cout << "s: " << s.first << ", " << s.second << std::endl;
        s = parent[s];
        path.push_back(s);
    }

    return path;
}

LRTAStar::PointWithPriority LRTAStar::calcSmallestH(std::map<LRTAStar::Point, double> &h_value)
{
    LRTAStar::PointWithPriority s_with_smallest_h = *h_value.begin();  // Start with the first pair in the map
    
    // Iterate over the map to find the pair with the smallest fValue
    for (const auto& pair : h_value) {
        if (pair.second < s_with_smallest_h.second) {
            s_with_smallest_h = pair;
        }
    }

    return s_with_smallest_h;  
    
}