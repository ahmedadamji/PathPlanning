#include "DStarLite.h"
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
#include <thread>
#include <atomic>

void DStarLite::init()
{
    // _parent[_xI] = _xI;
    // _g[_xI] = 0.0;
    // _g[_xG] = DStarLite::INF;

    // _open.emplace(_xI, this->fValue(_xI));

    // _xS = _xI;

    for (int i = 0; i < _env.x_range; i++)
    {
        for (int j = 0; j < _env.y_range; j++)
        {
            DStarLite::Point s = std::make_pair(i, j);
            // _t[s] = "NEW";
            // _k[s] = 0.0;
            // _h[s] = DStarLite::INF;
            _rhs[s] = DStarLite::INF;
            _g[s] = DStarLite::INF;

        }
    }

    // _h[_xG] = 0.0;
    _rhs[_xG] = 0.0;
    _U[_xG] = this->calculateKey(_xG);
    _closed.clear();
    _open.clear();
    _count = 0;

}

DStarLite::PointVectorPointSetPair DStarLite::searching()
{
    DStarLite::PointVectorPointSetPair pathVisitedPair;
    DStarLite::PointVector path;
    DStarLite::PointSet closed;
    
    _repeatedCount = 0;

    this->init();

    this->computePath();
    path = this->extractPath();
    pathVisitedPair = std::make_pair(path, _closed);

    _plot.plot_animation_repeated_astar("D* Lite", pathVisitedPair.second, pathVisitedPair.first, _repeatedCount, false);
            
    // Start a thread which listens to keyboard input
    std::thread inputThread(&DStarLite::checkForInput, this);

    while (!this->stopLoop)
    {
        cv::Point click_coordinates = _plot.get_click_coordinates("D* Lite");
        DStarLite::Point click_coordinates_pair = std::make_pair(click_coordinates.x, click_coordinates.y);
        this->_env.update_obs(_plot._env.get_obs());
        _obs = _env.get_obs();

        _repeatedCount++;

        DStarLite::Point s_curr = _xI;
        DStarLite::Point s_last = _xI;
        int i = 0;
        _path.clear();
        path.clear();
        path.push_back(_xI);

        // The algorithm enters a loop, where it navigates from the current state towards the goal.
        // It does this by considering all neighbors of the current state, and selecting the one with the lowest cost.

        while (s_curr != _xG)
        {
            std::map<Point, double> h_neighbours;

            DStarLite::PointVector neighbours = this->getNeighbours(s_curr);
            for(auto s_next : neighbours)
            {
                // The h_neighbours values are updated based on the values in _g and cost to reach the neighbour from the current state.
                h_neighbours[s_next] = _g[s_next] + this->cost(s_curr, s_next);
            }
            // Finding the state with minimum cost in _h_neighbours:
            // The selected neighbor then becomes the current state.
            s_curr = std::min_element(h_neighbours.begin(), h_neighbours.end(), [](const auto& a, const auto& b) { return a.second < b.second; })->first;
            path.push_back(s_curr);

            // We only do this once because only one obstacle is added / removed at a time.
            if (i < 1)
            {
                _km += this->heuristic(s_last, s_curr);
                s_last = s_curr;
                // If an obstacle is found at the current state, we set the cost of moving to this state to infinity, effectively marking it as impassable.
                if (std::find(_obs.begin(), _obs.end(), click_coordinates_pair) != _obs.end())
                {
                    _g[click_coordinates_pair] = DStarLite::INF;
                    _rhs[click_coordinates_pair] = DStarLite::INF;
                }

                // Otherwise, we update the vertex as it just bebcame available to pass, and we do the same for all neighboring states.
                else
                {
                    this->updateVertex(click_coordinates_pair);
                }

                DStarLite::PointVector neighbours_click_coordinates = this->getNeighbours(click_coordinates_pair);
                for(auto s_next : neighbours_click_coordinates)
                {
                    this->updateVertex(s_next);
                }

                i++;

                _count++;
                _closed.clear();
                // We then recompute the _U, which is the set of states that need to be explored.
                this->computePath();
                // pathVisitedPair = std::make_pair(_path, _closed);
                // _plot.plot_animation_repeated_astar("D* Lite", pathVisitedPair.second, pathVisitedPair.first, _repeatedCount, false);


            }

        }

        // // Print the updated obstacle space size.
        // std::cout << "Obstacle space size: " << this->_env.get_obs().size() << std::endl;

        // std::cout << "Click coordinate x: " << click_coordinates.x << std::endl;
        // std::cout << "Click coordinate y: " << click_coordinates.y << std::endl;
        pathVisitedPair = std::make_pair(path, _closed);
        
        _plot.plot_animation_repeated_astar("D* Lite", pathVisitedPair.second, pathVisitedPair.first, _repeatedCount, false);



    }
    

    _plot.plot_animation_repeated_astar("D* Lite", pathVisitedPair.second, pathVisitedPair.first, _repeatedCount, true);
    cv::waitKey(0); // Wait until a key is pressed

    inputThread.join(); // Make sure to join the thread
    

    return pathVisitedPair;
}


void DStarLite::computePath()
{
    DStarLite::PointSet visited;

    std::pair<DStarLite::Point, std::pair<double,double>> s_with_v;
    std::pair<double,double> k_old;

    while(true)
    {
        // Finding the state with minimum key in _U (std::map<Point, std::pair<double,double>> _U;):
        s_with_v = this->topKey();

        // When you use the >= operator to compare two std::pair<double, double> objects,
        // the comparison is performed lexicographically,
        // meaning it first compares the first elements of the pairs.
        // If the first elements are equal, it then compares the second elements.
        // This condition tells us if the goal state has been reached because rhs and g values of the start state are consistent.
        // and the cost of the shortest path from the start state to the goal is smaller than the cost from the top state in _U to the goal.
        if (s_with_v.second >= this->calculateKey(_xI) && _rhs[_xI] == _g[_xI])
        {
            break;
        }

        k_old = s_with_v.second;
        // Remove the state with minimum key from _U.
        _U.erase(s_with_v.first);
        // Add the state with minimum key to _closed.
        _closed.insert(s_with_v.first);

        if (k_old < this->calculateKey(s_with_v.first))
        {
            // If the key of the state with minimum key is less than the new key of the state with minimum key,
            // then update the key of the state with minimum key.
            _U[s_with_v.first] = this->calculateKey(s_with_v.first);
        }
        else if (_g[s_with_v.first] > _rhs[s_with_v.first])
        {
            // If the g value of the state with minimum key is greater than the rhs value of the state with minimum key,
            // then update the g value of the state with minimum key.
            _g[s_with_v.first] = _rhs[s_with_v.first];
            // Update the g value of the state with minimum key's neighbours.
            DStarLite::PointVector neighbours = this->getNeighbours(s_with_v.first);
            for(auto s_next : neighbours)
            {
                this->updateVertex(s_next);
            }
        }
        else
        {
            // If the g value of the state with minimum key is less than or equal to the rhs value of the state with minimum key,
            // then update the g value of the state with minimum key's neighbours.
            _g[s_with_v.first] = DStarLite::INF;
            DStarLite::PointVector neighbours = this->getNeighbours(s_with_v.first);
            for(auto s_next : neighbours)
            {
                this->updateVertex(s_next);
            }
            this->updateVertex(s_with_v.first);
        }

        
    }

}

void DStarLite::updateVertex(Point s)
{
    // If s is not the goal state, then compute its rhs value.
    // The rhs value of a state s is the smallest g-value of any state s' that is a successor of s plus the cost of moving from s to s'.
    // The g-value of the goal state is 0 in this implementation.
    // _rhs can change due to new obstacles or changes in edge costs, causing g and rhs to become inconsistent.
    if (s != _xG)
    {
        _rhs[s] = DStarLite::INF;
        PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            _rhs[s] = std::min(_rhs[s], _g[s_next] + this->cost(s, s_next));
        }
    }
    
    // If s is a key in _U, remove it.
    // This is done because we want to store it in _U only if the g value and rhs value of s are not equal.
    // This is done in the following if condition.
    // We need to do that because if the g value and rhs value of s are equal,
    // then the g value of s is consistent with the rhs value of s.
    if (_U.count(s) != 0) {
        _U.erase(s);
    }

    // If the g-value and rhs-value of s are not equal, add the key of s to _U.
    // This is because the g-value of s is not consistent with the rhs-value of s.
    // This inconsistency is integral for D* Lite Lite, as resolving it leads to an updated shortest path.
    if (_g[s] != _rhs[s]) {
        _U[s] = this->calculateKey(s);
    }
}

std::pair<double,double> DStarLite::calculateKey(Point s)
{

    std::pair<double,double> key;
    // This is the minimum estimated cost of a path from the start node to the goal through this node.
    // It's a combination of the cost from the goal to reach this node (the g value)
    // and a heuristic estimate of the cost to reach the this node from the start (the heuristic from start to current state).
    // This value is used to prioritize nodes that are estimated to be on a shorter path.
    // _km is added to the heuristic in the key calculation to account for changes in the start position,
    // ensuring that the priority queue remains sorted correctly.
    key.first = std::min(_g[s], _rhs[s]) + this->heuristic(_xI, s) + _km;
    // This value is used as a tie-breaker when two nodes have the same first key value.
    // Its basically the cost to reach the goal from this node (the g value).
    // The nodes that reach the goal with a lower cost are prioritized.
    key.second = std::min(_g[s], _rhs[s]);
    return key;
}

std::pair<DStarLite::Point, std::pair<double,double>> DStarLite::topKey()
{
    DStarLite::Point s;

    // Here _U is of the form : std::map<Point, std::pair<double,double>>
    // We are finding the state with minimum key in _U.
    // The key for each state in _U is a pair of doubles.
    // The first double is the minimum estimated cost of a path from the start node to the goal through this node.
    // The second double is used as a tie-breaker when two nodes have the same first key value.
    s = std::min_element(_U.begin(), _U.end(), [](const auto& a, const auto& b) {
        if (a.second.first != b.second.first) { // compare first elements
            return a.second.first < b.second.first;
        } else { // in case of ties, compare second elements
            return a.second.second < b.second.second;
        }
    })->first;

    return std::make_pair(s, _U[s]);
}


DStarLite::PointVector DStarLite::extractPath()
{
    DStarLite::PointVector path;
    
    Point s = _xI;
    path.push_back(s);

    for (int k = 0; k < 100; k++)
    {
        std::map<Point, double> _g_neighbours;
        PointVector neighbours = this->getNeighbours(s);
        for(auto s_next : neighbours)
        {
            if(!this->isCollision(s, s_next))
            {
                // The g_neighbours values are updated based on the values in _g.
                _g_neighbours[s_next] = _g[s_next];
            }
        }
        // Finding the state with minimum cost in _g_neighbours:
        s = std::min_element(_g_neighbours.begin(), _g_neighbours.end(), [](const auto& a, const auto& b) { return a.second < b.second; })->first;
        path.push_back(s);
        if (s == _xG)
        {
            break;
        }
    }

    return path;
}

double DStarLite::heuristic(DStarLite::Point &s, DStarLite::Point &goal)
{
    // D* Lite uses a heuristic function to estimate the cost to reach the goal from a given node.
    // The quality of the heuristic greatly affects the efficiency of the D* Lite search.
    // A good heuristic function can guide the D* Lite algorithm to explore less unnecessary paths
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


std::atomic<bool> DStarLite::stopLoop(false);

void DStarLite::checkForInput()
{
    // This code is used to check if a key has been pressed on the keyboard in UNIX based systems

    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

    char c;
    while (!this->stopLoop) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            std::cout << "Input received: " << c << std::endl;
            this->stopLoop = true;
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
}