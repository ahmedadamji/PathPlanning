#ifndef ASTAR_H
#define ASTAR_H

#include "Plotting.h"
#include "Env.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <unordered_set>
#include <vector>
#include <map>
#include <string>
#include <stack>
#include <queue>
#include <utility>
#include <algorithm>
#include <math.h>


class AStar {
    public:
        // Define types.
        typedef std::pair<int, int> Point;
        typedef std::vector<Point> PointVector;
        typedef std::set<Point> PointSet;
        typedef std::unordered_set<Point> UnorderedPointSet;
        typedef std::pair<PointVector, PointVector> PointVectorPair;
        typedef std::pair<PointSet, PointSet> PointSetPair;
        typedef std::pair<PointVector, PointSet> PointVectorPointSetPair;


        static constexpr double INF = std::numeric_limits<double>::infinity();


        /**
         * @brief Construct a new AStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        AStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type);

        /**
         * @brief Run the A* algorithm
         * 
         * This function runs the A* algorithm and returns the path and visited nodes.
         * 
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair searching();

        /**
         * @brief 
         * 
         *
         * 
         * This function runs the A* algorithm with repeated forward and backward search and returns the path and visited nodes.
         * 
         * @param e: The weight of A* algorithm.
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair searchingRepeatedAStar(double &e);

        /**
         * @brief Run the A* algorithm with repeated forward and backward search
         * 
         * This function runs the A* algorithm with repeated forward and backward search and returns the path and visited nodes.
         * 
         * @param xI: The start point.
         * @param xG: The goal point.
         * @param e: The weight of A* algorithm.
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair repeatedSearching(AStar::Point &xI, AStar::Point &xG, double &e);

        /**
         * @brief Get Neighbours of a point
         * 
         * This function returns the neighbours of a point that are not obstacles.
         * 
         * @param s: The state
         * @return neighbours: The neighbours of the state
         */
        PointVector getNeighbours(Point &s);

        /**
         * @brief Get the cost of travelling from one point to another
         * 
         * This function returns the cost of travelling from one point to another.
         * 
         * @param s_start: The starting point
         * @param s_goal: The goal point
         * @return cost: The cost of travelling from one point to another
         */
        double cost(Point &s_start, Point &s_goal);

        /**
         * @brief Check if the line between two points is in collision
         * 
         * This function checks if the line segment between two points is in collision with the obstacles.
         * 
         * @param s_start: The starting point
         * @param s_goal: The goal point
         * @return isColliding: True if the line segment is in collision, false otherwise
         */
        bool isCollision(Point &s_start, Point &s_goal);

        /**
         * @brief Get the f value of a point
         * 
         * This function returns the f value of a point, which is the sum of the g value and the heuristic value.
         * f = g + h. (g: Cost to come, h: heuristic value)
         * 
         * @param s: The point
         * @return f: The f value of the point
         */
        double fValue(Point &s);

        /**
         * @brief Extract the path from the parent dictionary
         * 
         * This function extracts the path by backtracking from the goal point to the start point using the parent dictionary.
         * 
         * @param parent: The parent dictionary
         * @return path: The path 
         */
        PointVector extractPath(std::map<Point, Point> &parent);

        /**
         * @brief Get the heuristic value of a point
         * 
         * This function returns the heuristic value of a point.
         * 
         * @param s: The point
         * @return h: The heuristic value of the point
         */
        double heuristic(Point &s);


        Point _xI; // start point
        Point _xG; // goal point
        Point _xC; // current point

        PointVector _uSet; // list of motions
        PointVector _path; // list of nodes in the path

        // These could ideally be unordered sets, to speed up lookup, but pairs are not hashable by default in C++.
        // This does not create problems with sets because set in C++ is implemented as a binary search tree and not a hash table.
        PointSet _closed; // list of closed nodes
        PointSet _obs; // set of obstacles

        std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open; // priority queue of open nodes
        // std::queue<Point> _open;
        // std::stack<Point> _open;
        
        std::map<Point, Point> _parent; // dict of parents of each node
        std::map<Point, double> _g; // cost from start to current node

        std::string _heuristicType; // type of heuristic

        int _repeatedCount = 0; // count of repeated A* iterations

        // added private references
        Env& _env; 
        Plotting& _plot;




};

#endif  // ASTAR_H