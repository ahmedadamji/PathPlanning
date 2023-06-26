#ifndef BIDIRECTIONALASTAR_H
#define BIDIRECTIONALASTAR_H

#include "Plotting.h"
#include "Env.h"
#include "AStar.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <unordered_set>
#include <vector>
#include <map>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>
#include <math.h>

class BidirectionalAStar: public AStar
{
    public:
        /**
         * @brief Construct a new BidirectionalAStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        BidirectionalAStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type) : AStar(env, plot, xI, xG, heuristic_type) {};

        /**
         * @brief Initialize the BidirectionalAStar A* algorithm
         * 
         * This function initializes the parameters used in the BidirectionalAStar A* algorithm.
         * 
         */
        void init();
        
        /**
         * @brief Run the BidirectionalAStar algorithm
         * 
         * This function runs the BidirectionalAStar algorithm and returns the path and visited nodes.
         * 
         * @return pathVisitedPair: The path and visited nodes.
         */
        PointVectorPointSetPair searching();
        /**
         * @brief Get the heuristic value of a point to the goal
         * 
         * This function returns the heuristic value of a point.
         * 
         * @param s: The point
         * @param goal: The goal point
         * @return h: The heuristic value of the point
         */
        double heuristic(Point &s, Point &goal);

        /**
         * @brief Get the f value of a point from forward search
         * 
         * This function returns the f value of a point, which is the sum of the g value and the heuristic value.
         * f = g + h. (g: Cost to come, h: heuristic value)
         * 
         * @param s: The point
         * @return f: The f value of the point from forward search
         */
        double fValueForward(Point &s);

        /**
         * @brief Get the f value of a point from backward search
         * 
         * This function returns the f value of a point, which is the sum of the g value and the heuristic value.
         * f = g + h. (g: Cost to come, h: heuristic value)
         * 
         * @param s: The point
         * @return f: The f value of the point from backward search
         */
        double fValueBackward(Point &s);

        
        /**
         * @brief Extract the path from start to goal from the meeting point
         * 
         * This function extracts the path by backtracking from the goal point and start point to the meeting point and concatenating the two paths.
         * 
         * @return path: The path 
         */
        PointVector extractPath();


    private:

        Point _xCF; // current point from forward search
        Point _xCB; // current point from backward search
        Point _xM;  // Meeting point

        PointVector _path; // list of path points
        PointVector _path_forward; // list of path points from forward search
        PointVector _path_backward; // list of path points from backward search

        // These could ideally be unordered sets, to speed up lookup, but pairs are not hashable by default in C++.
        // This does not create problems with sets because set in C++ is implemented as a binary search tree and not a hash table.
        PointSet _closed_forward; // set of visited points from forward search
        PointSet _closed_backward; // set of visited points from backward search
        
        std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open_forward; // priority queue of open nodes from forward search
        std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open_backward; // priority queue of open nodes from backward search

        std::map<Point, Point> _parent_forward; // dict of parents of each node from forward search
        std::map<Point, Point> _parent_backward; // dict of parents of each node from backward search
        
        std::map<Point, double> _g_forward; // cost from start to current node
        std::map<Point, double> _g_backward; // cost from end to current node


};

#endif  // BidirectionalAStar_H