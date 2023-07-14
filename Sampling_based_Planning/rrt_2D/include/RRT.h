#ifndef RRT_H
#define RRT_H

#include "Plotting.h"
#include "Env.h"
#include "Utils.h"
#include"Node.h"
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

class RRT {
    public:
        // Define types.
        typedef std::pair<int, int> Point;
        typedef std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > PointQueue;
        typedef std::vector<Point> PointVector;
        typedef std::set<Point> PointSet;
        typedef std::unordered_set<Point> UnorderedPointSet;
        typedef std::pair<PointVector, PointVector> PointVectorPair;
        typedef std::pair<PointSet, PointSet> PointSetPair;
        typedef std::pair<PointVector, PointSet> PointVectorPointSetPair;
        typedef std::pair<PointQueue, PointVector> PointQueuePointVectorPair;
        typedef std::pair<PointQueue, PointSet> PointQueuePointSetPair;


        static constexpr double INF = std::numeric_limits<double>::infinity();


        /**
         * @brief Construct a new RRT object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        RRT(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type);

        /**
         * @brief Update the obstacle set
         * 
         */
        void update_obs();

        Point _xI; // start point
        Point _xG; // goal point
        Point _xC; // current point

        PointVector _uSet; // list of motions
        PointVector _path; // list of nodes in the path

        // These could ideally be unordered sets, to speed up lookup, but pairs are not hashable by default in C++.
        // This does not create problems with sets because set in C++ is implemented as a binary search tree and not a hash table.
        PointSet _closed; // list of closed nodes
        
        // Obstacle vectors.
        std::vector<std::vector<int>> obs_boundary;
        std::vector<std::vector<int>> obs_rectangle;
        std::vector<std::vector<int>> obs_circle;

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

#endif  // RRT_H