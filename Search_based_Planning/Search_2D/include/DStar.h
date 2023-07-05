#ifndef DSTAR_H
#define DSTAR_H

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
#include <stack>
#include <queue>
#include <utility>
#include <algorithm>
#include <math.h>
#include <thread>
#include <atomic>

/**
 * @brief This is the D* Lite algorithm class
 * 
 */
class DStar: public AStar
{
    public:

        typedef std::pair<Point, double> PointWithPriority;


        /**
         * @brief Construct a new DStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        DStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type) : AStar(env, plot, xI, xG, heuristic_type) {};

        /**
         * @brief Initialize the D* algorithm
         * 
         * This function initializes the parameters used in the D* algorithm.
         * 
         */
        void init();

        /**
         * @brief 
         * 
         *
         * 
         * This function runs the D* algorithm and returns the path and visited nodes.
         * 
         * @param e: The weight of D* algorithm.
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair searching();

        /**
         * @brief This function computes the path for the D* algorithm.
         * 
         */
        void computePath();

        /**
         * @brief This function updates the vertex.
         * 
         * @param s 
         */
        void updateVertex(Point s);

        /**
         * @brief This function calculates the key of the current state.
         * 
         * @param s 
         * @return std::pair<double,double> The key of the current state.
         */
        std::pair<double,double> calculateKey(Point s);

        /**
         * @brief This function returns the state with the smallest value in _U.
         * 
         * @return std::pair<DStar::Point, std::pair<double,double>> The state with the smallest value in _U.
         */
        std::pair<DStar::Point, std::pair<double,double>> topKey();

        /**
         * @brief Extract the path
         * 
         * This function extracts the path by finding the state with the smallest g value for neighbors of each state until the goal state is reached.
         * 
         * @return path: The path 
         */
        PointVector extractPath();

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
         * @brief Check for input
         * 
         * This function checks for input from the user.
         * 
         */
        void checkForInput();
        
        private:

            int _repeatedCount = 0;

            std::map<Point, double> _incons;
            // // std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open; // priority queue of open nodes
            // // This data structure is used here instead of priority_queue because it is easier to update the open set with the states in the incons set if there are the same states in the open set.
            // std::map<Point, double> _open;

            // The open set is a set of points without priority.
            std::set<Point> _open;

            // std::map<Point, std::string> _t;
            // std::map<Point, double> _h;
            // std::map<Point, double> _k;
            std::map<Point, double> _g;
            std::map<Point, double> _rhs;
            std::map<Point, std::pair<double,double>> _U;

            Point _xS;

            int _count = 0;

            // This stands for "key modifier".
            // It's a global counter that's incremented whenever the start node is moved.
            double _km = 0.0;

            static std::atomic<bool> stopLoop;




};

#endif  // DSTAR_H