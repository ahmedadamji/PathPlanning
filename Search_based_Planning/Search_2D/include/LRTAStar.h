#ifndef LRTASTAR_H
#define LRTASTAR_H

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


class LRTAStar: public AStar
{
    public:

        typedef std::pair<Point, double> PointWithPriority;


        /**
         * @brief Construct a new LRTAStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        LRTAStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type) : AStar(env, plot, xI, xG, heuristic_type) {};

        /**
         * @brief Initialize the Anytime Repairing A* algorithm
         * 
         * This function initializes the parameters used in the Anytime Repairing A* algorithm.
         * 
         * @param N: The number of nodes to be expanded in each iteration.
         */
        void init(int N);

        /**
         * @brief This is the main function of the Learning Real-Time A* algorithm
         *
         * This function runs the LRTA* algorithm and returns the path and visited nodes.
         * 
         * @param N: The number of nodes to be expanded in each iteration.
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair searching(int N);

        /**
         * @brief Extract the path from the closed set
         * 
         * @param xS: The start point of the current iteration
         * @param h_value: The h_value table for the current iteration
         * @return std::pair<Point, PointVector>: The start point and the path
         */
        std::pair<Point, PointVector> extractPathInClose(LRTAStar::Point &xS, std::map<Point, double> &h_value);

        /**
         * @brief Performs an iteration of the LRTA* algorithm
         * 
         * @param closed set for the current iteration
         * @return h_value table for the current iteration
         */
        std::map<Point, double> iteration(LRTAStar::PointSet &closed);


        /**
         * @brief Run the A* algorithm with repeated forward and backward search
         * 
         * This function runs the A* algorithm with repeated forward and backward search and returns the path and visited nodes.
         * 
         * @param xI: The start point.
         * @return PointQueuePointSetPair: The open and closed nodes.
         */
        PointQueuePointSetPair repeatedSearching(LRTAStar::Point &xI);

        /**
         * @brief Extract the path from the parent dictionary
         * 
         * This function extracts the path by backtracking from the goal point to the current point using the parent dictionary.
         * 
         * @param xS: The start point
         * @param parent: The parent dictionary
         * @return path: The path 
         */
        PointVector extractPath(Point &xS, std::map<Point, Point> &parent);

        /**
         * @brief Calculate the smallest h value
         * 
         * This function calculates the smallest h value.
         * 
         * @return PointWithPriority: The point with the smallest h value.
         */
        PointWithPriority calcSmallestH(std::map<Point, double> &h_value);

        
        private:

            int _N = 0;

            std::map<Point, double> _incons;
            // std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open; // priority queue of open nodes
            // This data structure is used here instead of priority_queue because it is easier to update the open set with the states in the incons set if there are the same states in the open set.

            std::map<Point, double> _h_table;

            Point _xS; // The start point of the current iteration




};

#endif  // LRTASTAR_H