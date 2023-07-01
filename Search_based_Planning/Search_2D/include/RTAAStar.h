#ifndef RTAASTAR_H
#define RTAASTAR_H

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


class RTAAStar: public AStar
{
    public:

        typedef std::pair<Point, double> PointWithPriority;


        /**
         * @brief Construct a new RTAAStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        RTAAStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type) : AStar(env, plot, xI, xG, heuristic_type) {};

        /**
         * @brief Initialize the Anytime Repairing RTAA* algorithm
         * 
         * This function initializes the parameters used in the Anytime Repairing RTAA* algorithm.
         * 
         * @param N: The number of nodes to be expanded in each iteration.
         */
        void init(int N);

        /**
         * @brief This is the main function of the Real-Time Adaptive RTAA* algorithm
         *
         * This function runs the RTAA* algorithm and returns the path and visited nodes.
         * 
         * @param N: The number of nodes to be expanded in each iteration.
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair searching(int N);

        /**
         * @brief Extract the path from the closed set
         * 
         * @param xS: The start point of the current iteration
         * @param xN: The next point of the current iteration
         * @param h_value: The h_value table for the current iteration
         * @return std::pair<Point, PointVector>: The start point and the path
         */
        std::pair<Point, PointVector> extractPathInClose(RTAAStar::Point &xS, RTAAStar::Point &xN, std::map<Point, double> &h_value);

        /**
         * @brief Computes the h_value table given the open and closed sets
         * 
         * @param open set for the current iteration
         * @param closed set for the current iteration
         * @return std::pair<Point, std::map<Point, double>>: The current point and the h_value table
         */
        std::pair<Point, std::map<Point, double>> computeHValue(RTAAStar::PointQueue &open, RTAAStar::PointSet &closed);


        /**
         * @brief Run the RTAA* algorithm with repeated forward and backward search
         * 
         * This function runs the RTAA* algorithm with repeated forward and backward search and returns the path and visited nodes.
         * 
         * @param xI: The start point.
         * @return PointQueuePointVectorPair: The open and closed nodes.
         */
        PointQueuePointVectorPair repeatedSearching(RTAAStar::Point &xI);

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

        /**
         * @brief Calculate the largest h value
         * 
         * This function calculates the largest h value.
         * 
         * @return PointWithPriority: The point with the largest h value.
         */
        PointWithPriority calcLargestH(std::map<Point, double> &h_value);

        
        private:

            int _N = 0;

            std::map<Point, double> _incons;
            // std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open; // priority queue of open nodes
            // This data structure is used here instead of priority_queue because it is easier to update the open set with the states in the incons set if there are the same states in the open set.

            std::map<Point, double> _h_table;

            std::map<Point, double> _g_table;

            std::map<Point, Point> _parent;

            Point _xS; // The start point of the current iteration




};

#endif  // RTAASTAR_H