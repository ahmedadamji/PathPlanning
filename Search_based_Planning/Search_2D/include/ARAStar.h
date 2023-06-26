#ifndef ARASTAR_H
#define ARASTAR_H

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


class ARAStar: public AStar
{
    public:

        typedef std::pair<Point, double> PointWithPriority;


        /**
         * @brief Construct a new ARAStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        ARAStar(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type) : AStar(env, plot, xI, xG, heuristic_type) {};

        /**
         * @brief Initialize the Anytime Repairing A* algorithm
         * 
         * This function initializes the parameters used in the Anytime Repairing A* algorithm.
         * 
         */
        void init();

        /**
         * @brief 
         * 
         *
         * 
         * This function runs the ARA* algorithm and returns the path and visited nodes.
         * 
         * @param e: The weight of ARA* algorithm.
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair searching(double &e);

        /**
         * @brief Run the ARA* algorithm with repeated forward and backward search
         * 
         * This function runs the ARA* algorithm with repeated forward and backward search and returns the path and visited nodes.
         * 
         * @return PointVectorPointSetPair: The path and visited nodes.
         */
        PointVectorPointSetPair improvePath();

        /**
         * @brief Update the epsilon value
         * 
         * This function updates the epsilon value.
         * 
         * @return e: The updated epsilon value
         */
        double updateEpsilon();

        /**
         * @brief Calculate the smallest f value
         * 
         * This function calculates the smallest f value.
         * 
         * @return PointWithPriority: The point with the smallest f value.
         */
        PointWithPriority calcSmallestF();

        /**
         * @brief Get the f value of a point
         * 
         * This function returns the f value of a point, which is the sum of the g value and the heuristic value.
         * f = g + h. (g: Cost to come, h: heuristic value)
         * 
         * @param s: The point
         * @return f: The f value of the point
         */
        double fValue(const Point &s);

        /**
         * @brief Get the heuristic value of a point
         * 
         * This function returns the heuristic value of a point.
         * 
         * @param s: The point
         * @return h: The heuristic value of the point
         */
        double heuristic(const Point &s);
        
        private:

            int _repeatedCount = 0;

            std::map<Point, double> _incons;
            // std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > _open; // priority queue of open nodes
            // This data structure is used here instead of priority_queue because it is easier to update the open set with the states in the incons set if there are the same states in the open set.
            std::map<Point, double> _open;

            double _e = 0.0;




};

#endif  // ARASTAR_H