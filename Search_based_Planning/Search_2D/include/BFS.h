#ifndef BFS_H
#define BFS_H

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

class BFS: public AStar
{
    public:
        /**
         * @brief Construct a new AStar object 
         * 
         * @param env 
         * @param plot 
         * @param xI 
         * @param xG 
         * @param heuristic_type 
         */
        BFS(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type) : AStar(env, plot, xI, xG, heuristic_type) {};

        /**
         * @brief Run the BFS algorithm
         * 
         * This function runs the BFS algorithm and returns the path and visited nodes.
         * 
         * @return pathVisitedPair: The path and visited nodes.
         */
        PointSetPair searching();



    private:

};

#endif  // BFS_H