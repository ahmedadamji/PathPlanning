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
        typedef std::pair<double, double> Point;
        typedef std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>> > PointQueue;
        typedef std::vector<Point> PointVector;
        typedef std::vector<Node> NodeVector;
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
         * @param utils
         * @param xI 
         * @param xG 
         * @param step_len
         * @param goal_sample_rate
         * @param iter_max
         */
        RRT(Env &env, Plotting &plot, Utils &utils, Point xI, Point xG, double step_len, double goal_sample_rate, double iter_max);

        /**
         * @brief Planning function
         * 
         * @return PointVector: The planned path
         */
        PointVector Planning();

        /**
         * @brief Generate a random node
         * 
         * @return Node: The random node
         */
        Node generate_random_node();

        /**
         * @brief Get the nearest node object to the given node from the given node list
         * 
         * @param node_list 
         * @param n 
         * @return Node 
         */
        Node nearest_neighbor(NodeVector node_list, Node n);

        /**
         * @brief Get a new state which is in the path from n1 to n2 bounded by step length
         * 
         * @param n1 
         * @param n2 
         * @return Node 
         */
        Node new_state(Node n1, Node n2);

        /**
         * @brief Update the obstacle set
         * 
         */
        void update_obs();
        
        /**
         * @brief Exrect the path from the given end node to the start node
         * 
         * @param node_end 
         * @return PointVector 
         */
        PointVector extract_path(Node node_end);

        Point _xI; // start point
        Node _xI_node; // start node
        Point _xG; // goal point
        Node _xG_node; // goal node

        NodeVector _vertex;

        int _x_range, _y_range; // range of x and y

        PointVector _path; // list of nodes in the path
        
        // Obstacle vectors.
        std::vector<std::vector<int>> obs_boundary;
        std::vector<std::vector<int>> obs_rectangle;
        std::vector<std::vector<int>> obs_circle;

        // Parameters for RRT.
        double _step_len, _goal_sample_rate, _iter_max;

        // Instances of other classes.
        Env& _env; 
        Plotting& _plot;
        Utils &_utils;




};

#endif  // RRT_H