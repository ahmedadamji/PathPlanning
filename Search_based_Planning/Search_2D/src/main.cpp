#include "Plotting.h"
#include "Env.h"
#include "AStar.h"
#include "BFS.h"
#include "DFS.h"
#include "Dijkstra.h"
#include "BestFirst.h"
#include "BidirectionalAStar.h"
#include "ARAStar.h"
#include <opencv2/opencv.hpp>

int main()
{
    std::pair<int, int> xI = {5, 5}; // Define xI directly.
    std::pair<int, int> xG = {35, 25}; // Define xG directly.

    // Initialize environment with its range and starting and ending points.
    Env env(40, 40, xI, xG);

    // Setting the cell size for plotting.
    int cell_size = 15;
    
    // Initialize a Plotting object.
    Plotting plot(env.x_range, env.y_range, env.get_xI(), env.get_xG(), env.get_obs(), cell_size);

    // Update the obstacle set in the Plotting object.
    plot.update_obs(env.get_obs());

    // Plot grid.
    plot.plot_grid();

    // // Initialize and run the A* algorithm.
    // // AStar astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // AStar astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // AStar::PointVectorPointSetPair pathVisitedPair = astar.searching();

    // // Initialize and run the BFS algorithm.
    // // BFS bfs(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // BFS bfs(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // BFS::PointVectorPointSetPair pathVisitedPair = bfs.searching();

    // // Initialize and run the DFS algorithm.
    // // BFS dfs(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // DFS dfs(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // DFS::PointVectorPointSetPair pathVisitedPair = dfs.searching();

    // // Initialize and run the Dijkstra algorithm.
    // // Dijkstra dijkstra(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // Dijkstra dijkstra(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // Dijkstra::PointVectorPointSetPair pathVisitedPair = dijkstra.searching();

    // // Initialize and run the BestFirst algorithm.
    // // BestFirst bestfirst(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // BestFirst bestfirst(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // BestFirst::PointVectorPointSetPair pathVisitedPair = bestfirst.searching();

    // // Initialize and run the BidirectionalAStar algorithm.
    // // BidirectionalAStar bidirectional_astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // BidirectionalAStar bidirectional_astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // BidirectionalAStar::PointVectorPointSetPair pathVisitedPair = bidirectional_astar.searching();

    // // Initialize and run the Repeated A* algorithm.
    // // AStar astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // AStar astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // double e = 2.5;
    // AStar::PointVectorPointSetPair pathVisitedPair = astar.searchingRepeatedAStar(e);

    // Initialize and run the Anytime Repairing A* algorithm.
    // ARAStar arastar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    ARAStar arastar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    double e = 2.5;
    ARAStar::PointVectorPointSetPair pathVisitedPair = arastar.searching(e);

    return 0;
}
