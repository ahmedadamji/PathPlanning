#include "Plotting.h"
#include "Env.h"
#include "AStar.h"
#include "BFS.h"
#include "DFS.h"
#include "Dijkstra.h"
#include "BestFirst.h"
#include "BidirectionalAStar.h"
#include "ARAStar.h"
#include "LRTAStar.h"
#include "RTAAStar.h"
#include "DStarLite.h"
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
    Plotting plot(env, cell_size);

    // Update the obstacle set in the Plotting object.
    plot.update_obs(env.get_obs());

    // Plot grid.
    plot.plot_grid();

    // // Initialize and run the A* algorithm.
    // // AStar astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // AStar astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // AStar::PointVectorPointSetPair pathVisitedPair = astar.searching();
    // plot.save_as_gif("../gifs/astar.gif");

    // // Initialize and run the BFS algorithm.
    // // BFS bfs(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // BFS bfs(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // BFS::PointVectorPointSetPair pathVisitedPair = bfs.searching();
    // plot.save_as_gif("../gifs/bfs.gif");

    // // Initialize and run the DFS algorithm.
    // // BFS dfs(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // DFS dfs(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // DFS::PointVectorPointSetPair pathVisitedPair = dfs.searching();
    // plot.save_as_gif("../gifs/dfs.gif");

    // // Initialize and run the Dijkstra algorithm.
    // // Dijkstra dijkstra(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // Dijkstra dijkstra(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // Dijkstra::PointVectorPointSetPair pathVisitedPair = dijkstra.searching();
    // plot.save_as_gif("../gifs/dijkstra.gif");

    // // Initialize and run the BestFirst algorithm.
    // // BestFirst bestfirst(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // BestFirst bestfirst(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // BestFirst::PointVectorPointSetPair pathVisitedPair = bestfirst.searching();
    // plot.save_as_gif("../gifs/bestfirst.gif");

    // // Initialize and run the BidirectionalAStar algorithm.
    // // BidirectionalAStar bidirectional_astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // BidirectionalAStar bidirectional_astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // BidirectionalAStar::PointVectorPointSetPair pathVisitedPair = bidirectional_astar.searching();
    // plot.save_as_gif("../gifs/bidirectional_astar.gif");

    // // Initialize and run the Repeated A* algorithm.
    // // AStar astar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // AStar astar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // double e = 2.5;
    // AStar::PointVectorPointSetPair pathVisitedPair = astar.searchingRepeatedAStar(e);
    // plot.save_as_gif("../gifs/repeated_astar.gif");

    // // Initialize and run the Anytime Repairing A* algorithm.
    // // ARAStar arastar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // ARAStar arastar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // double e = 2.5;
    // // As we can visualise, In this ARA* implementation, we avoid revisiting nodes that are consistent (i.e., their g-values match the cost of the shortest path found so far)
    // // and have already been added to the closed set. This is one of the optimizations that make ARA* more efficient than standard A* in certain scenarios.
    // ARAStar::PointVectorPointSetPair pathVisitedPair = arastar.searching(e);
    // plot.save_as_gif("../gifs/arastar.gif");

    // // Initialize and run the Learning Real Time A* algorithm.
    // // LRTAStar lrtastar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // LRTAStar lrtastar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // int N = 250; // The number of nodes to be expanded in each iteration.
    // LRTAStar::PointVectorPointSetPair pathVisitedPair = lrtastar.searching(N);
    // plot.save_as_gif("../gifs/lrtastar.gif");

    // // Initialize and run the Real Time Adaptive A* algorithm.
    // // RTAAStar rtaastar(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    // RTAAStar rtaastar(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    // int N = 240; // The number of nodes to be expanded in each iteration.
    // RTAAStar::PointVectorPointSetPair pathVisitedPair = rtaastar.searching(N);
    // plot.save_as_gif("../gifs/rtaastar.gif");

    // This is the D* Lite algorithm.
    // DStarLite dstarlite(env, plot, env.get_xI(), env.get_xG(), "manhattan");
    DStarLite dstarlite(env, plot, env.get_xI(), env.get_xG(), "euclidean");
    DStarLite::PointVectorPointSetPair pathVisitedPair = dstarlite.searching();
    plot.save_as_gif("../gifs/dstarlite.gif");

    return 0;
}
