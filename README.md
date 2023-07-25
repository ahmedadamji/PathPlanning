# Path Planning Algorithms and Visualization in C++

---
## Overview
This repository contains the C++ Implementation of Path Planning Algorithms of some common path planning algorithms used in robotics, including Search-based algorithms and Sampling-based algorithms based on the Python Implementation by [Huiming Zhou](https://github.com/zhm-real).
The algorithms can be visualized using the custom plotting class, which includes the generation of a new environment during each run for the grid-based planners, and an interactive dynamic environment developed to showcase dynamic algorithms.
The related papers are listed in ZHM-REAL's original Python repository [Papers](https://github.com/zhm-real/PathPlanning#papers).


---
## Compilation and Build
Clone the repository:

```
git clone https://github.com/ahmedadamji/PathPlanning.git
cd PathPlanning
```

Make sure you have CMake installed. If not, install it from https://cmake.org/download/.

Create a build directory and navigate into it:

```
mkdir build
cd build
```
Run CMake to configure the build:

```
cmake ..
```
Build the project:


---
## Running the Programs
### Search_2D
To run the search-based algorithms (A*, Dijkstra, Best-First, etc.):

Open the terminal and navigate to the build directory.

Run the executable for the search_2D program:

```
./PathPlanning
```
The program will visualize the path planning algorithms on a 2D grid environment.

### RRT_2D
To run the sampling-based algorithm (RRT-Connect):

Open the terminal and navigate to the build directory.

Run the executable for the rrt_2D program:

```
./SamplingBasedPlanning2D
```
The program will visualize the RRT-Connect algorithm on a 2D grid environment.


Note: In each main file, you can comment/uncomment the specific algorithm instantiation to choose which algorithm to run.


---
## Dependencies
The following dependencies are required to build and run the programs:

OpenCV 2.4 or later: Install OpenCV from https://opencv.org/releases/ or use your package manager.
Please make sure you have installed the required dependencies before compiling and running the programs.



---

## Animations - Search Based Planning

<div align="center">

#### BFS
![BFS](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/bfs.gif)

#### DFS
![DFS](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/dfs.gif)

#### Dijkstra
![Dijkstra](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/dijkstra.gif)

#### Best-First Search
![Best-First](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/bestfirst.gif)

#### A* Search
![A*](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/astar.gif)

#### Repeated A* Search
![Repeated A*](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/repeated_astar.gif)

#### Bidirectional A* Search
![Bidirectional A*](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/bidirectional_astar.gif)

#### ARA* Search
![ARA*](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/arastar.gif)

#### LRTA* Search
![LRTA*](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/lrtastar.gif)

#### RTAA* Search
![RTAA*](https://github.com/ahmedadamji/PathPlanning/blob/main/Search_based_Planning/Search_2D/gifs/rtaastar.gif)

</div>


## Animations - Sampling-Based

<div align="center">

#### RRT 2D
![RRT 2D](https://github.com/ahmedadamji/PathPlanning/blob/main/Sampling_based_Planning/rrt_2D/gifs/rrt.gif)

#### Goal Biased RRT
![Goal Biased RRT](https://github.com/ahmedadamji/PathPlanning/blob/main/Sampling_based_Planning/rrt_2D/gifs/goal_biased_rrt.gif)

#### RRT Connect
![RRT Connect](https://github.com/ahmedadamji/PathPlanning/blob/main/Sampling_based_Planning/rrt_2D/gifs/rrt_connect.gif)

</div>



---
## Acknowledgments
Thanks to [Huiming Zhou](https://github.com/zhm-real) for providing the original Python implementation and related papers for the path planning algorithms.
