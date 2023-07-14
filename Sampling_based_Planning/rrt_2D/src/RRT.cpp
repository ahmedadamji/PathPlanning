#include "RRT.h"
#include "Plotting.h"
#include "Env.h"
#include "Utils.h"
#include "Node.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <unordered_set>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <algorithm>
#include <stack>
#include <math.h>

RRT::RRT(Env &env, Plotting &plot, Point xI, Point xG, std::string heuristic_type)
    : _env(env), _plot(plot), _xI(xI), _xG(xG), _heuristicType(heuristic_type)
{
    _uSet = _env.motions;
    this->obs_boundary = env.get_obs_boundary();
    this->obs_rectangle = env.get_obs_rectangle();
    this->obs_circle = env.get_obs_circle();

}

void RRT::update_obs()
{
    this->obs_boundary = this->_env.get_obs_boundary();
    this->obs_rectangle = this->_env.get_obs_rectangle();
    this->obs_circle = this->_env.get_obs_circle();
}