#ifndef ENV_H
#define ENV_H

#include <set>
#include <utility>
#include <vector>

class Env {
public:
    
    Env(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG);

    void update_obs(std::set<std::pair<int, int>> new_obs);

    std::set<std::pair<int, int>> get_obs();

    std::pair<int, int> get_xI();

    std::pair<int, int> get_xG();

    void add_obs_circle(std::pair<int, int> obs_point, int radius);

    void add_obs_rectangle(std::pair<int, int> obs_point, int width, int height);

    void remove_obs_circle(std::pair<int, int> obs_point);

    void remove_obs_rectangle(std::pair<int, int> obs_point);

    std::vector<std::vector<int>> get_obs_boundary();

    std::vector<std::vector<int>> get_obs_rectangle();

    std::vector<std::vector<int>> get_obs_circle();



    int x_range, y_range;

    std::pair<int, int> xI, xG;

    // std::vector<std::pair<int, int>> motions;

    std::vector<std::vector<int>> obs_boundary;
    std::vector<std::vector<int>> obs_rectangle;
    std::vector<std::vector<int>> obs_circle;
};

#endif  // ENV_H
