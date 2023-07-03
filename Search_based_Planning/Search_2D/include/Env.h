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


    int x_range, y_range;
    std::pair<int, int> xI, xG;
    std::vector<std::pair<int, int>> motions;
    std::set<std::pair<int, int>> obs;
    std::set<std::pair<int, int>> obs_map();

    void add_obs(std::pair<int, int> obs_point);
    void remove_obs(std::pair<int, int> obs_point);
};

#endif  // ENV_H
