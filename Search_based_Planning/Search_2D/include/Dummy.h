#ifndef DUMMY_H
#define DUMMY_H

#include "Plotting.h"
#include "Env.h"
#include <vector>

class Dummy {
public:
    void run(Env &env, Plotting &plot);
private:
    void display_plots(Plotting &plot,
                   std::set<std::pair<int, int>> &visited,
                   std::set<std::pair<int, int>> &path);
    void expand_search_and_plot(int x, int y, double &expand_search, 
                                std::set<std::pair<int, int>> &visited, 
                                std::set<std::pair<int, int>> &path,
                                std::set<std::pair<int, int>> &obstacles, 
                                std::pair<int, int> &xI, 
                                std::pair<int, int> &xG, 
                                Plotting &plot,
                                bool is_horizontal);
};

#endif
