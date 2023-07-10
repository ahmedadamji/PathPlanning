// In Env.cpp
#include "Env.h"
#include <random>

Env::Env(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG) {
    this->x_range = x_range;
    this->y_range = y_range;
    this->xI = xI;
    this->xG = xG;
    this->obs = this->obs_map();
    this->obs_boundary = get_obs_boundary();
    this->obs_rectangle = get_obs_rectangle();
    this->obs_circle = get_obs_circle();
    this->motions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1} };
    // this->motions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };

}

void Env::update_obs(std::set<std::pair<int, int>> obs) {
    this->obs = obs;
}

std::set<std::pair<int, int>> Env::get_obs() {
    return this->obs;
}

std::pair<int, int> Env::get_xI() {
    return xI;
}

std::pair<int, int> Env::get_xG() {
    return xG;
}

std::set<std::pair<int, int>> Env::obs_map() {
    std::set<std::pair<int, int>> obs;
    int total_cells = x_range * y_range;

    int num_walls = 5;
    int min_wall_length = 5;
    int max_wall_length = 20;
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist_x(0, x_range - 1);
    std::uniform_int_distribution<int> dist_y(0, y_range - 1);
    std::uniform_int_distribution<int> dist_dx(min_wall_length, max_wall_length);
    std::uniform_int_distribution<int> dist_dy(min_wall_length, max_wall_length);
    
    for (int wall = 0; wall < num_walls; wall++) {
        
        std::pair<int, int> wI = {dist_x(mt), dist_y(mt)};

        // We use a min function to ensure we don't go out of the environment
        std::pair<int, int> wG = {std::min(wI.first + dist_dx(mt), x_range - 1),
                                  std::min(wI.second + dist_dy(mt), y_range - 1)}; 

        bool starts_horizontal = mt() % 2 == 0; // Randomly choose between expanding horizontally or vertically first

        if (starts_horizontal) {
            // Expand walls form random start to random goal
            for(int i = wI.first; i < wG.first; i++){
                if ((this->xG == std::make_pair(i, wI.second)) || (this->xI == std::make_pair(i, wI.second)))
                {
                    break;
                }
                obs.insert({i, wI.second});
            }
            for(int i = wI.second; i < wG.second; i++){
                if ((this->xG == std::make_pair(wG.first, i)) || (this->xI == std::make_pair(wG.first, i)))
                {
                    break;
                }
                obs.insert({wG.first, i});
            }
        }
        else {
            // Expand walls form random start to random goal
            for(int i = wI.second; i < wG.second; i++){
                if ((this->xG == std::make_pair(wI.first, i)) || (this->xI == std::make_pair(wI.first, i)))
                {
                    break;
                }
                obs.insert({wI.first, i});
            }
            for(int i = wI.first; i < wG.first; i++){
                if ((this->xG == std::make_pair(i, wG.second)) || (this->xI == std::make_pair(i, wG.second)))
                {
                    break;
                }
                obs.insert({i, wG.second});
            }
        }
    }
    

    return obs;
}

// Function to add a new obstacle to the environment
void Env::add_obs(std::pair<int, int> obs_point) {
    this->obs.insert(obs_point);
}

// Function to remove an obstacle from the environment
void Env::remove_obs(std::pair<int, int> obs_point) {
    this->obs.erase(obs_point);
}

std::vector<std::vector<int>> Env::get_obs_boundary()
{
    std::vector<std::vector<int>> obs_boundary = {
        {0, 0, 1, 30},
        {0, 30, 50, 1},
        {1, 0, 50, 1},
        {50, 1, 1, 30}
    };
    
    return obs_boundary;
}

std::vector<std::vector<int>> Env::get_obs_rectangle()
{
    std::vector<std::vector<int>> obs_rectangle = {
        {14, 12, 8, 2},
        {18, 22, 8, 3},
        {26, 7, 2, 12},
        {32, 14, 10, 2}
    };
    
    return obs_rectangle;
}

std::vector<std::vector<int>> Env::get_obs_circle()
{
    std::vector<std::vector<int>> obs_cir = {
        {7, 12, 3},
        {46, 20, 2},
        {15, 5, 2},
        {37, 7, 3},
        {37, 23, 3}
    };
    
    return obs_cir;
}