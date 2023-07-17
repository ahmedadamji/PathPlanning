// In Env.cpp
#include "Env.h"
#include <random>

Env::Env(int x_range, int y_range, std::pair<int, int> xI, std::pair<int, int> xG) {
    this->x_range = x_range;
    this->y_range = y_range;
    this->xI = xI;
    this->xG = xG;
    this->obs_boundary = get_obs_boundary();
    this->obs_rectangle = get_obs_rectangle();
    this->obs_circle = get_obs_circle();
    // this->motions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1} };
    // // this->motions = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };

}

std::pair<int, int> Env::get_xI() {
    return xI;
}

std::pair<int, int> Env::get_xG() {
    return xG;
}

// Function to add a new circle obstacle to the environment
void Env::add_obs_circle(std::pair<int, int> obs_point, int radius) {
    this->obs_circle.push_back({obs_point.first, obs_point.second, radius});
}

// Function to add a new rectangle obstacle to the environment
void Env::add_obs_rectangle(std::pair<int, int> obs_point, int width, int height) {
    this->obs_rectangle.push_back({obs_point.first, obs_point.second, width, height});
}

// Funtion to remove a circle obstacle from the environment
void Env::remove_obs_circle(std::pair<int, int> obs_point) {
    for (int i = 0; i < this->obs_circle.size(); i++) {
        // Remove the obstacle if the point exists within the bounds of a circle obstacle
        if (std::hypot(this->obs_circle[i][0] - obs_point.first, this->obs_circle[i][1] - obs_point.second) <= this->obs_circle[i][2]) {
            this->obs_circle.erase(this->obs_circle.begin() + i);
            break;
        }
    }
}

// Function to remove a rectangle obstacle from the environment
void Env::remove_obs_rectangle(std::pair<int, int> obs_point) {
    for (int i = 0; i < this->obs_rectangle.size(); i++) {
        // Remove the obstacle if the point exists within the bounds of a rectangle obstacle
        if (this->obs_rectangle[i][0] <= obs_point.first && this->obs_rectangle[i][0] + this->obs_rectangle[i][2] >= obs_point.first && this->obs_rectangle[i][1] <= obs_point.second && this->obs_rectangle[i][1] + this->obs_rectangle[i][3] >= obs_point.second) {
            this->obs_rectangle.erase(this->obs_rectangle.begin() + i);
            break;
        }
    }
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