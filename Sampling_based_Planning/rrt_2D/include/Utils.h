#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <utility>
#include "Env.h"
#include "Node.h"

class Utils {
public:
    Utils(Env &env);
    bool check_collision_with_obs(Node &node);
    std::vector<std::vector<Eigen::Vector2d>> get_obs_vertex();
    bool is_intersect_rect(Node &start, Node &end, std::pair<Eigen::Vector2d, Eigen::Vector2d> ray, Eigen::Vector2d a, Eigen::Vector2d b);
    bool is_intersect_circle(std::pair<Eigen::Vector2d, Eigen::Vector2d> ray, Eigen::Vector2d a, double r);
    bool check_collision(Node &start, Node &end);
    std::pair<Eigen::Vector2d, Eigen::Vector2d> get_ray(Node &start, Node &end);
    double get_dist(Node &start, Node &end);

    Env &env;
    const double delta = 0.5;
};

#endif /* UTILS_H */
