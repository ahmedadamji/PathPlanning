#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <utility>
#include "Env.h"
#include "Node.h"

class Utils {
public:
    Utils(Env &env);
    bool check_collision(Node &node);
    std::pair<Eigen::Vector2d, Eigen::Vector2d> get_ray(Node &start, Node &end);
    double get_dist(Node &start, Node &end);

private:
    Env &env;
    const double delta = 0.5;
};

#endif /* UTILS_H */
